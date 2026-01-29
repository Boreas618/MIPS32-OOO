/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */

/**
 * Recovery Unit - Milestone 5.6
 *
 * Handles pipeline recovery for mispredictions and exceptions.
 *
 * Branch Misprediction Recovery:
 * 1. Identify mispredicted branch in ROB
 * 2. Flush all younger instructions (ROB, RS, LSQ)
 * 3. Restore RAT from checkpoint or reconstruct from ROB
 * 4. Redirect fetch to correct path
 *
 * Memory Order Violation Recovery:
 * 1. Detect when store address matches speculatively-executed load
 * 2. Flush from violating load onward
 * 3. Re-execute load and younger instructions
 */
module RecoveryUnit #(
    parameter NUM_PHYS_REGS = 64,
    parameter ROB_ENTRIES = 64,
    parameter CHECKPOINT_COUNT = 8
) (
    input  logic clk,
    input  logic rst,

    // ========== ROB Recovery Interface ==========
    input  logic rob_flush,
    input  logic [31:0] rob_flush_pc,
    input  logic [6:0] rob_flush_rob_idx,
    input  logic [2:0] rob_flush_checkpoint_id,

    // ========== Memory Violation Interface ==========
    input  logic mem_violation,
    input  logic [6:0] mem_violation_rob_idx,
    input  logic [31:0] mem_violation_pc,                         // PC of violating load for redirect

    // ========== Branch Resolution Interface (from BranchExecute) ==========
    input  logic branch_mispredicted,
    input  logic [31:0] branch_correct_pc,
    input  logic [6:0] branch_rob_idx,
    input  logic [2:0] branch_checkpoint_id,

    // ========== Recovery Outputs ==========
    // Global flush signal
    output logic flush,
    output logic [31:0] flush_pc,
    output logic [6:0] flush_rob_idx,
    output logic [2:0] flush_checkpoint_id,
    
    // Fetch redirect
    output logic redirect_valid,
    output logic [31:0] redirect_pc,
    
    // RAT restore
    output logic rat_restore,
    output logic [2:0] rat_restore_checkpoint,
    
    // FreeList restore
    output logic freelist_restore,
    output logic [2:0] freelist_restore_checkpoint,
    
    // Reservation Station flush
    output logic rs_flush,
    output logic [6:0] rs_flush_rob_idx,
    
    // LSQ flush
    output logic lsq_flush,
    output logic [6:0] lsq_flush_rob_idx,
    
    // Instruction Queue flush
    output logic iq_flush,

    // ========== Recovery State ==========
    output logic recovery_in_progress,
    output logic [1:0] recovery_type  // 00=none, 01=branch, 10=memory, 11=exception
);

    // Recovery types
    localparam RECOVERY_NONE = 2'b00;
    localparam RECOVERY_BRANCH = 2'b01;
    localparam RECOVERY_MEMORY = 2'b10;
    localparam RECOVERY_EXCEPTION = 2'b11;

    // State machine for recovery
    typedef enum logic [1:0] {
        IDLE,
        FLUSH_START,
        RESTORE_STATE,
        REDIRECT
    } recovery_state_t;
    
    recovery_state_t state;
    
    // Latched recovery information
    logic [31:0] recovery_pc;
    logic [6:0] recovery_rob_idx;
    logic [2:0] recovery_checkpoint;
    logic [1:0] recovery_reason;

    // Priority: ROB flush > memory violation > branch misprediction
    // (ROB flush comes from committed mispredicted branch)
    logic trigger_recovery;
    logic [31:0] trigger_pc;
    logic [6:0] trigger_rob_idx;
    logic [2:0] trigger_checkpoint;
    logic [1:0] trigger_reason;
    
    always_comb begin
        trigger_recovery = 1'b0;
        trigger_pc = '0;
        trigger_rob_idx = '0;
        trigger_checkpoint = '0;
        trigger_reason = RECOVERY_NONE;
        
        if (rob_flush) begin
            // ROB flush (committed mispredicted branch)
            trigger_recovery = 1'b1;
            trigger_pc = rob_flush_pc;
            trigger_rob_idx = rob_flush_rob_idx;
            trigger_checkpoint = rob_flush_checkpoint_id;
            trigger_reason = RECOVERY_BRANCH;
        end else if (mem_violation) begin
            // Memory order violation - re-execute from violating load
            trigger_recovery = 1'b1;
            trigger_pc = mem_violation_pc;  // PC of violating load for re-execution
            trigger_rob_idx = mem_violation_rob_idx;
            trigger_checkpoint = '0;  // Memory violations don't use checkpoints
            trigger_reason = RECOVERY_MEMORY;
        end
    end

    // Outputs
    always_comb begin
        // Default outputs
        flush = 1'b0;
        flush_pc = '0;
        flush_rob_idx = '0;
        flush_checkpoint_id = '0;
        redirect_valid = 1'b0;
        redirect_pc = '0;
        rat_restore = 1'b0;
        rat_restore_checkpoint = '0;
        freelist_restore = 1'b0;
        freelist_restore_checkpoint = '0;
        rs_flush = 1'b0;
        rs_flush_rob_idx = '0;
        lsq_flush = 1'b0;
        lsq_flush_rob_idx = '0;
        iq_flush = 1'b0;
        recovery_in_progress = (state != IDLE);
        recovery_type = (state != IDLE) ? recovery_reason : RECOVERY_NONE;
        
        case (state)
            IDLE: begin
                // Watch for recovery triggers
                if (trigger_recovery) begin
                    flush = 1'b1;
                    flush_pc = trigger_pc;
                    flush_rob_idx = trigger_rob_idx;
                    flush_checkpoint_id = trigger_checkpoint;
                end
            end
            
            FLUSH_START: begin
                // Assert all flush signals
                flush = 1'b1;
                flush_pc = recovery_pc;
                flush_rob_idx = recovery_rob_idx;
                flush_checkpoint_id = recovery_checkpoint;
                
                rs_flush = 1'b1;
                rs_flush_rob_idx = recovery_rob_idx;
                
                lsq_flush = 1'b1;
                lsq_flush_rob_idx = recovery_rob_idx;
                
                iq_flush = 1'b1;
            end
            
            RESTORE_STATE: begin
                // Restore RAT and FreeList from checkpoint (for branch misprediction)
                if (recovery_reason == RECOVERY_BRANCH) begin
                    rat_restore = 1'b1;
                    rat_restore_checkpoint = recovery_checkpoint;
                    
                    freelist_restore = 1'b1;
                    freelist_restore_checkpoint = recovery_checkpoint;
                end
            end
            
            REDIRECT: begin
                // Redirect fetch to correct PC
                redirect_valid = 1'b1;
                redirect_pc = recovery_pc;
            end
        endcase
    end

    // State machine
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            recovery_pc <= '0;
            recovery_rob_idx <= '0;
            recovery_checkpoint <= '0;
            recovery_reason <= RECOVERY_NONE;
        end else begin
            case (state)
                IDLE: begin
                    if (trigger_recovery) begin
                        state <= FLUSH_START;
                        recovery_pc <= trigger_pc;
                        recovery_rob_idx <= trigger_rob_idx;
                        recovery_checkpoint <= trigger_checkpoint;
                        recovery_reason <= trigger_reason;
                    end
                end
                
                FLUSH_START: begin
                    // Flush for one cycle, then restore state
                    state <= RESTORE_STATE;
                end
                
                RESTORE_STATE: begin
                    // Restore for one cycle, then redirect
                    state <= REDIRECT;
                end
                
                REDIRECT: begin
                    // Redirect for one cycle, then done
                    state <= IDLE;
                    recovery_reason <= RECOVERY_NONE;
                end
            endcase
        end
    end

endmodule
