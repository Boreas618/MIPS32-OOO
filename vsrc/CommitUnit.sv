/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */

/**
 * Commit Unit - Milestone 5.2
 *
 * Coordinates in-order retirement from the ROB.
 * Responsibilities:
 * - Commit 2-4 instructions per cycle from ROB head
 * - In-order: only commit if all older instructions committed
 * - Update architectural state (via RenameUnit committed RAT)
 * - Free old physical register mappings (via FreeList)
 * - Deallocate ROB entries
 * - Exception handling: commit up to excepting instruction, then trap
 */
module CommitUnit #(
    parameter COMMIT_WIDTH = 4,
    parameter NUM_PHYS_REGS = 64,
    parameter ROB_ENTRIES = 64
) (
    input  logic clk,
    input  logic rst,

    // ========== ROB Interface ==========
    // Commit requests from ROB
    input  logic [COMMIT_WIDTH-1:0] rob_commit_valid,
    input  logic [4:0] rob_commit_arch_rd [COMMIT_WIDTH-1:0],
    input  logic [5:0] rob_commit_phys_rd [COMMIT_WIDTH-1:0],
    input  logic [5:0] rob_commit_old_phys_rd [COMMIT_WIDTH-1:0],
    input  logic [COMMIT_WIDTH-1:0] rob_commit_reg_write,
    input  logic [COMMIT_WIDTH-1:0] rob_commit_store,
    input  logic [COMMIT_WIDTH-1:0] rob_commit_load,             // Is a load instruction
    input  logic [6:0] rob_commit_rob_idx [COMMIT_WIDTH-1:0],
    input  logic [31:0] rob_commit_pc [COMMIT_WIDTH-1:0],
    
    // ROB flush signals
    input  logic rob_flush,
    input  logic [31:0] rob_flush_pc,
    input  logic [6:0] rob_flush_rob_idx,
    input  logic [2:0] rob_flush_checkpoint_id,

    // ========== RenameUnit Interface (committed RAT update) ==========
    output logic [COMMIT_WIDTH-1:0] commit_valid,
    output logic [4:0] commit_arch_rd [COMMIT_WIDTH-1:0],
    output logic [5:0] commit_phys_rd [COMMIT_WIDTH-1:0],
    output logic [COMMIT_WIDTH-1:0] commit_reg_write,

    // ========== FreeList Interface (free old physical registers) ==========
    output logic [COMMIT_WIDTH-1:0] free_req,
    output logic [5:0] free_preg [COMMIT_WIDTH-1:0],

    // ========== Store Queue Interface (commit stores to memory) ==========
    output logic [COMMIT_WIDTH-1:0] store_commit,
    output logic [6:0] store_commit_rob_idx [COMMIT_WIDTH-1:0],

    // ========== Recovery Interface ==========
    output logic flush_pipeline,
    output logic [31:0] flush_target_pc,
    output logic [2:0] flush_checkpoint,

    // ========== Performance Counters ==========
    output logic [31:0] committed_insts,
    output logic [31:0] committed_stores,
    output logic [31:0] committed_loads
);

    // Internal tracking
    logic [31:0] inst_counter;
    logic [31:0] store_counter;
    logic [31:0] load_counter;

    // Pass through commit signals from ROB
    always_comb begin
        for (int i = 0; i < COMMIT_WIDTH; i++) begin
            commit_valid[i] = rob_commit_valid[i];
            commit_arch_rd[i] = rob_commit_arch_rd[i];
            commit_phys_rd[i] = rob_commit_phys_rd[i];
            commit_reg_write[i] = rob_commit_reg_write[i];
            
            // Free the old physical register for instructions that write registers
            // Only free if the old_phys_rd is valid (not an architectural register 0-31 initially)
            // and the instruction actually writes a register
            free_req[i] = rob_commit_valid[i] && rob_commit_reg_write[i] && 
                         (rob_commit_old_phys_rd[i] >= 6'd32);  // Only free renamed regs
            free_preg[i] = rob_commit_old_phys_rd[i];
            
            // Store commit signals
            store_commit[i] = rob_commit_valid[i] && rob_commit_store[i];
            store_commit_rob_idx[i] = rob_commit_rob_idx[i];
        end
    end

    // Recovery signals
    assign flush_pipeline = rob_flush;
    assign flush_target_pc = rob_flush_pc;
    assign flush_checkpoint = rob_flush_checkpoint_id;

    // Performance counter outputs
    assign committed_insts = inst_counter;
    assign committed_stores = store_counter;
    assign committed_loads = load_counter;

    // Count committed instructions
    always_ff @(posedge clk) begin
        if (rst) begin
            inst_counter <= '0;
            store_counter <= '0;
            load_counter <= '0;
        end else if (!rob_flush) begin
            // Count how many instructions committed this cycle
            logic [2:0] commit_count;
            logic [2:0] store_count;
            logic [2:0] load_count;
            
            commit_count = '0;
            store_count = '0;
            load_count = '0;
            
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (rob_commit_valid[i]) begin
                    commit_count = commit_count + 1;
                    if (rob_commit_store[i]) begin
                        store_count = store_count + 1;
                    end
                    if (rob_commit_load[i]) begin
                        load_count = load_count + 1;
                    end
                end
            end
            
            inst_counter <= inst_counter + {29'b0, commit_count};
            store_counter <= store_counter + {29'b0, store_count};
            load_counter <= load_counter + {29'b0, load_count};
        end
    end

endmodule
