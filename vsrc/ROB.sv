/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "BranchTypes.svh"

/**
 * Reorder Buffer (ROB) - Milestone 5.1
 *
 * Implements in-order commit for out-of-order execution.
 * Structure: Circular buffer with 64-128 entries.
 *
 * Operations:
 * - Allocate: Add entry at tail on dispatch
 * - Complete: Mark entry done when result ready (via CDB)
 * - Commit: Retire from head when complete and no exceptions
 * - Flush: Clear entries on misprediction/exception
 */
module ROB #(
    parameter ROB_ENTRIES = 64,
    parameter NUM_PHYS_REGS = 64,
    parameter DISPATCH_WIDTH = 4,
    parameter COMMIT_WIDTH = 4,
    parameter CDB_WIDTH = 4
) (
    input  logic clk,
    input  logic rst,

    // ========== Dispatch Interface (from RenameUnit) ==========
    input  logic [DISPATCH_WIDTH-1:0] dispatch_valid,
    input  logic [4:0] dispatch_arch_rd [DISPATCH_WIDTH-1:0],     // Architectural destination reg
    input  logic [5:0] dispatch_phys_rd [DISPATCH_WIDTH-1:0],     // Physical destination reg
    input  logic [5:0] dispatch_old_phys_rd [DISPATCH_WIDTH-1:0], // Old physical mapping (for free list)
    input  logic [DISPATCH_WIDTH-1:0] dispatch_reg_write,         // Has destination register
    input  logic [DISPATCH_WIDTH-1:0] dispatch_mem_write,         // Is a store instruction
    input  logic [DISPATCH_WIDTH-1:0] dispatch_mem_read,          // Is a load instruction
    input  logic [DISPATCH_WIDTH-1:0] dispatch_branch,            // Is a branch instruction
    input  logic [3:0] dispatch_branch_type [DISPATCH_WIDTH-1:0], // Branch type encoding
    input  logic [31:0] dispatch_pc [DISPATCH_WIDTH-1:0],         // PC for debugging/exceptions
    input  logic [31:0] dispatch_predicted_target [DISPATCH_WIDTH-1:0],
    input  logic [DISPATCH_WIDTH-1:0] dispatch_predicted_taken,
    input  logic [2:0] dispatch_checkpoint_id [DISPATCH_WIDTH-1:0], // Branch checkpoint ID
    input  logic [DISPATCH_WIDTH-1:0] dispatch_checkpoint_valid,

    // Dispatch outputs
    output logic [6:0] dispatch_rob_idx [DISPATCH_WIDTH-1:0],     // Allocated ROB indices
    output logic [DISPATCH_WIDTH-1:0] dispatch_rob_valid,         // Allocation successful
    output logic rob_full,                                         // Cannot accept more dispatches

    // ========== CDB Interface (mark complete) ==========
    input  logic [CDB_WIDTH-1:0] cdb_valid,
    input  logic [5:0] cdb_tag [CDB_WIDTH-1:0],                   // Physical register tag
    input  logic [31:0] cdb_data [CDB_WIDTH-1:0],                 // Result data
    input  logic [6:0] cdb_rob_idx [CDB_WIDTH-1:0],               // ROB index to mark complete

    // Branch completion from CDB
    input  logic cdb_branch_complete,
    input  logic cdb_branch_mispredicted,
    input  logic [31:0] cdb_branch_correct_pc,
    input  logic [6:0] cdb_branch_rob_idx,

    // ========== Commit Interface (to RenameUnit/FreeList/Memory) ==========
    output logic [COMMIT_WIDTH-1:0] commit_valid,
    output logic [4:0] commit_arch_rd [COMMIT_WIDTH-1:0],         // Architectural dest reg
    output logic [5:0] commit_phys_rd [COMMIT_WIDTH-1:0],         // Physical dest reg (update committed RAT)
    output logic [5:0] commit_old_phys_rd [COMMIT_WIDTH-1:0],     // Old physical reg (free to free list)
    output logic [COMMIT_WIDTH-1:0] commit_reg_write,             // Has register write
    output logic [COMMIT_WIDTH-1:0] commit_store,                  // Is a committed store
    output logic [COMMIT_WIDTH-1:0] commit_load,                   // Is a committed load
    output logic [6:0] commit_rob_idx [COMMIT_WIDTH-1:0],         // ROB indices being committed
    output logic [31:0] commit_pc [COMMIT_WIDTH-1:0],             // PC of committed instructions

    // ========== Recovery Interface ==========
    output logic flush,                                            // Flush pipeline
    output logic [31:0] flush_pc,                                  // PC to redirect to
    output logic [6:0] flush_rob_idx,                              // ROB index of flushing instruction
    output logic [2:0] flush_checkpoint_id,                        // Checkpoint to restore

    // ========== Store Commit Interface (to StoreQueue) ==========
    output logic [COMMIT_WIDTH-1:0] store_commit_valid,           // Stores ready to commit
    output logic [6:0] store_commit_rob_idx [COMMIT_WIDTH-1:0],   // ROB indices of committed stores

    // ========== Load/Store Queue Integration ==========
    // Load completion (for memory order tracking)
    input  logic load_complete_valid,
    input  logic [6:0] load_complete_rob_idx,
    
    // Store address ready (from StoreQueue)
    input  logic store_addr_valid,
    input  logic [6:0] store_addr_rob_idx,

    // ========== Status ==========
    output logic [$clog2(ROB_ENTRIES):0] rob_count,
    output logic rob_empty,
    output logic [6:0] rob_head,
    output logic [6:0] rob_tail
);

    localparam ROB_IDX_BITS = $clog2(ROB_ENTRIES);

    // ROB entry structure
    typedef struct packed {
        logic valid;                    // Entry is valid
        logic completed;                // Instruction has completed execution
        logic [4:0] arch_rd;            // Architectural destination register
        logic [5:0] phys_rd;            // Physical destination register
        logic [5:0] old_phys_rd;        // Old physical mapping (for freeing)
        logic reg_write;                // Has a register destination
        logic mem_write;                // Is a store instruction
        logic mem_read;                 // Is a load instruction
        logic branch;                   // Is a branch instruction
        logic [3:0] branch_type;        // Branch type
        logic branch_mispredicted;      // Branch was mispredicted
        logic [31:0] branch_correct_pc; // Correct PC for recovery
        logic [31:0] pc;                // PC of this instruction
        logic [31:0] predicted_target;  // Predicted branch target
        logic predicted_taken;          // Predicted taken/not-taken
        logic [2:0] checkpoint_id;      // Branch checkpoint ID
        logic checkpoint_valid;         // Has valid checkpoint
        logic exception;                // Has an exception
        logic [31:0] result;            // Result value (optional, for debugging)
    } rob_entry_t;

    // ROB storage
    rob_entry_t entries [ROB_ENTRIES-1:0];

    // Head and tail pointers (one extra bit for full/empty detection)
    logic [ROB_IDX_BITS:0] head_ptr;
    logic [ROB_IDX_BITS:0] tail_ptr;

    // Derived signals
    wire [ROB_IDX_BITS-1:0] head_idx = head_ptr[ROB_IDX_BITS-1:0];
    wire [ROB_IDX_BITS-1:0] tail_idx = tail_ptr[ROB_IDX_BITS-1:0];

    // Count tracking
    logic [$clog2(ROB_ENTRIES):0] count;
    
    // Full/empty detection
    wire ptr_match = (head_ptr[ROB_IDX_BITS-1:0] == tail_ptr[ROB_IDX_BITS-1:0]);
    wire wrap_bit_match = (head_ptr[ROB_IDX_BITS] == tail_ptr[ROB_IDX_BITS]);
    
    assign rob_empty = ptr_match && wrap_bit_match;
    assign rob_full = ptr_match && !wrap_bit_match;
    assign rob_count = count;
    assign rob_head = {1'b0, head_idx};
    assign rob_tail = {1'b0, tail_idx};

    // ==========================================================================
    // Dispatch Logic - Allocate entries at tail
    // ==========================================================================
    
    // Calculate how many entries we can allocate
    logic [2:0] dispatch_count;
    logic [DISPATCH_WIDTH-1:0] can_dispatch;
    logic [ROB_IDX_BITS:0] next_tail_ptr [DISPATCH_WIDTH:0];
    
    always_comb begin
        next_tail_ptr[0] = tail_ptr;
        dispatch_count = 0;
        
        for (int i = 0; i < DISPATCH_WIDTH; i++) begin
            // Check if we have space and dispatch is valid
            logic has_space;
            logic [ROB_IDX_BITS:0] tentative_tail;
            
            tentative_tail = next_tail_ptr[i] + 1;
            // Check if adding this entry would make the ROB full
            has_space = !((tentative_tail[ROB_IDX_BITS-1:0] == head_ptr[ROB_IDX_BITS-1:0]) && 
                         (tentative_tail[ROB_IDX_BITS] != head_ptr[ROB_IDX_BITS]));
            
            can_dispatch[i] = dispatch_valid[i] && has_space;
            
            if (can_dispatch[i]) begin
                dispatch_rob_idx[i] = {1'b0, next_tail_ptr[i][ROB_IDX_BITS-1:0]};
                next_tail_ptr[i+1] = next_tail_ptr[i] + 1;
                dispatch_count = dispatch_count + 1;
            end else begin
                dispatch_rob_idx[i] = '0;
                next_tail_ptr[i+1] = next_tail_ptr[i];
            end
        end
        
        dispatch_rob_valid = can_dispatch;
    end

    // ==========================================================================
    // Commit Logic - Retire entries from head
    // ==========================================================================
    
    logic [COMMIT_WIDTH-1:0] can_commit;
    logic [2:0] commit_count;
    logic [ROB_IDX_BITS:0] next_head_ptr [COMMIT_WIDTH:0];
    
    // Check for mispredicted branch at commit
    logic commit_misprediction;
    logic [6:0] misprediction_rob_idx;
    logic [31:0] misprediction_correct_pc;
    logic [2:0] misprediction_checkpoint_id;
    
    always_comb begin
        next_head_ptr[0] = head_ptr;
        commit_count = 0;
        commit_misprediction = 0;
        misprediction_rob_idx = '0;
        misprediction_correct_pc = '0;
        misprediction_checkpoint_id = '0;
        
        for (int i = 0; i < COMMIT_WIDTH; i++) begin
            logic [ROB_IDX_BITS-1:0] check_idx;
            logic entry_valid;
            logic entry_complete;
            logic is_mispredicted_branch;
            
            check_idx = next_head_ptr[i][ROB_IDX_BITS-1:0];
            entry_valid = entries[check_idx].valid;
            entry_complete = entries[check_idx].completed;
            is_mispredicted_branch = entries[check_idx].branch && entries[check_idx].branch_mispredicted;
            
            // Can commit if: entry is valid, complete, and no prior misprediction in this cycle
            can_commit[i] = entry_valid && entry_complete && !commit_misprediction &&
                           (next_head_ptr[i] != tail_ptr);  // Don't commit past tail
            
            if (can_commit[i]) begin
                commit_valid[i] = 1'b1;
                commit_arch_rd[i] = entries[check_idx].arch_rd;
                commit_phys_rd[i] = entries[check_idx].phys_rd;
                commit_old_phys_rd[i] = entries[check_idx].old_phys_rd;
                commit_reg_write[i] = entries[check_idx].reg_write;
                commit_store[i] = entries[check_idx].mem_write;
                commit_load[i] = entries[check_idx].mem_read;
                commit_rob_idx[i] = {1'b0, check_idx};
                commit_pc[i] = entries[check_idx].pc;
                
                // Store commit signals
                store_commit_valid[i] = entries[check_idx].mem_write;
                store_commit_rob_idx[i] = {1'b0, check_idx};
                
                next_head_ptr[i+1] = next_head_ptr[i] + 1;
                commit_count = commit_count + 1;
                
                // Check for misprediction - stop committing after mispredicted branch
                if (is_mispredicted_branch) begin
                    commit_misprediction = 1'b1;
                    misprediction_rob_idx = {1'b0, check_idx};
                    misprediction_correct_pc = entries[check_idx].branch_correct_pc;
                    misprediction_checkpoint_id = entries[check_idx].checkpoint_id;
                end
            end else begin
                commit_valid[i] = 1'b0;
                commit_arch_rd[i] = '0;
                commit_phys_rd[i] = '0;
                commit_old_phys_rd[i] = '0;
                commit_reg_write[i] = 1'b0;
                commit_store[i] = 1'b0;
                commit_load[i] = 1'b0;
                commit_rob_idx[i] = '0;
                commit_pc[i] = '0;
                store_commit_valid[i] = 1'b0;
                store_commit_rob_idx[i] = '0;
                next_head_ptr[i+1] = next_head_ptr[i];
            end
        end
    end
    
    // Flush outputs
    assign flush = commit_misprediction;
    assign flush_pc = misprediction_correct_pc;
    assign flush_rob_idx = misprediction_rob_idx;
    assign flush_checkpoint_id = misprediction_checkpoint_id;

    // ==========================================================================
    // ROB State Update
    // ==========================================================================
    
    always_ff @(posedge clk) begin
        if (rst) begin
            head_ptr <= '0;
            tail_ptr <= '0;
            count <= '0;
            
            for (int i = 0; i < ROB_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
                entries[i].completed <= 1'b0;
            end
        end else if (flush) begin
            // On flush: clear all entries after mispredicted branch
            // Keep head, reset tail to one past the mispredicted instruction
            tail_ptr <= misprediction_rob_idx[ROB_IDX_BITS-1:0] + 1;
            head_ptr <= next_head_ptr[COMMIT_WIDTH];
            
            // Invalidate all entries from (misprediction_rob_idx + 1) to tail
            for (int i = 0; i < ROB_ENTRIES; i++) begin
                // Check if this entry is younger than the mispredicted branch
                logic [ROB_IDX_BITS-1:0] idx;
                idx = i[ROB_IDX_BITS-1:0];
                
                // Clear the mispredicted entry itself
                if (idx == misprediction_rob_idx[ROB_IDX_BITS-1:0]) begin
                    entries[idx].valid <= 1'b0;
                    entries[idx].completed <= 1'b0;
                end else if (entries[idx].valid) begin
                    // Use ROB idx comparison for age - younger entries have larger indices
                    // (accounting for wraparound)
                    logic is_younger;
                    logic [ROB_IDX_BITS:0] entry_age, mispredict_age;
                    
                    // Calculate age relative to head
                    entry_age = {1'b0, idx} - {1'b0, head_idx};
                    mispredict_age = misprediction_rob_idx[ROB_IDX_BITS-1:0] - head_idx;
                    
                    is_younger = (entry_age > mispredict_age);
                    
                    if (is_younger) begin
                        entries[idx].valid <= 1'b0;
                        entries[idx].completed <= 1'b0;
                    end
                end
            end
            
            // Update count
            count <= misprediction_rob_idx[ROB_IDX_BITS-1:0] - head_idx + 1 - commit_count;
        end else begin
            // Normal operation
            
            // Dispatch: allocate new entries
            for (int i = 0; i < DISPATCH_WIDTH; i++) begin
                if (can_dispatch[i]) begin
                    logic [ROB_IDX_BITS-1:0] alloc_idx;
                    alloc_idx = dispatch_rob_idx[i][ROB_IDX_BITS-1:0];
                    
                    entries[alloc_idx].valid <= 1'b1;
                    entries[alloc_idx].completed <= 1'b0;
                    entries[alloc_idx].arch_rd <= dispatch_arch_rd[i];
                    entries[alloc_idx].phys_rd <= dispatch_phys_rd[i];
                    entries[alloc_idx].old_phys_rd <= dispatch_old_phys_rd[i];
                    entries[alloc_idx].reg_write <= dispatch_reg_write[i];
                    entries[alloc_idx].mem_write <= dispatch_mem_write[i];
                    entries[alloc_idx].mem_read <= dispatch_mem_read[i];
                    entries[alloc_idx].branch <= dispatch_branch[i];
                    entries[alloc_idx].branch_type <= dispatch_branch_type[i];
                    entries[alloc_idx].branch_mispredicted <= 1'b0;
                    entries[alloc_idx].branch_correct_pc <= 32'b0;
                    entries[alloc_idx].pc <= dispatch_pc[i];
                    entries[alloc_idx].predicted_target <= dispatch_predicted_target[i];
                    entries[alloc_idx].predicted_taken <= dispatch_predicted_taken[i];
                    entries[alloc_idx].checkpoint_id <= dispatch_checkpoint_id[i];
                    entries[alloc_idx].checkpoint_valid <= dispatch_checkpoint_valid[i];
                    entries[alloc_idx].exception <= 1'b0;
                    entries[alloc_idx].result <= 32'b0;
                end
            end
            
            // CDB: mark entries complete
            for (int i = 0; i < CDB_WIDTH; i++) begin
                if (cdb_valid[i]) begin
                    logic [ROB_IDX_BITS-1:0] complete_idx;
                    complete_idx = cdb_rob_idx[i][ROB_IDX_BITS-1:0];
                    
                    if (entries[complete_idx].valid) begin
                        entries[complete_idx].completed <= 1'b1;
                        entries[complete_idx].result <= cdb_data[i];
                    end
                end
            end
            
            // Branch completion
            if (cdb_branch_complete) begin
                logic [ROB_IDX_BITS-1:0] branch_idx;
                branch_idx = cdb_branch_rob_idx[ROB_IDX_BITS-1:0];
                
                if (entries[branch_idx].valid && entries[branch_idx].branch) begin
                    entries[branch_idx].completed <= 1'b1;
                    entries[branch_idx].branch_mispredicted <= cdb_branch_mispredicted;
                    entries[branch_idx].branch_correct_pc <= cdb_branch_correct_pc;
                end
            end
            
            // Load completion
            if (load_complete_valid) begin
                logic [ROB_IDX_BITS-1:0] load_idx;
                load_idx = load_complete_rob_idx[ROB_IDX_BITS-1:0];
                
                if (entries[load_idx].valid && entries[load_idx].mem_read) begin
                    entries[load_idx].completed <= 1'b1;
                end
            end
            
            // Store address ready (stores complete when they have address)
            if (store_addr_valid) begin
                logic [ROB_IDX_BITS-1:0] store_idx;
                store_idx = store_addr_rob_idx[ROB_IDX_BITS-1:0];
                
                if (entries[store_idx].valid && entries[store_idx].mem_write) begin
                    entries[store_idx].completed <= 1'b1;
                end
            end
            
            // Commit: invalidate committed entries
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (can_commit[i]) begin
                    logic [ROB_IDX_BITS-1:0] commit_idx;
                    commit_idx = commit_rob_idx[i][ROB_IDX_BITS-1:0];
                    
                    entries[commit_idx].valid <= 1'b0;
                    entries[commit_idx].completed <= 1'b0;
                end
            end
            
            // Update pointers
            tail_ptr <= next_tail_ptr[DISPATCH_WIDTH];
            head_ptr <= next_head_ptr[COMMIT_WIDTH];
            
            // Update count
            count <= count + dispatch_count - commit_count;
        end
    end

endmodule
