/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"
`include "BranchTypes.svh"

/**
 * Reservation Station - Dynamic Scheduling
 *
 * Implements Tomasulo-style reservation stations for out-of-order execution.
 * Each entry holds a renamed instruction waiting for operands.
 *
 * Operations:
 * - Dispatch: Insert renamed instruction from RenameUnit
 * - Wake-up: Mark sources ready when CDB broadcasts results
 * - Select: Pick oldest ready instruction for execution
 * - Issue: Send to functional unit
 *
 * Configuration:
 * - Parameterized number of entries (8-16 typical)
 * - Supports unified or per-FU design
 */
module ReservationStation #(
    parameter NUM_ENTRIES = 16,              // Number of RS entries
    parameter NUM_PHYS_REGS = 64,            // Physical registers
    parameter DISPATCH_WIDTH = 4,            // Instructions dispatched per cycle
    parameter ISSUE_WIDTH = 2,               // Instructions issued per cycle
    parameter CDB_WIDTH = 4                  // CDB broadcast width
) (
    input   logic                           clk,
    input   logic                           rst,

    // Dispatch interface - from RenameUnit
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] dispatch_prs1 [DISPATCH_WIDTH-1:0],
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] dispatch_prs2 [DISPATCH_WIDTH-1:0],
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] dispatch_prd [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_prs1_value [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_prs2_value [DISPATCH_WIDTH-1:0],
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_prs1_ready,
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_prs2_ready,
    
    // Control signals for dispatched instructions
    input   logic   [3:0]                   dispatch_alu_control [DISPATCH_WIDTH-1:0],
    input   logic   [1:0]                   dispatch_alu_src [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_imm [DISPATCH_WIDTH-1:0],
    input   logic   [4:0]                   dispatch_shamt [DISPATCH_WIDTH-1:0],
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_reg_write,
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_mem_to_reg,
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_mem_write,
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_mem_read,
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_branch,
    input   logic   [31:0]                  dispatch_pc [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_pc_plus_4 [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_branch_target [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_jump_target [DISPATCH_WIDTH-1:0],
    input   logic   [3:0]                   dispatch_branch_type [DISPATCH_WIDTH-1:0],
    input   logic   [31:0]                  dispatch_predicted_target [DISPATCH_WIDTH-1:0],
    input   logic   [DISPATCH_WIDTH-1:0]    dispatch_predicted_taken,
    input   logic   [6:0]                   dispatch_rob_idx [DISPATCH_WIDTH-1:0], // ROB index for ordering
    
    // Wake-up interface - from CDB
    input   logic   [CDB_WIDTH-1:0]         cdb_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] cdb_tag [CDB_WIDTH-1:0],
    input   logic   [31:0]                  cdb_data [CDB_WIDTH-1:0],
    
    // Issue interface - to functional units
    output  logic   [ISSUE_WIDTH-1:0]       issue_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] issue_prd [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_src1 [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_src2 [ISSUE_WIDTH-1:0],
    output  logic   [3:0]                   issue_alu_control [ISSUE_WIDTH-1:0],
    output  logic   [1:0]                   issue_alu_src [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_imm [ISSUE_WIDTH-1:0],
    output  logic   [4:0]                   issue_shamt [ISSUE_WIDTH-1:0],
    output  logic   [ISSUE_WIDTH-1:0]       issue_reg_write,
    output  logic   [ISSUE_WIDTH-1:0]       issue_mem_to_reg,
    output  logic   [ISSUE_WIDTH-1:0]       issue_mem_write,
    output  logic   [ISSUE_WIDTH-1:0]       issue_mem_read,
    output  logic   [ISSUE_WIDTH-1:0]       issue_branch,
    output  logic   [31:0]                  issue_pc [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_pc_plus_4 [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_branch_target [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_jump_target [ISSUE_WIDTH-1:0],
    output  logic   [3:0]                   issue_branch_type [ISSUE_WIDTH-1:0],
    output  logic   [31:0]                  issue_predicted_target [ISSUE_WIDTH-1:0],
    output  logic   [ISSUE_WIDTH-1:0]       issue_predicted_taken,
    output  logic   [6:0]                   issue_rob_idx [ISSUE_WIDTH-1:0],
    output  logic   [$clog2(NUM_ENTRIES)-1:0] issue_rs_idx [ISSUE_WIDTH-1:0],
    
    // Issue acknowledgment from functional units
    input   logic   [ISSUE_WIDTH-1:0]       issue_ack,
    
    // Flush interface
    input   logic                           flush,
    
    // Status outputs
    output  logic   [$clog2(NUM_ENTRIES):0] free_count,
    output  logic                           full,
    output  logic                           empty
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);
    localparam ENTRY_BITS = $clog2(NUM_ENTRIES);

    // Reservation Station Entry structure
    typedef struct packed {
        logic                   valid;          // Entry is valid
        logic                   issued;         // Has been issued (waiting for completion)
        logic [PREG_BITS-1:0]   prs1;           // Physical source 1 tag
        logic [PREG_BITS-1:0]   prs2;           // Physical source 2 tag
        logic [PREG_BITS-1:0]   prd;            // Physical destination tag
        logic [31:0]            src1_value;     // Source 1 value (if ready)
        logic [31:0]            src2_value;     // Source 2 value (if ready)
        logic                   src1_ready;     // Source 1 is ready
        logic                   src2_ready;     // Source 2 is ready
        logic [3:0]             alu_control;    // ALU operation
        logic [1:0]             alu_src;        // ALU source select
        logic [31:0]            imm;            // Immediate value
        logic [4:0]             shamt;          // Shift amount
        logic                   reg_write;      // Write to register
        logic                   mem_to_reg;     // Memory to register
        logic                   mem_write;      // Memory write
        logic                   mem_read;       // Memory read
        logic                   branch;         // Is branch
        logic [31:0]            pc;             // Instruction PC
        logic [31:0]            pc_plus_4;      // PC + 4
        logic [31:0]            branch_target;  // Branch target
        logic [31:0]            jump_target;    // Jump target
        logic [3:0]             branch_type;    // Branch type
        logic [31:0]            predicted_target; // Predicted target
        logic                   predicted_taken;  // Predicted taken
        logic [6:0]             rob_idx;        // ROB index for age ordering
    } rs_entry_t;

    // Reservation station storage
    rs_entry_t entries [NUM_ENTRIES-1:0];

    // Ready signals for selection
    logic [NUM_ENTRIES-1:0] entry_ready;
    logic [NUM_ENTRIES-1:0] entry_valid;
    
    // Free entry tracking
    logic [NUM_ENTRIES-1:0] entry_free;
    logic [ENTRY_BITS-1:0] free_idx [DISPATCH_WIDTH-1:0];
    logic [DISPATCH_WIDTH-1:0] alloc_valid;

    // Selection logic outputs
    logic [ENTRY_BITS-1:0] selected_idx [ISSUE_WIDTH-1:0];
    logic [ISSUE_WIDTH-1:0] select_valid;

    // Determine which entries are ready and valid
    always_comb begin
        for (int i = 0; i < NUM_ENTRIES; i++) begin
            entry_valid[i] = entries[i].valid && !entries[i].issued;
            entry_ready[i] = entry_valid[i] && entries[i].src1_ready && entries[i].src2_ready;
            entry_free[i] = !entries[i].valid;
        end
    end

    // Find free entries for dispatch (priority encoder)
    always_comb begin
        logic [NUM_ENTRIES-1:0] remaining_free;
        remaining_free = entry_free;
        
        for (int d = 0; d < DISPATCH_WIDTH; d++) begin
            alloc_valid[d] = 1'b0;
            free_idx[d] = '0;
            
            for (int i = 0; i < NUM_ENTRIES; i++) begin
                if (remaining_free[i] && !alloc_valid[d]) begin
                    free_idx[d] = i[ENTRY_BITS-1:0];
                    alloc_valid[d] = 1'b1;
                    remaining_free[i] = 1'b0;
                end
            end
        end
    end

    // Select oldest ready instructions for issue
    // Uses ROB index as age - lower index is older
    
    // Intermediate signals for selection
    logic [NUM_ENTRIES-1:0] remaining_ready_0, remaining_ready_1;
    logic [6:0] min_rob_idx_0, min_rob_idx_1;
    logic found_0, found_1;
    
    always_comb begin
        // Initialize
        remaining_ready_0 = entry_ready;
        remaining_ready_1 = entry_ready;
        
        // First issue slot selection
        select_valid[0] = 1'b0;
        selected_idx[0] = '0;
        min_rob_idx_0 = 7'h7F;
        found_0 = 1'b0;
        
        for (int i = 0; i < NUM_ENTRIES; i++) begin
            if (remaining_ready_0[i]) begin
                if (!found_0 || entries[i].rob_idx < min_rob_idx_0) begin
                    selected_idx[0] = i[ENTRY_BITS-1:0];
                    min_rob_idx_0 = entries[i].rob_idx;
                    found_0 = 1'b1;
                end
            end
        end
        
        if (found_0) begin
            select_valid[0] = 1'b1;
            remaining_ready_1[selected_idx[0]] = 1'b0;
        end
        
        // Second issue slot selection (if ISSUE_WIDTH > 1)
        if (ISSUE_WIDTH > 1) begin
            select_valid[1] = 1'b0;
            selected_idx[1] = '0;
            min_rob_idx_1 = 7'h7F;
            found_1 = 1'b0;
            
            for (int i = 0; i < NUM_ENTRIES; i++) begin
                if (remaining_ready_1[i]) begin
                    if (!found_1 || entries[i].rob_idx < min_rob_idx_1) begin
                        selected_idx[1] = i[ENTRY_BITS-1:0];
                        min_rob_idx_1 = entries[i].rob_idx;
                        found_1 = 1'b1;
                    end
                end
            end
            
            if (found_1) begin
                select_valid[1] = 1'b1;
            end
        end
    end

    // Issue outputs
    always_comb begin
        for (int s = 0; s < ISSUE_WIDTH; s++) begin
            issue_valid[s] = select_valid[s];
            issue_rs_idx[s] = selected_idx[s];
            
            if (select_valid[s]) begin
                issue_prd[s] = entries[selected_idx[s]].prd;
                issue_src1[s] = entries[selected_idx[s]].src1_value;
                issue_src2[s] = entries[selected_idx[s]].src2_value;
                issue_alu_control[s] = entries[selected_idx[s]].alu_control;
                issue_alu_src[s] = entries[selected_idx[s]].alu_src;
                issue_imm[s] = entries[selected_idx[s]].imm;
                issue_shamt[s] = entries[selected_idx[s]].shamt;
                issue_reg_write[s] = entries[selected_idx[s]].reg_write;
                issue_mem_to_reg[s] = entries[selected_idx[s]].mem_to_reg;
                issue_mem_write[s] = entries[selected_idx[s]].mem_write;
                issue_mem_read[s] = entries[selected_idx[s]].mem_read;
                issue_branch[s] = entries[selected_idx[s]].branch;
                issue_pc[s] = entries[selected_idx[s]].pc;
                issue_pc_plus_4[s] = entries[selected_idx[s]].pc_plus_4;
                issue_branch_target[s] = entries[selected_idx[s]].branch_target;
                issue_jump_target[s] = entries[selected_idx[s]].jump_target;
                issue_branch_type[s] = entries[selected_idx[s]].branch_type;
                issue_predicted_target[s] = entries[selected_idx[s]].predicted_target;
                issue_predicted_taken[s] = entries[selected_idx[s]].predicted_taken;
                issue_rob_idx[s] = entries[selected_idx[s]].rob_idx;
            end else begin
                issue_prd[s] = '0;
                issue_src1[s] = '0;
                issue_src2[s] = '0;
                issue_alu_control[s] = '0;
                issue_alu_src[s] = '0;
                issue_imm[s] = '0;
                issue_shamt[s] = '0;
                issue_reg_write[s] = '0;
                issue_mem_to_reg[s] = '0;
                issue_mem_write[s] = '0;
                issue_mem_read[s] = '0;
                issue_branch[s] = '0;
                issue_pc[s] = '0;
                issue_pc_plus_4[s] = '0;
                issue_branch_target[s] = '0;
                issue_jump_target[s] = '0;
                issue_branch_type[s] = '0;
                issue_predicted_target[s] = '0;
                issue_predicted_taken[s] = '0;
                issue_rob_idx[s] = '0;
            end
        end
    end

    // Count free entries
    always_comb begin
        free_count = '0;
        for (int i = 0; i < NUM_ENTRIES; i++) begin
            if (entry_free[i]) begin
                free_count = free_count + 1;
            end
        end
    end

    assign full = (free_count == 0);
    assign empty = (free_count == NUM_ENTRIES);

    // Sequential logic: dispatch, wake-up, issue completion
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            for (int i = 0; i < NUM_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
                entries[i].issued <= 1'b0;
                entries[i].src1_ready <= 1'b0;
                entries[i].src2_ready <= 1'b0;
            end
        end else begin
            // Wake-up logic: check CDB for matching tags
            for (int i = 0; i < NUM_ENTRIES; i++) begin
                if (entries[i].valid && !entries[i].issued) begin
                    // Check each CDB entry for source 1
                    for (int c = 0; c < CDB_WIDTH; c++) begin
                        if (cdb_valid[c] && !entries[i].src1_ready && 
                            entries[i].prs1 == cdb_tag[c]) begin
                            entries[i].src1_value <= cdb_data[c];
                            entries[i].src1_ready <= 1'b1;
                        end
                    end
                    
                    // Check each CDB entry for source 2
                    for (int c = 0; c < CDB_WIDTH; c++) begin
                        if (cdb_valid[c] && !entries[i].src2_ready && 
                            entries[i].prs2 == cdb_tag[c]) begin
                            entries[i].src2_value <= cdb_data[c];
                            entries[i].src2_ready <= 1'b1;
                        end
                    end
                end
            end

            // Issue completion: mark issued entries
            for (int s = 0; s < ISSUE_WIDTH; s++) begin
                if (issue_valid[s] && issue_ack[s]) begin
                    entries[selected_idx[s]].issued <= 1'b1;
                end
            end
            
            // Free completed entries (when CDB broadcasts their result)
            for (int i = 0; i < NUM_ENTRIES; i++) begin
                if (entries[i].valid && entries[i].issued) begin
                    for (int c = 0; c < CDB_WIDTH; c++) begin
                        if (cdb_valid[c] && entries[i].prd == cdb_tag[c]) begin
                            entries[i].valid <= 1'b0;
                            entries[i].issued <= 1'b0;
                        end
                    end
                end
            end

            // Dispatch: insert new instructions
            for (int d = 0; d < DISPATCH_WIDTH; d++) begin
                if (dispatch_valid[d] && alloc_valid[d]) begin
                    entries[free_idx[d]].valid <= 1'b1;
                    entries[free_idx[d]].issued <= 1'b0;
                    entries[free_idx[d]].prs1 <= dispatch_prs1[d];
                    entries[free_idx[d]].prs2 <= dispatch_prs2[d];
                    entries[free_idx[d]].prd <= dispatch_prd[d];
                    entries[free_idx[d]].src1_value <= dispatch_prs1_value[d];
                    entries[free_idx[d]].src2_value <= dispatch_prs2_value[d];
                    entries[free_idx[d]].src1_ready <= dispatch_prs1_ready[d];
                    entries[free_idx[d]].src2_ready <= dispatch_prs2_ready[d];
                    entries[free_idx[d]].alu_control <= dispatch_alu_control[d];
                    entries[free_idx[d]].alu_src <= dispatch_alu_src[d];
                    entries[free_idx[d]].imm <= dispatch_imm[d];
                    entries[free_idx[d]].shamt <= dispatch_shamt[d];
                    entries[free_idx[d]].reg_write <= dispatch_reg_write[d];
                    entries[free_idx[d]].mem_to_reg <= dispatch_mem_to_reg[d];
                    entries[free_idx[d]].mem_write <= dispatch_mem_write[d];
                    entries[free_idx[d]].mem_read <= dispatch_mem_read[d];
                    entries[free_idx[d]].branch <= dispatch_branch[d];
                    entries[free_idx[d]].pc <= dispatch_pc[d];
                    entries[free_idx[d]].pc_plus_4 <= dispatch_pc_plus_4[d];
                    entries[free_idx[d]].branch_target <= dispatch_branch_target[d];
                    entries[free_idx[d]].jump_target <= dispatch_jump_target[d];
                    entries[free_idx[d]].branch_type <= dispatch_branch_type[d];
                    entries[free_idx[d]].predicted_target <= dispatch_predicted_target[d];
                    entries[free_idx[d]].predicted_taken <= dispatch_predicted_taken[d];
                    entries[free_idx[d]].rob_idx <= dispatch_rob_idx[d];
                    
                    // Check CDB for same-cycle wake-up
                    for (int c = 0; c < CDB_WIDTH; c++) begin
                        if (cdb_valid[c]) begin
                            if (!dispatch_prs1_ready[d] && dispatch_prs1[d] == cdb_tag[c]) begin
                                entries[free_idx[d]].src1_value <= cdb_data[c];
                                entries[free_idx[d]].src1_ready <= 1'b1;
                            end
                            if (!dispatch_prs2_ready[d] && dispatch_prs2[d] == cdb_tag[c]) begin
                                entries[free_idx[d]].src2_value <= cdb_data[c];
                                entries[free_idx[d]].src2_ready <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule
