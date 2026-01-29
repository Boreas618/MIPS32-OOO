/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"
`include "BranchTypes.svh"

/**
 * Register Rename Unit - RAT-based Register Renaming
 *
 * Implements register renaming using a Register Alias Table (RAT).
 * Maps architectural registers to physical registers for OoO execution.
 *
 * Components:
 * - RAT: 32 entries mapping arch reg -> phys reg
 * - Free List integration: allocate new physical registers
 * - Checkpointing: snapshot RAT on branches for recovery
 * - Commit RAT: architectural state for precise exceptions
 *
 * Configuration:
 * - Rename width: 4 instructions per cycle
 * - 64 physical registers
 * - 8 checkpoints for branch speculation
 */
module RenameUnit #(
    parameter RENAME_WIDTH = 4,             // Instructions to rename per cycle
    parameter NUM_ARCH_REGS = 32,           // Architectural registers (MIPS)
    parameter NUM_PHYS_REGS = 64,           // Physical registers
    parameter CHECKPOINT_COUNT = 8          // Number of RAT checkpoints
) (
    input   logic                           clk,
    input   logic                           rst,

    // Input from DecodeUnit - architectural register specifiers
    input   logic   [RENAME_WIDTH-1:0]      dec_valid,
    input   logic   [4:0]                   dec_rs [RENAME_WIDTH-1:0],        // Source 1 arch reg
    input   logic   [4:0]                   dec_rt [RENAME_WIDTH-1:0],        // Source 2 arch reg
    input   logic   [4:0]                   dec_dest [RENAME_WIDTH-1:0],      // Destination arch reg
    input   logic   [RENAME_WIDTH-1:0]      dec_reg_write,                    // Has destination
    input   logic   [RENAME_WIDTH-1:0]      dec_branch,                       // Is branch (need checkpoint)
    input   logic   [3:0]                   dec_branch_type [RENAME_WIDTH-1:0],

    // Control signals passthrough
    input   logic   [RENAME_WIDTH-1:0]      dec_mem_to_reg,
    input   logic   [RENAME_WIDTH-1:0]      dec_mem_write,
    input   logic   [RENAME_WIDTH-1:0]      dec_mem_read,
    input   logic   [3:0]                   dec_alu_control [RENAME_WIDTH-1:0],
    input   logic   [1:0]                   dec_alu_src [RENAME_WIDTH-1:0],
    input   logic   [31:0]                  dec_imm [RENAME_WIDTH-1:0],
    input   logic   [4:0]                   dec_shamt [RENAME_WIDTH-1:0],
    input   logic   [31:0]                  dec_pc [RENAME_WIDTH-1:0],
    input   logic   [31:0]                  dec_pc_plus_4 [RENAME_WIDTH-1:0],
    input   logic   [31:0]                  dec_branch_target [RENAME_WIDTH-1:0],
    input   logic   [31:0]                  dec_jump_target [RENAME_WIDTH-1:0],
    input   logic   [31:0]                  dec_predicted_target [RENAME_WIDTH-1:0],
    input   logic   [RENAME_WIDTH-1:0]      dec_predicted_taken,

    // Stall/flush control
    input   logic                           stall,
    input   logic                           flush,
    input   logic   [$clog2(CHECKPOINT_COUNT)-1:0] flush_checkpoint, // Checkpoint to restore on flush

    // Free list interface
    output  logic   [RENAME_WIDTH-1:0]      freelist_alloc_req,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] freelist_alloc_preg [RENAME_WIDTH-1:0],
    input   logic   [RENAME_WIDTH-1:0]      freelist_alloc_valid,
    output  logic                           freelist_checkpoint_save,
    output  logic   [$clog2(CHECKPOINT_COUNT)-1:0] freelist_checkpoint_slot,
    output  logic                           freelist_checkpoint_restore,

    // Commit interface - free old physical registers
    input   logic   [RENAME_WIDTH-1:0]      commit_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] commit_old_preg [RENAME_WIDTH-1:0],
    output  logic   [RENAME_WIDTH-1:0]      freelist_free_req,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] freelist_free_preg [RENAME_WIDTH-1:0],

    // Renamed output
    output  logic   [RENAME_WIDTH-1:0]      ren_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] ren_prs1 [RENAME_WIDTH-1:0],  // Physical source 1
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] ren_prs2 [RENAME_WIDTH-1:0],  // Physical source 2
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] ren_prd [RENAME_WIDTH-1:0],   // Physical destination
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] ren_old_prd [RENAME_WIDTH-1:0], // Old phys dest (for freelist)
    output  logic   [RENAME_WIDTH-1:0]      ren_prs1_ready,                    // Source 1 ready
    output  logic   [RENAME_WIDTH-1:0]      ren_prs2_ready,                    // Source 2 ready

    // Control signals passthrough
    output  logic   [RENAME_WIDTH-1:0]      ren_reg_write,
    output  logic   [RENAME_WIDTH-1:0]      ren_mem_to_reg,
    output  logic   [RENAME_WIDTH-1:0]      ren_mem_write,
    output  logic   [RENAME_WIDTH-1:0]      ren_mem_read,
    output  logic   [RENAME_WIDTH-1:0]      ren_branch,
    output  logic   [3:0]                   ren_alu_control [RENAME_WIDTH-1:0],
    output  logic   [1:0]                   ren_alu_src [RENAME_WIDTH-1:0],
    output  logic   [31:0]                  ren_imm [RENAME_WIDTH-1:0],
    output  logic   [4:0]                   ren_shamt [RENAME_WIDTH-1:0],
    output  logic   [31:0]                  ren_pc [RENAME_WIDTH-1:0],
    output  logic   [31:0]                  ren_pc_plus_4 [RENAME_WIDTH-1:0],
    output  logic   [31:0]                  ren_branch_target [RENAME_WIDTH-1:0],
    output  logic   [31:0]                  ren_jump_target [RENAME_WIDTH-1:0],
    output  logic   [3:0]                   ren_branch_type [RENAME_WIDTH-1:0],
    output  logic   [31:0]                  ren_predicted_target [RENAME_WIDTH-1:0],
    output  logic   [RENAME_WIDTH-1:0]      ren_predicted_taken,

    // Checkpoint allocation
    output  logic   [$clog2(CHECKPOINT_COUNT)-1:0] ren_checkpoint_id [RENAME_WIDTH-1:0],
    output  logic   [RENAME_WIDTH-1:0]      ren_checkpoint_valid,

    // Status
    output  logic                           rename_stall,       // Can't rename (no free regs)
    output  logic   [$clog2(RENAME_WIDTH):0] renamed_count
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);

    // Register Alias Table (RAT) - speculative state
    logic [PREG_BITS-1:0] rat [NUM_ARCH_REGS-1:0];
    logic [NUM_ARCH_REGS-1:0] rat_ready;  // Ready bits per architectural register

    // Committed RAT - architectural state for recovery
    logic [PREG_BITS-1:0] committed_rat [NUM_ARCH_REGS-1:0];

    // RAT checkpoints for branch recovery
    logic [PREG_BITS-1:0] checkpoint_rat [CHECKPOINT_COUNT-1:0][NUM_ARCH_REGS-1:0];
    logic [NUM_ARCH_REGS-1:0] checkpoint_ready [CHECKPOINT_COUNT-1:0];
    logic [$clog2(CHECKPOINT_COUNT)-1:0] next_checkpoint;
    logic [CHECKPOINT_COUNT-1:0] checkpoint_valid;

    // Intermediate RAT for intra-group renaming
    logic [PREG_BITS-1:0] temp_rat [NUM_ARCH_REGS-1:0];
    logic [NUM_ARCH_REGS-1:0] temp_ready;

    // Internal signals
    logic need_checkpoint;
    logic [$clog2(CHECKPOINT_COUNT)-1:0] alloc_checkpoint;
    logic [RENAME_WIDTH-1:0] needs_dest_rename;

    // Determine which instructions need destination renaming
    always_comb begin
        for (int i = 0; i < RENAME_WIDTH; i++) begin
            needs_dest_rename[i] = dec_valid[i] && dec_reg_write[i] && (dec_dest[i] != 5'b0);
        end
    end

    // Request physical registers from free list
    assign freelist_alloc_req = needs_dest_rename & ~{RENAME_WIDTH{stall}};

    // Check if we need a checkpoint for any branch
    always_comb begin
        need_checkpoint = 1'b0;
        for (int i = 0; i < RENAME_WIDTH; i++) begin
            if (dec_valid[i] && dec_branch[i] && 
                dec_branch_type[i] != `BRANCH_TYPE_NONE) begin
                need_checkpoint = 1'b1;
            end
        end
    end

    // Checkpoint signals to free list
    assign freelist_checkpoint_save = need_checkpoint && !stall && !flush;
    assign freelist_checkpoint_slot = next_checkpoint;
    assign freelist_checkpoint_restore = flush;

    // Commit interface - free old physical registers
    always_comb begin
        for (int i = 0; i < RENAME_WIDTH; i++) begin
            freelist_free_req[i] = commit_valid[i];
            freelist_free_preg[i] = commit_old_preg[i];
        end
    end

    // Rename stall if we can't allocate all needed physical registers
    always_comb begin
        rename_stall = 1'b0;
        for (int i = 0; i < RENAME_WIDTH; i++) begin
            if (needs_dest_rename[i] && !freelist_alloc_valid[i]) begin
                rename_stall = 1'b1;
            end
        end
    end

    // Rename logic - combinational lookup with intra-group forwarding
    always_comb begin
        // Initialize temporary RAT from current RAT
        for (int r = 0; r < NUM_ARCH_REGS; r++) begin
            temp_rat[r] = rat[r];
            temp_ready[r] = rat_ready[r];
        end

        // Process each instruction in program order
        for (int i = 0; i < RENAME_WIDTH; i++) begin
            ren_valid[i] = dec_valid[i] && !stall && !flush;

            // Look up source registers in temp_rat (includes updates from earlier instructions)
            if (dec_rs[i] == 5'b0) begin
                ren_prs1[i] = '0;
                ren_prs1_ready[i] = 1'b1;
            end else begin
                ren_prs1[i] = temp_rat[dec_rs[i]];
                ren_prs1_ready[i] = temp_ready[dec_rs[i]];
            end

            if (dec_rt[i] == 5'b0) begin
                ren_prs2[i] = '0;
                ren_prs2_ready[i] = 1'b1;
            end else begin
                ren_prs2[i] = temp_rat[dec_rt[i]];
                ren_prs2_ready[i] = temp_ready[dec_rt[i]];
            end

            // Allocate new physical register for destination
            if (needs_dest_rename[i] && freelist_alloc_valid[i]) begin
                ren_prd[i] = freelist_alloc_preg[i];
                ren_old_prd[i] = temp_rat[dec_dest[i]];

                // Update temp_rat for subsequent instructions in this group
                temp_rat[dec_dest[i]] = freelist_alloc_preg[i];
                temp_ready[dec_dest[i]] = 1'b0;  // Not ready until execution completes
            end else begin
                ren_prd[i] = '0;
                ren_old_prd[i] = '0;
            end

            // Passthrough control signals
            ren_reg_write[i] = dec_reg_write[i];
            ren_mem_to_reg[i] = dec_mem_to_reg[i];
            ren_mem_write[i] = dec_mem_write[i];
            ren_mem_read[i] = dec_mem_read[i];
            ren_branch[i] = dec_branch[i];
            ren_alu_control[i] = dec_alu_control[i];
            ren_alu_src[i] = dec_alu_src[i];
            ren_imm[i] = dec_imm[i];
            ren_shamt[i] = dec_shamt[i];
            ren_pc[i] = dec_pc[i];
            ren_pc_plus_4[i] = dec_pc_plus_4[i];
            ren_branch_target[i] = dec_branch_target[i];
            ren_jump_target[i] = dec_jump_target[i];
            ren_branch_type[i] = dec_branch_type[i];
            ren_predicted_target[i] = dec_predicted_target[i];
            ren_predicted_taken[i] = dec_predicted_taken[i];

            // Checkpoint assignment for branches
            if (dec_valid[i] && dec_branch[i]) begin
                ren_checkpoint_id[i] = next_checkpoint;
                ren_checkpoint_valid[i] = 1'b1;
            end else begin
                ren_checkpoint_id[i] = '0;
                ren_checkpoint_valid[i] = 1'b0;
            end
        end
    end

    // Count renamed instructions
    always_comb begin
        renamed_count = '0;
        for (int i = 0; i < RENAME_WIDTH; i++) begin
            if (ren_valid[i]) begin
                renamed_count = renamed_count + 1;
            end
        end
    end

    // Sequential RAT update and checkpointing
    always_ff @(posedge clk) begin
        if (rst) begin
            // Initialize RAT: arch reg i maps to phys reg i
            for (int r = 0; r < NUM_ARCH_REGS; r++) begin
                rat[r] <= r;
                committed_rat[r] <= r;
                rat_ready[r] <= 1'b1;  // All architectural regs start ready
            end
            rat[0] <= '0;  // $zero always maps to physical 0
            rat_ready[0] <= 1'b1;

            // Clear checkpoints
            for (int c = 0; c < CHECKPOINT_COUNT; c++) begin
                checkpoint_valid[c] <= 1'b0;
                for (int r = 0; r < NUM_ARCH_REGS; r++) begin
                    checkpoint_rat[c][r] <= r;
                    checkpoint_ready[c][r] <= 1'b1;
                end
            end
            next_checkpoint <= '0;
        end else if (flush) begin
            // Restore RAT from checkpoint
            if (checkpoint_valid[flush_checkpoint]) begin
                for (int r = 0; r < NUM_ARCH_REGS; r++) begin
                    rat[r] <= checkpoint_rat[flush_checkpoint][r];
                    rat_ready[r] <= checkpoint_ready[flush_checkpoint][r];
                end
            end else begin
                // Fallback to committed RAT
                for (int r = 0; r < NUM_ARCH_REGS; r++) begin
                    rat[r] <= committed_rat[r];
                    rat_ready[r] <= 1'b1;
                end
            end

            // Invalidate all checkpoints except the one we restored to
            // All checkpoints "newer" than flush_checkpoint are now invalid since we're rolling back
            // Simple approach: invalidate all others, let them be re-allocated as needed
            for (int c = 0; c < CHECKPOINT_COUNT; c++) begin
                if (c != flush_checkpoint[$clog2(CHECKPOINT_COUNT)-1:0]) begin
                    checkpoint_valid[c] <= 1'b0;
                end
            end
            // Reset next_checkpoint to allocate after the restored checkpoint
            next_checkpoint <= (flush_checkpoint + 1) % CHECKPOINT_COUNT;
        end else if (!stall) begin
            // Update RAT with new mappings
            for (int i = 0; i < RENAME_WIDTH; i++) begin
                if (ren_valid[i] && dec_reg_write[i] && dec_dest[i] != 5'b0 && freelist_alloc_valid[i]) begin
                    rat[dec_dest[i]] <= freelist_alloc_preg[i];
                    rat_ready[dec_dest[i]] <= 1'b0;
                end
            end

            // Save checkpoint on branch
            if (need_checkpoint) begin
                for (int r = 0; r < NUM_ARCH_REGS; r++) begin
                    checkpoint_rat[next_checkpoint][r] <= temp_rat[r];
                    checkpoint_ready[next_checkpoint][r] <= temp_ready[r];
                end
                checkpoint_valid[next_checkpoint] <= 1'b1;
                next_checkpoint <= (next_checkpoint + 1) % CHECKPOINT_COUNT;
            end
        end
    end

    // Update committed RAT on instruction commit (handled externally by ROB)
    // Update ready bits when execution completes (handled externally by CDB)

endmodule
