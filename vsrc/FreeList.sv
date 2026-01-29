/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off SELRANGE */
`include "Config.svh"

/**
 * Free List - Physical Register Management
 *
 * Manages allocation and deallocation of physical registers for register renaming.
 * Implements a FIFO-based free list to track available physical registers.
 *
 * Configuration:
 * - 64 physical registers (can be expanded to 96)
 * - Supports allocation of up to ALLOC_WIDTH registers per cycle
 * - Supports freeing of up to FREE_WIDTH registers per cycle
 * - Checkpointing for branch misprediction recovery
 */
module FreeList #(
    parameter NUM_PHYS_REGS = 64,           // Total physical registers
    parameter NUM_ARCH_REGS = 32,           // Architectural registers (MIPS)
    parameter ALLOC_WIDTH = 4,              // Max allocations per cycle
    parameter FREE_WIDTH = 4,               // Max frees per cycle
    parameter CHECKPOINT_COUNT = 8          // Number of checkpoint slots
) (
    input   logic                           clk,
    input   logic                           rst,

    // Allocation interface - request physical registers for destinations
    input   logic   [ALLOC_WIDTH-1:0]       alloc_req,          // Request allocation for each slot
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] alloc_preg [ALLOC_WIDTH-1:0],  // Allocated physical registers
    output  logic   [ALLOC_WIDTH-1:0]       alloc_valid,        // Allocation succeeded

    // Free interface - return physical registers on commit
    input   logic   [FREE_WIDTH-1:0]        free_req,           // Request to free registers
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] free_preg [FREE_WIDTH-1:0],   // Physical registers to free

    // Status
    output  logic   [$clog2(NUM_PHYS_REGS):0] free_count,       // Number of free registers
    output  logic                           empty,              // No free registers
    output  logic                           full,               // All registers free (after reset)

    // Checkpoint interface for branch recovery
    input   logic                           checkpoint_save,    // Save current state
    input   logic   [$clog2(CHECKPOINT_COUNT)-1:0] checkpoint_slot, // Slot to save/restore
    input   logic                           checkpoint_restore, // Restore from checkpoint
    output  logic   [$clog2(CHECKPOINT_COUNT)-1:0] next_checkpoint_slot // Next available slot
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);
    localparam PTR_BITS = $clog2(NUM_PHYS_REGS);

    // Free list storage - circular buffer
    logic [PREG_BITS-1:0] free_list [NUM_PHYS_REGS-1:0];
    logic [PTR_BITS-1:0] head;              // Next register to allocate
    logic [PTR_BITS-1:0] tail;              // Next position to add freed register
    logic [PTR_BITS:0] count;               // Number of free registers

    // Checkpoint storage
    logic [PTR_BITS-1:0] checkpoint_head [CHECKPOINT_COUNT-1:0];
    logic [PTR_BITS-1:0] checkpoint_tail [CHECKPOINT_COUNT-1:0];
    logic [PTR_BITS:0] checkpoint_count [CHECKPOINT_COUNT-1:0];
    logic [$clog2(CHECKPOINT_COUNT)-1:0] next_slot;

    // Internal signals for allocation
    logic [PTR_BITS-1:0] alloc_head_next;
    logic [ALLOC_WIDTH:0] alloc_total;

    // Count allocations
    always_comb begin
        alloc_total = '0;
        for (int i = 0; i < ALLOC_WIDTH; i++) begin
            if (alloc_req[i] && (alloc_total < count)) begin
                alloc_valid[i] = 1'b1;
                alloc_preg[i] = free_list[(head + alloc_total[PTR_BITS-1:0]) % NUM_PHYS_REGS];
                alloc_total = alloc_total + 1;
            end else begin
                alloc_valid[i] = 1'b0;
                alloc_preg[i] = '0;
            end
        end
    end

    // Calculate next head after allocations
    always_comb begin
        alloc_head_next = (head + alloc_total[PTR_BITS-1:0]) % NUM_PHYS_REGS;
    end

    // Count frees
    function automatic logic [PTR_BITS:0] count_frees();
        logic [PTR_BITS:0] cnt;
        cnt = '0;
        for (int i = 0; i < FREE_WIDTH; i++) begin
            if (free_req[i]) cnt = cnt + 1;
        end
        return cnt;
    endfunction

    // Status outputs
    assign free_count = count;
    assign empty = (count == 0);
    assign full = (count == NUM_PHYS_REGS);
    assign next_checkpoint_slot = next_slot;

    // Main state machine
    always_ff @(posedge clk) begin
        if (rst) begin
            // Initialize free list with physical registers 32-63
            // (0-31 are initially mapped to architectural registers)
            head <= '0;
            tail <= NUM_PHYS_REGS - NUM_ARCH_REGS;
            count <= NUM_PHYS_REGS - NUM_ARCH_REGS;
            next_slot <= '0;

            for (int i = 0; i < NUM_PHYS_REGS - NUM_ARCH_REGS; i++) begin
                free_list[i] <= NUM_ARCH_REGS + i;
            end
            for (int i = NUM_PHYS_REGS - NUM_ARCH_REGS; i < NUM_PHYS_REGS; i++) begin
                free_list[i] <= '0;
            end

            // Clear checkpoints
            for (int i = 0; i < CHECKPOINT_COUNT; i++) begin
                checkpoint_head[i] <= '0;
                checkpoint_tail[i] <= '0;
                checkpoint_count[i] <= '0;
            end
        end else if (checkpoint_restore) begin
            // Restore from checkpoint
            head <= checkpoint_head[checkpoint_slot];
            tail <= checkpoint_tail[checkpoint_slot];
            count <= checkpoint_count[checkpoint_slot];
        end else begin
            // Save checkpoint if requested
            if (checkpoint_save) begin
                checkpoint_head[checkpoint_slot] <= head;
                checkpoint_tail[checkpoint_slot] <= tail;
                checkpoint_count[checkpoint_slot] <= count;
                next_slot <= (checkpoint_slot + 1) % CHECKPOINT_COUNT;
            end

            // Update head after allocations
            head <= alloc_head_next;

            // Add freed registers to tail - pack contiguously
            // FIX: Use a running offset instead of index i to handle sparse free_req
            begin
                automatic logic [PTR_BITS:0] free_offset = '0;
                for (int i = 0; i < FREE_WIDTH; i++) begin
                    if (free_req[i]) begin
                        free_list[(tail + free_offset[PTR_BITS-1:0]) % NUM_PHYS_REGS] <= free_preg[i];
                        free_offset = free_offset + 1;
                    end
                end
            end
            tail <= (tail + count_frees()) % NUM_PHYS_REGS;

            // Update count: subtract allocations, add frees
            count <= count - alloc_total + count_frees();
        end
    end

endmodule
