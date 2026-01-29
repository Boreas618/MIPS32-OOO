/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"

/**
 * Instruction Queue - Fetch Buffer
 *
 * FIFO buffer that decouples fetch rate from decode rate.
 * Handles fetch bubbles (cache miss, branch redirect) by buffering instructions.
 *
 * Configuration:
 * - 32 entries (configurable)
 * - Accepts up to ENQUEUE_WIDTH instructions per cycle from fetch
 * - Provides up to DEQUEUE_WIDTH instructions per cycle to decode
 */
module InstructionQueue #(
    parameter QUEUE_DEPTH = 32,             // Number of entries
    parameter ENQUEUE_WIDTH = 4,            // Max instructions from fetch
    parameter DEQUEUE_WIDTH = 4             // Max instructions to decode
) (
    input   logic                           clk,
    input   logic                           rst,

    // Enqueue interface (from FetchUnit)
    input   logic   [31:0]                  enq_insts [ENQUEUE_WIDTH-1:0],
    input   logic   [31:0]                  enq_pcs [ENQUEUE_WIDTH-1:0],
    input   logic   [ENQUEUE_WIDTH-1:0]     enq_valid,
    input   logic   [$clog2(ENQUEUE_WIDTH):0] enq_count,
    input   logic                           enq_ready,          // Fetch has valid data

    // Prediction metadata (stored with instructions)
    input   logic   [31:0]                  enq_predicted_target,
    input   logic                           enq_predicted_taken,

    // Dequeue interface (to DecodeUnit)
    output  logic   [31:0]                  deq_insts [DEQUEUE_WIDTH-1:0],
    output  logic   [31:0]                  deq_pcs [DEQUEUE_WIDTH-1:0],
    output  logic   [DEQUEUE_WIDTH-1:0]     deq_valid,
    output  logic   [$clog2(DEQUEUE_WIDTH):0] deq_count,
    output  logic                           deq_ready,          // Queue has data
    input   logic                           deq_ack,            // Decode consumed data

    // Prediction metadata output
    output  logic   [31:0]                  deq_predicted_target [DEQUEUE_WIDTH-1:0],
    output  logic   [DEQUEUE_WIDTH-1:0]     deq_predicted_taken,

    // Flush control
    input   logic                           flush,

    // Status outputs
    output  logic   [$clog2(QUEUE_DEPTH):0] queue_count,
    output  logic                           queue_empty,
    output  logic                           queue_full,
    output  logic   [$clog2(QUEUE_DEPTH):0] free_slots,

    // Stall signal to fetch
    output  logic                           fetch_stall         // Queue almost full
);

    localparam PTR_BITS = $clog2(QUEUE_DEPTH);

    // Queue entry structure
    typedef struct packed {
        logic [31:0] inst;
        logic [31:0] pc;
        logic [31:0] predicted_target;
        logic        predicted_taken;
        logic        valid;
    } queue_entry_t;

    // Queue storage
    queue_entry_t queue [QUEUE_DEPTH-1:0];

    // Queue pointers
    logic [PTR_BITS-1:0] head;              // Read pointer (oldest instruction)
    logic [PTR_BITS-1:0] tail;              // Write pointer (next free slot)
    logic [PTR_BITS:0] count;               // Number of valid entries

    // Watermarks for flow control
    localparam ALMOST_FULL_THRESHOLD = QUEUE_DEPTH - ENQUEUE_WIDTH - 2;

    // Status signals
    assign queue_count = count;
    assign queue_empty = (count == 0);
    assign queue_full = (count >= QUEUE_DEPTH);
    assign free_slots = QUEUE_DEPTH - count;
    assign fetch_stall = (count >= ALMOST_FULL_THRESHOLD);
    assign deq_ready = (count >= 1);

    // Calculate how many we can actually enqueue
    wire [$clog2(ENQUEUE_WIDTH):0] actual_enqueue = 
        (enq_ready && !flush && (free_slots >= enq_count)) ? enq_count : '0;

    // Calculate how many we can dequeue
    wire [$clog2(DEQUEUE_WIDTH):0] available_dequeue = 
        (count > DEQUEUE_WIDTH) ? DEQUEUE_WIDTH : count[$clog2(DEQUEUE_WIDTH):0];

    // Dequeue logic - combinational read
    always_comb begin
        deq_count = '0;

        for (int i = 0; i < DEQUEUE_WIDTH; i++) begin
            if (i < available_dequeue && queue[(head + i) % QUEUE_DEPTH].valid) begin
                deq_insts[i] = queue[(head + i) % QUEUE_DEPTH].inst;
                deq_pcs[i] = queue[(head + i) % QUEUE_DEPTH].pc;
                deq_predicted_target[i] = queue[(head + i) % QUEUE_DEPTH].predicted_target;
                deq_predicted_taken[i] = queue[(head + i) % QUEUE_DEPTH].predicted_taken;
                deq_valid[i] = 1'b1;
                deq_count = deq_count + 1;
            end else begin
                deq_insts[i] = 32'b0;
                deq_pcs[i] = 32'b0;
                deq_predicted_target[i] = 32'b0;
                deq_predicted_taken[i] = 1'b0;
                deq_valid[i] = 1'b0;
            end
        end
    end

    // Sequential enqueue and state management
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            head <= '0;
            tail <= '0;
            count <= '0;

            // Invalidate all entries
            for (int i = 0; i < QUEUE_DEPTH; i++) begin
                queue[i].valid <= 1'b0;
                queue[i].inst <= 32'b0;
                queue[i].pc <= 32'b0;
                queue[i].predicted_target <= 32'b0;
                queue[i].predicted_taken <= 1'b0;
            end
        end else begin
            // Enqueue new instructions
            if (enq_ready && !flush) begin
                for (int i = 0; i < ENQUEUE_WIDTH; i++) begin
                    if (enq_valid[i] && ((tail + i) % QUEUE_DEPTH != head || count == 0 || i < free_slots)) begin
                        queue[(tail + i) % QUEUE_DEPTH].inst <= enq_insts[i];
                        queue[(tail + i) % QUEUE_DEPTH].pc <= enq_pcs[i];
                        queue[(tail + i) % QUEUE_DEPTH].predicted_target <= enq_predicted_target;
                        queue[(tail + i) % QUEUE_DEPTH].predicted_taken <= enq_predicted_taken;
                        queue[(tail + i) % QUEUE_DEPTH].valid <= 1'b1;
                    end
                end
                tail <= (tail + actual_enqueue) % QUEUE_DEPTH;
            end

            // Dequeue on acknowledge
            if (deq_ack && deq_ready) begin
                for (int i = 0; i < DEQUEUE_WIDTH; i++) begin
                    if (deq_valid[i]) begin
                        queue[(head + i) % QUEUE_DEPTH].valid <= 1'b0;
                    end
                end
                head <= (head + deq_count) % QUEUE_DEPTH;
            end

            // Update count
            count <= count + actual_enqueue - (deq_ack ? deq_count : '0);
        end
    end

endmodule
