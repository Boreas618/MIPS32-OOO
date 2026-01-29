/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
`include "Config.svh"
`include "BranchTypes.svh"

/**
 * Wide Fetch Unit for Out-of-Order Frontend
 *
 * Fetches 2-4 instructions per cycle from the I-cache.
 * Handles fetch across cache line boundaries and integrates with branch prediction.
 *
 * Configuration:
 * - Fetch width: 4 instructions (configurable)
 * - Cache line: 64 bytes (16 instructions)
 * - Handles split fetches across cache lines
 */
module FetchUnit #(
    parameter FETCH_WIDTH = 4,              // Instructions per cycle
    parameter CACHE_LINE_SIZE = 64,         // Cache line size in bytes
    parameter WORDS_PER_LINE = 16           // 32-bit words per cache line
) (
    input   logic                           clk,
    input   logic                           rst,

    // PC interface
    input   logic   [31:0]                  pc,
    output  logic   [31:0]                  next_pc,
    output  logic                           pc_valid,

    // Stall/flush control
    input   logic                           stall,
    input   logic                           flush,
    input   logic   [31:0]                  redirect_pc,    // New PC on flush/misprediction

    // I-cache interface
    output  logic   [31:0]                  icache_addr,
    input   logic   [31:0]                  icache_data [WORDS_PER_LINE-1:0], // Entire cache line
    input   logic                           icache_hit,
    input   logic                           icache_valid,

    // Branch prediction interface
    input   logic   [31:0]                  predicted_pc,
    input   logic                           prediction_valid,
    input   logic                           predicted_taken,

    // Fetch packet output
    output  logic   [31:0]                  fetch_insts [FETCH_WIDTH-1:0],    // Fetched instructions
    output  logic   [31:0]                  fetch_pcs [FETCH_WIDTH-1:0],      // PC of each instruction
    output  logic   [FETCH_WIDTH-1:0]       fetch_valid,                       // Which slots are valid
    output  logic   [$clog2(FETCH_WIDTH):0] fetch_count,                      // Number of valid instructions
    output  logic                           fetch_ready,                       // Fetch packet ready

    // Prediction info for pipeline tracking
    output  logic   [31:0]                  fetch_predicted_target,
    output  logic                           fetch_predicted_taken,

    // Performance counters
    output  logic   [31:0]                  fetch_stall_cycles,
    output  logic   [31:0]                  fetch_redirect_count
);

    // State machine for fetch
    typedef enum logic [2:0] {
        FETCH_IDLE,
        FETCH_REQUEST,
        FETCH_WAIT,
        FETCH_COMPLETE,
        FETCH_REDIRECT
    } fetch_state_t;

    fetch_state_t state, next_state;

    // Internal registers
    logic [31:0] current_pc;

    // Calculate how many instructions we can fetch from current position
    wire [$clog2(WORDS_PER_LINE)-1:0] word_offset = current_pc[5:2];  // Offset within cache line
    wire [$clog2(WORDS_PER_LINE):0] words_remaining = WORDS_PER_LINE - word_offset;
    wire [$clog2(FETCH_WIDTH):0] fetch_available = (words_remaining > FETCH_WIDTH) ? FETCH_WIDTH : words_remaining[$clog2(FETCH_WIDTH):0];

    // Cache line address (aligned)
    assign icache_addr = {current_pc[31:6], 6'b0};

    // PC and fetch outputs
    assign pc_valid = (state == FETCH_COMPLETE);

    // Calculate instructions to fetch
    always_comb begin
        // Default outputs
        for (int i = 0; i < FETCH_WIDTH; i++) begin
            fetch_insts[i] = 32'b0;
            fetch_pcs[i] = 32'b0;
            fetch_valid[i] = 1'b0;
        end
        fetch_count = '0;
        fetch_ready = 1'b0;

        if (state == FETCH_COMPLETE && icache_valid && icache_hit) begin
            fetch_ready = 1'b1;

            // Extract instructions from cache line
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (i < fetch_available) begin
                    fetch_insts[i] = icache_data[word_offset + i];
                    fetch_pcs[i] = current_pc + (i << 2);
                    fetch_valid[i] = 1'b1;
                    fetch_count = fetch_count + 1;

                    // Check for branch prediction - stop fetching after predicted taken
                    if (prediction_valid && predicted_taken &&
                        (fetch_pcs[i] == pc)) begin
                        // This is the predicted branch - mark remaining slots invalid
                        for (int j = i + 1; j < FETCH_WIDTH; j++) begin
                            fetch_valid[j] = 1'b0;
                        end
                        break;
                    end
                end
            end

        end
    end

    // Next PC calculation
    always_comb begin
        if (flush) begin
            next_pc = redirect_pc;
        end else if (prediction_valid && predicted_taken) begin
            next_pc = predicted_pc;
        end else begin
            next_pc = current_pc + (fetch_count << 2);
        end
    end

    // Prediction info forwarding
    assign fetch_predicted_target = prediction_valid ? predicted_pc : (current_pc + 32'd4);
    assign fetch_predicted_taken = prediction_valid && predicted_taken;

    // State machine
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= FETCH_IDLE;
            current_pc <= `TEXT_BASE;
            fetch_stall_cycles <= 32'b0;
            fetch_redirect_count <= 32'b0;
        end else begin
            state <= next_state;

            case (state)
                FETCH_IDLE: begin
                    current_pc <= pc;
                end

                FETCH_REQUEST: begin
                    // Request sent to I-cache
                end

                FETCH_WAIT: begin
                    fetch_stall_cycles <= fetch_stall_cycles + 1;
                end

                FETCH_COMPLETE: begin
                    if (!stall) begin
                        if (flush) begin
                            current_pc <= redirect_pc;
                            fetch_redirect_count <= fetch_redirect_count + 1;
                        end else if (prediction_valid && predicted_taken) begin
                            current_pc <= predicted_pc;
                        end else begin
                            current_pc <= current_pc + (fetch_count << 2);
                        end
                    end
                end

                FETCH_REDIRECT: begin
                    current_pc <= redirect_pc;
                    fetch_redirect_count <= fetch_redirect_count + 1;
                end

                default: begin
                    current_pc <= `TEXT_BASE;
                end
            endcase
        end
    end

    // Next state logic
    always_comb begin
        next_state = state;

        case (state)
            FETCH_IDLE: begin
                if (!stall) begin
                    next_state = FETCH_REQUEST;
                end
            end

            FETCH_REQUEST: begin
                next_state = FETCH_WAIT;
            end

            FETCH_WAIT: begin
                if (flush) begin
                    next_state = FETCH_REDIRECT;
                end else if (icache_valid) begin
                    if (icache_hit) begin
                        next_state = FETCH_COMPLETE;
                    end else begin
                        // Cache miss - keep waiting
                        next_state = FETCH_WAIT;
                    end
                end
            end

            FETCH_COMPLETE: begin
                if (flush) begin
                    next_state = FETCH_REDIRECT;
                end else if (stall) begin
                    next_state = FETCH_COMPLETE;
                end else begin
                    next_state = FETCH_REQUEST;
                end
            end

            FETCH_REDIRECT: begin
                next_state = FETCH_REQUEST;
            end

            default: begin
                next_state = FETCH_IDLE;
            end
        endcase
    end

endmodule
