/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDPARAM */
`include "Config.svh"

/**
 * L1 Instruction Cache
 *
 * Configuration:
 * - Direct-mapped
 * - 16KB total size
 * - 64B cache lines (16 words per line)
 * - 256 sets
 * - Address: [31:14] tag (18 bits), [13:6] index (8 bits), [5:0] offset (6 bits)
 *
 * Hit latency: 1 cycle
 * Miss penalty: 20 cycles (fetch from main memory, no L2/L3)
 */
module L1ICache (
    input   logic           clk,
    input   logic           rst,
    input   logic           stall,

    input   logic   [31:0]  addr,

    output  logic   [31:0]  r_data,
    output  logic   [1:0]   r_data_status,

    output  logic   [31:0]  hit_count,
    output  logic   [31:0]  miss_count
);

    /* Cache parameters */
    localparam NUM_SETS         = 256;
    localparam WORDS_PER_LINE   = 16;
    localparam INDEX_BITS       = 8;
    localparam TAG_BITS         = 18;

    /* Cache storage */
    logic [TAG_BITS-1:0]    tag_array   [NUM_SETS-1:0];
    logic                   valid_array [NUM_SETS-1:0];
    logic [31:0]            data_array  [NUM_SETS-1:0][WORDS_PER_LINE-1:0];

    import "DPI-C" function void mm_read(
        input longint addr,
        output longint data
    );

    /* Address decomposition */
    wire [TAG_BITS-1:0]     cur_tag   = addr[31:14];
    wire [INDEX_BITS-1:0]   cur_index = addr[13:6];
    wire [3:0]              cur_word  = addr[5:2];

    /* Cache lookup */
    wire cache_hit = valid_array[cur_index] && (tag_array[cur_index] == cur_tag);

    /* Latency simulation - configured in Config.svh */
    parameter HIT_LATENCY = `L1_HIT_LATENCY;
    parameter MISS_LATENCY = `L1_MISS_LATENCY;
    logic [7:0] cycle_counter;
    logic [7:0] target_latency;
    logic is_hit_reg;

    always_ff @(posedge clk) begin
        if (rst || stall) begin
            r_data_status <= 2'b0;
            cycle_counter <= 8'b0;
            r_data <= 32'b0;
            target_latency <= 8'b0;
            is_hit_reg <= 1'b0;
        end else begin
            if (r_data_status == 2'b0) begin
                r_data_status <= 2'b1;
                cycle_counter <= 8'b0;
                // Determine latency based on hit/miss at start of access
                is_hit_reg <= cache_hit;
                target_latency <= cache_hit ? HIT_LATENCY : MISS_LATENCY;
            end else if (r_data_status == 2'b1) begin
                if (cycle_counter == target_latency) begin
                    if (is_hit_reg) begin
                        // Cache hit - read from cached line
                        r_data <= data_array[cur_index][cur_word];
                        hit_count <= hit_count + 1;
                    end else begin
                        // Cache miss - fetch entire line from main memory
                        logic [63:0] mem_data;
                        logic [31:0] line_base;
                        line_base = {addr[31:6], 6'b0};

                        // Fetch all 16 words (DPI-C calls are instant in simulation)
                        for (int w = 0; w < WORDS_PER_LINE; w++) begin
                            mm_read({32'b0, line_base + (w << 2)}, mem_data);
                            data_array[cur_index][w] <= mem_data[31:0];
                        end

                        // Read the requested word directly
                        mm_read({32'b0, addr}, mem_data);
                        r_data <= mem_data[31:0];

                        // Update tag and valid
                        tag_array[cur_index] <= cur_tag;
                        valid_array[cur_index] <= 1'b1;

                        miss_count <= miss_count + 1;
                    end
                    r_data_status <= 2'b10;
                    cycle_counter <= 8'b0;
                end else begin
                    cycle_counter <= cycle_counter + 1;
                end
            end else begin
                r_data_status <= 2'b0;
                cycle_counter <= 8'b0;
                r_data <= 32'b0;
            end
        end
    end

    // Initialize cache on reset
    initial begin
        for (int i = 0; i < NUM_SETS; i++) begin
            valid_array[i] = 1'b0;
            tag_array[i] = '0;
            for (int w = 0; w < WORDS_PER_LINE; w++) begin
                data_array[i][w] = 32'b0;
            end
        end
        hit_count = 32'b0;
        miss_count = 32'b0;
    end

endmodule
