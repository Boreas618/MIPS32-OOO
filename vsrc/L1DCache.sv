/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDPARAM */
`include "Config.svh"

/**
 * L1 Data Cache
 * 
 * Configuration:
 * - 4-way set-associative
 * - 16KB total size
 * - 64B cache lines (16 words per line)
 * - 64 sets
 * - Address: [31:12] tag (20 bits), [11:6] index (6 bits), [5:0] offset (6 bits)
 * - Write-back, write-allocate policy
 * - LRU replacement
 */
module L1DCache (
    input   logic           clk,
    input   logic           rst,

    input   logic           valid,
    input   logic   [31:0]  addr,
    input   logic           write_enabled,
    input   logic   [31:0]  w_data,

    output  logic   [31:0]  r_data,
    output  logic   [1:0]   status,

    output  logic   [31:0]  hit_count,
    output  logic   [31:0]  miss_count
);

    /* Cache parameters */
    localparam NUM_WAYS         = 4;
    localparam NUM_SETS         = 64;
    localparam WORDS_PER_LINE   = 16;
    localparam INDEX_BITS       = 6;
    localparam TAG_BITS         = 20;

    /* Cache storage */
    logic [TAG_BITS-1:0]    tag_array   [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic                   valid_array [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic                   dirty_array [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [31:0]            data_array  [NUM_SETS-1:0][NUM_WAYS-1:0][WORDS_PER_LINE-1:0];
    logic [1:0]             lru_counter [NUM_SETS-1:0][NUM_WAYS-1:0];

    import "DPI-C" function void mm_read(
        input longint addr,
        output longint data
    );

    import "DPI-C" function void mm_write(
        input longint addr,
        input longint data
    );

    /* Saved request */
    logic [31:0] saved_addr;
    logic saved_write_en;
    logic [31:0] saved_w_data;

    /* Address decomposition */
    wire [TAG_BITS-1:0]     cur_tag   = saved_addr[31:12];
    wire [INDEX_BITS-1:0]   cur_index = saved_addr[11:6];
    wire [3:0]              cur_word  = saved_addr[5:2];

    /* Cache lookup - check all ways */
    logic cache_hit;
    logic [1:0] hit_way;
    
    always_comb begin
        cache_hit = 1'b0;
        hit_way = 2'b0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (valid_array[cur_index][w] && (tag_array[cur_index][w] == cur_tag)) begin
                cache_hit = 1'b1;
                hit_way = w[1:0];
            end
        end
    end

    /* LRU victim selection */
    logic [1:0] victim_way;
    always_comb begin
        victim_way = 2'b00;
        // Prefer invalid ways first
        if (!valid_array[cur_index][0]) victim_way = 2'b00;
        else if (!valid_array[cur_index][1]) victim_way = 2'b01;
        else if (!valid_array[cur_index][2]) victim_way = 2'b10;
        else if (!valid_array[cur_index][3]) victim_way = 2'b11;
        else begin
            // All valid - find LRU
            if (lru_counter[cur_index][0] <= lru_counter[cur_index][1] &&
                lru_counter[cur_index][0] <= lru_counter[cur_index][2] &&
                lru_counter[cur_index][0] <= lru_counter[cur_index][3])
                victim_way = 2'b00;
            else if (lru_counter[cur_index][1] <= lru_counter[cur_index][2] &&
                     lru_counter[cur_index][1] <= lru_counter[cur_index][3])
                victim_way = 2'b01;
            else if (lru_counter[cur_index][2] <= lru_counter[cur_index][3])
                victim_way = 2'b10;
            else
                victim_way = 2'b11;
        end
    end

    /* Latency simulation */
    parameter LATENCY = 10;
    logic [7:0] cycle_counter;

    always_ff @(posedge clk) begin
        if (rst) begin
            status <= 2'b0;
            cycle_counter <= 8'b0;
            r_data <= 32'b0;
            saved_addr <= 32'b0;
            saved_write_en <= 1'b0;
            saved_w_data <= 32'b0;
        end else begin
            if (status == 2'b0 && valid) begin
                status <= 2'b1;
                cycle_counter <= 8'b1;
                saved_addr <= addr;
                saved_write_en <= write_enabled;
                saved_w_data <= w_data;
            end else if (status == 2'b1) begin
                if (cycle_counter == LATENCY) begin
                    logic [1:0] way_to_use;
                    logic [63:0] mem_data;
                    logic [31:0] line_base;
                    logic [63:0] wb_data;
                    
                    if (cache_hit) begin
                        way_to_use = hit_way;
                        hit_count <= hit_count + 1;
                    end else begin
                        way_to_use = victim_way;
                        miss_count <= miss_count + 1;
                        
                        // Write back dirty line if needed
                        if (valid_array[cur_index][victim_way] && dirty_array[cur_index][victim_way]) begin
                            line_base = {tag_array[cur_index][victim_way], cur_index, 6'b0};
                            for (int w = 0; w < WORDS_PER_LINE; w++) begin
                                wb_data = {32'b0, data_array[cur_index][victim_way][w]};
                                mm_write({32'b0, line_base + (w << 2)}, wb_data);
                            end
                        end
                        
                        // Fetch new line
                        line_base = {saved_addr[31:6], 6'b0};
                        for (int w = 0; w < WORDS_PER_LINE; w++) begin
                            mm_read({32'b0, line_base + (w << 2)}, mem_data);
                            data_array[cur_index][victim_way][w] <= mem_data[31:0];
                        end
                        
                        // Update metadata
                        tag_array[cur_index][victim_way] <= cur_tag;
                        valid_array[cur_index][victim_way] <= 1'b1;
                        dirty_array[cur_index][victim_way] <= 1'b0;
                    end
                    
                    // Handle read/write
                    if (saved_write_en) begin
                        // Write operation
                        data_array[cur_index][way_to_use][cur_word] <= saved_w_data;
                        dirty_array[cur_index][way_to_use] <= 1'b1;
                    end else begin
                        // Read operation
                        if (cache_hit) begin
                            r_data <= data_array[cur_index][hit_way][cur_word];
                        end else begin
                            // Data was just fetched, read from memory again
                            mm_read({32'b0, saved_addr}, mem_data);
                            r_data <= mem_data[31:0];
                        end
                    end
                    
                    // Update LRU
                    lru_counter[cur_index][way_to_use] <= 2'b11;
                    for (int w = 0; w < NUM_WAYS; w++) begin
                        if (w[1:0] != way_to_use && lru_counter[cur_index][w] > 2'b00) begin
                            lru_counter[cur_index][w] <= lru_counter[cur_index][w] - 1;
                        end
                    end
                    
                    status <= 2'b10;
                    cycle_counter <= 8'b0;
                end else begin
                    cycle_counter <= cycle_counter + 1;
                end
            end else begin
                status <= 2'b0;
                cycle_counter <= 8'b0;
            end
        end
    end

    // Initialize cache on reset
    initial begin
        for (int s = 0; s < NUM_SETS; s++) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                valid_array[s][w] = 1'b0;
                dirty_array[s][w] = 1'b0;
                tag_array[s][w] = '0;
                lru_counter[s][w] = w[1:0];
                for (int word = 0; word < WORDS_PER_LINE; word++) begin
                    data_array[s][w][word] = 32'b0;
                end
            end
        end
        hit_count = 32'b0;
        miss_count = 32'b0;
    end

endmodule
