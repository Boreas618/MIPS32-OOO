`include "Config.svh"

/*
 * Gshare Branch Direction Predictor
 * 
 * Uses XOR of Global History Register (GHR) with PC bits to index
 * into a Pattern History Table (PHT) of 2-bit saturating counters.
 *
 * 2-bit counter states:
 *   00: Strongly not taken
 *   01: Weakly not taken
 *   10: Weakly taken
 *   11: Strongly taken
 *
 * Prediction: taken if counter[1] == 1
 */
module BranchPredictor #(
    parameter GHR_BITS = 10,           // 10-bit global history
    parameter PHT_SIZE = 1024,         // 2^10 = 1024 entries
    parameter INDEX_BITS = $clog2(PHT_SIZE)
)(
    input   logic           clk,
    input   logic           rst,
    
    // Prediction interface (IF stage)
    /* verilator lint_off UNUSEDSIGNAL */
    input   logic   [31:0]  lookup_pc,
    /* verilator lint_on UNUSEDSIGNAL */
    output  logic           predicted_taken,
    
    // Update interface (EX/MEM stage)
    input   logic           update_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input   logic   [31:0]  update_pc,
    /* verilator lint_on UNUSEDSIGNAL */
    input   logic           update_taken,
    input   logic   [GHR_BITS-1:0] update_ghr,  // GHR at time of prediction
    
    // GHR output for saving with predictions
    output  logic   [GHR_BITS-1:0] current_ghr_out
);

    // Global History Register
    logic [GHR_BITS-1:0] ghr;
    
    // Pattern History Table (2-bit saturating counters)
    logic [1:0] pht [PHT_SIZE-1:0];
    
    // Index calculation: XOR GHR with PC bits
    wire [INDEX_BITS-1:0] lookup_index = ghr ^ lookup_pc[INDEX_BITS+1:2];
    wire [INDEX_BITS-1:0] update_index = update_ghr ^ update_pc[INDEX_BITS+1:2];
    
    // Current GHR output for saving with prediction
    // (Exposed so branch unit can save it for updates)
    
    // Prediction logic (combinational)
    /* verilator lint_off UNUSEDSIGNAL */
    logic [1:0] lookup_counter;
    /* verilator lint_on UNUSEDSIGNAL */
    assign lookup_counter = pht[lookup_index];
    assign predicted_taken = lookup_counter[1];  // Taken if MSB is 1
    
    // Update logic (sequential)
    integer i;
    always_ff @(posedge clk) begin
        if (rst) begin
            ghr <= '0;
            for (i = 0; i < PHT_SIZE; i = i + 1) begin
                pht[i] <= 2'b01;  // Initialize to weakly not taken
            end
        end else if (update_valid) begin
            // Update GHR: shift left, insert actual outcome
            ghr <= {ghr[GHR_BITS-2:0], update_taken};
            
            // Update PHT counter with saturation
            if (update_taken) begin
                // Increment (saturate at 11)
                if (pht[update_index] != 2'b11) begin
                    pht[update_index] <= pht[update_index] + 1;
                end
            end else begin
                // Decrement (saturate at 00)
                if (pht[update_index] != 2'b00) begin
                    pht[update_index] <= pht[update_index] - 1;
                end
            end
        end
    end
    
    // Expose current GHR for saving with predictions
    assign current_ghr_out = ghr;

endmodule
