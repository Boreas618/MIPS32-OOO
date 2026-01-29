`include "Config.svh"

/*
 * Branch Target Buffer (BTB)
 * 
 * 512-entry direct-mapped BTB for predicting branch targets.
 * Each entry contains:
 *   - Valid bit
 *   - Tag (upper PC bits)
 *   - Target address
 *   - Branch type (for distinguishing calls/returns)
 *
 * Branch types:
 *   2'b00: Conditional branch (BEQ, BNE, BGEZ, etc.)
 *   2'b01: Unconditional jump (J)
 *   2'b10: Call (JAL) - push to RAS
 *   2'b11: Return (JR $ra) - pop from RAS
 */
module BTB #(
    parameter NUM_ENTRIES = 512,
    parameter INDEX_BITS = $clog2(NUM_ENTRIES)  // 9 bits for 512 entries
)(
    input   logic           clk,
    input   logic           rst,
    
    // Lookup interface (IF stage)
    /* verilator lint_off UNUSEDSIGNAL */
    input   logic   [31:0]  lookup_pc,
    /* verilator lint_on UNUSEDSIGNAL */
    output  logic           hit,
    output  logic   [31:0]  predicted_target,
    output  logic   [1:0]   branch_type,  // 00=cond, 01=jump, 10=call, 11=return
    
    // Update interface (EX/MEM stage)
    input   logic           update_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input   logic   [31:0]  update_pc,
    /* verilator lint_on UNUSEDSIGNAL */
    input   logic   [31:0]  update_target,
    input   logic   [1:0]   update_type
);

    // Tag width: 32 - INDEX_BITS - 2 (word alignment)
    localparam TAG_BITS = 32 - INDEX_BITS - 2;
    
    // BTB entry structure
    typedef struct packed {
        logic               valid;
        logic [TAG_BITS-1:0] tag;
        logic [31:0]        target;
        logic [1:0]         btype;
    } btb_entry_t;
    
    // BTB storage
    btb_entry_t btb [NUM_ENTRIES-1:0];
    
    // Index and tag extraction
    wire [INDEX_BITS-1:0] lookup_index = lookup_pc[INDEX_BITS+1:2];
    wire [TAG_BITS-1:0]   lookup_tag   = lookup_pc[31:INDEX_BITS+2];
    
    wire [INDEX_BITS-1:0] update_index = update_pc[INDEX_BITS+1:2];
    wire [TAG_BITS-1:0]   update_tag   = update_pc[31:INDEX_BITS+2];
    
    // Lookup logic (combinational)
    btb_entry_t lookup_entry;
    assign lookup_entry = btb[lookup_index];
    
    assign hit = lookup_entry.valid && (lookup_entry.tag == lookup_tag);
    assign predicted_target = lookup_entry.target;
    assign branch_type = lookup_entry.btype;
    
    // Update logic (sequential)
    integer i;
    always_ff @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
                btb[i].valid <= 1'b0;
                btb[i].tag <= '0;
                btb[i].target <= 32'b0;
                btb[i].btype <= 2'b0;
            end
        end else if (update_valid) begin
            btb[update_index].valid <= 1'b1;
            btb[update_index].tag <= update_tag;
            btb[update_index].target <= update_target;
            btb[update_index].btype <= update_type;
        end
    end

endmodule
