`include "Config.svh"

/*
 * Return Address Stack (RAS)
 * 
 * 16-entry circular stack for predicting function return addresses.
 * - Push on JAL (function call): save PC+8 (return address)
 * - Pop on JR $ra (function return): predict return target
 *
 * Handles overflow/underflow gracefully:
 * - Overflow: wrap around (oldest entry overwritten)
 * - Underflow: return invalid prediction
 */
module RAS #(
    parameter DEPTH = 16,
    parameter PTR_BITS = $clog2(DEPTH)
)(
    input   logic           clk,
    input   logic           rst,
    
    // Prediction interface (IF stage)
    input   logic           pop,              // Pop request (for return)
    output  logic   [31:0]  predicted_target,
    output  logic           valid,            // Valid prediction available
    
    // Update interface
    input   logic           push,             // Push request (for call)
    input   logic   [31:0]  push_addr,        // Return address to push
    
    // Recovery interface (for misprediction)
    // Checkpoint format: {sp[PTR_BITS-1:0], count[PTR_BITS:0]} = 2*PTR_BITS + 1 bits
    input   logic           recover,                        // Restore to checkpoint
    input   logic   [2*PTR_BITS:0] recover_ptr,            // Pointer to restore
    output  logic   [2*PTR_BITS:0] checkpoint              // Current state for saving
);

    // Stack storage
    logic [31:0] stack [DEPTH-1:0];
    
    // Stack pointer (points to next empty slot)
    logic [PTR_BITS-1:0] sp;
    
    // Count of valid entries (0 to DEPTH)
    logic [PTR_BITS:0] count;
    
    // Checkpoint: {sp[PTR_BITS-1:0], count[PTR_BITS:0]}
    assign checkpoint = {sp, count};
    
    // Top of stack (what would be popped)
    wire [PTR_BITS-1:0] top_index = sp - 1;
    assign predicted_target = stack[top_index];
    assign valid = (count > 0);
    
    // Stack operations
    integer i;
    always_ff @(posedge clk) begin
        if (rst) begin
            sp <= '0;
            count <= '0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                stack[i] <= 32'b0;
            end
        end else if (recover) begin
            // Restore from checkpoint: {sp[PTR_BITS-1:0], count[PTR_BITS:0]}
            sp <= recover_ptr[2*PTR_BITS:PTR_BITS+1];     // Upper PTR_BITS bits
            count <= recover_ptr[PTR_BITS:0];              // Lower PTR_BITS+1 bits
        end else begin
            // Handle simultaneous push/pop (call in delay slot of return - rare)
            if (push && pop) begin
                // Push takes precedence, then pop
                // Net effect: replace top of stack
                stack[top_index] <= push_addr;
            end else if (push) begin
                // Push: store at sp, increment sp
                stack[sp] <= push_addr;
                sp <= sp + 1;
                if (count < DEPTH) begin
                    count <= count + 1;
                end
            end else if (pop && count > 0) begin
                // Pop: decrement sp (entry still there but logically removed)
                sp <= sp - 1;
                count <= count - 1;
            end
        end
    end

endmodule
