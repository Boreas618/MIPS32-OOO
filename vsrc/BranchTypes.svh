`ifndef __BRANCH_TYPES_SVH__
`define __BRANCH_TYPES_SVH__

/*
 * Centralized branch type encoding definitions
 * Used by Decode.sv, BranchUnit.sv, and Memory.sv
 *
 * 4-bit encoding for branch_type_d/branch_type_e signal:
 *   0000: No branch
 *   0001: J   - Unconditional jump
 *   0010: JAL - Jump and link (call)
 *   0011: JR  - Jump register (return if $ra)
 *   0100: BEQ - Branch if equal
 *   0101: BNE - Branch if not equal
 *   0110: BGEZ - Branch if greater than or equal to zero
 *   0111: BLTZ - Branch if less than zero
 */

// Branch type encodings (decode/pipeline)
`define BRANCH_TYPE_NONE  4'b0000
`define BRANCH_TYPE_J     4'b0001
`define BRANCH_TYPE_JAL   4'b0010
`define BRANCH_TYPE_JR    4'b0011
`define BRANCH_TYPE_BEQ   4'b0100
`define BRANCH_TYPE_BNE   4'b0101
`define BRANCH_TYPE_BGEZ  4'b0110
`define BRANCH_TYPE_BLTZ  4'b0111

// BTB type encodings (internal to branch prediction)
`define BTB_TYPE_COND     2'b00   // Conditional branch (BEQ, BNE, BGEZ, BLTZ)
`define BTB_TYPE_JUMP     2'b01   // Unconditional jump (J)
`define BTB_TYPE_CALL     2'b10   // Call (JAL)
`define BTB_TYPE_RETURN   2'b11   // Return (JR $ra)

`endif
