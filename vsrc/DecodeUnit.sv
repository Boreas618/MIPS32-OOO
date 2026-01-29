/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
`include "Icode.svh"
`include "BranchTypes.svh"

/**
 * Parallel Decode Unit for Out-of-Order Frontend
 *
 * Decodes 2-4 instructions per cycle using replicated decode logic.
 * Detects intra-group dependencies and outputs decoded micro-ops.
 *
 * Configuration:
 * - Decode width: 4 instructions per cycle
 * - Replicated decoders for parallel operation
 * - Dependency detection within decode group
 */
module DecodeUnit #(
    parameter DECODE_WIDTH = 4              // Instructions to decode per cycle
) (
    input   logic                           clk,
    input   logic                           rst,

    // Input from InstructionQueue
    input   logic   [31:0]                  insts [DECODE_WIDTH-1:0],
    input   logic   [31:0]                  pcs [DECODE_WIDTH-1:0],
    input   logic   [DECODE_WIDTH-1:0]      valid_in,
    input   logic                           queue_ready,

    // Prediction metadata passthrough
    input   logic   [31:0]                  predicted_target [DECODE_WIDTH-1:0],
    input   logic   [DECODE_WIDTH-1:0]      predicted_taken,

    // Stall/flush control
    input   logic                           stall,
    input   logic                           flush,

    // Decoded output - control signals
    output  logic   [DECODE_WIDTH-1:0]      dec_valid,
    output  logic   [DECODE_WIDTH-1:0]      dec_reg_write,      // Writes to register
    output  logic   [DECODE_WIDTH-1:0]      dec_mem_to_reg,     // Result from memory
    output  logic   [DECODE_WIDTH-1:0]      dec_mem_write,      // Writes to memory
    output  logic   [DECODE_WIDTH-1:0]      dec_mem_read,       // Reads from memory
    output  logic   [DECODE_WIDTH-1:0]      dec_branch,         // Branch instruction
    output  logic   [3:0]                   dec_alu_control [DECODE_WIDTH-1:0],  // ALU operation
    output  logic   [1:0]                   dec_alu_src [DECODE_WIDTH-1:0],      // ALU source select
    output  logic   [DECODE_WIDTH-1:0]      dec_reg_dst,        // Destination is rd vs rt

    // Register specifiers
    output  logic   [4:0]                   dec_rs [DECODE_WIDTH-1:0],           // Source register 1
    output  logic   [4:0]                   dec_rt [DECODE_WIDTH-1:0],           // Source register 2
    output  logic   [4:0]                   dec_rd [DECODE_WIDTH-1:0],           // Destination register
    output  logic   [4:0]                   dec_dest [DECODE_WIDTH-1:0],         // Actual destination (rd or rt)
    output  logic   [31:0]                  dec_imm [DECODE_WIDTH-1:0],          // Sign-extended immediate
    output  logic   [4:0]                   dec_shamt [DECODE_WIDTH-1:0],        // Shift amount

    // PC and branch targets
    output  logic   [31:0]                  dec_pc [DECODE_WIDTH-1:0],
    output  logic   [31:0]                  dec_pc_plus_4 [DECODE_WIDTH-1:0],
    output  logic   [31:0]                  dec_branch_target [DECODE_WIDTH-1:0], // Branch offset target
    output  logic   [31:0]                  dec_jump_target [DECODE_WIDTH-1:0],   // Jump target
    output  logic   [3:0]                   dec_branch_type [DECODE_WIDTH-1:0],

    // Prediction info passthrough
    output  logic   [31:0]                  dec_predicted_target [DECODE_WIDTH-1:0],
    output  logic   [DECODE_WIDTH-1:0]      dec_predicted_taken,

    // Intra-group dependency detection
    output  logic   [DECODE_WIDTH-1:0][DECODE_WIDTH-1:0] dep_matrix,  // dep_matrix[i][j] = inst i depends on inst j

    // Acknowledge back to queue
    output  logic                           decode_ack,

    // Status
    output  logic   [$clog2(DECODE_WIDTH):0] decoded_count
);

    // Internal decoded signals (combinational)
    logic [5:0] opcode [DECODE_WIDTH-1:0];
    logic [5:0] funct [DECODE_WIDTH-1:0];
    logic [31:0] sign_ext_imm [DECODE_WIDTH-1:0];
    logic [31:0] zero_ext_imm [DECODE_WIDTH-1:0];

    // Extract instruction fields
    always_comb begin
        for (int i = 0; i < DECODE_WIDTH; i++) begin
            opcode[i] = insts[i][31:26];
            funct[i] = insts[i][5:0];
            dec_rs[i] = insts[i][25:21];
            dec_rt[i] = insts[i][20:16];
            dec_rd[i] = insts[i][15:11];
            dec_shamt[i] = insts[i][10:6];

            // Sign extension
            sign_ext_imm[i] = {{16{insts[i][15]}}, insts[i][15:0]};
            zero_ext_imm[i] = {16'b0, insts[i][15:0]};

            // PC values
            dec_pc[i] = pcs[i];
            dec_pc_plus_4[i] = pcs[i] + 32'd4;

            // Branch target: PC + 4 + (sign_ext_imm << 2)
            dec_branch_target[i] = pcs[i] + 32'd4 + (sign_ext_imm[i] << 2);

            // Jump target: {PC[31:28], inst[25:0], 2'b00}
            dec_jump_target[i] = {pcs[i][31:28], insts[i][25:0], 2'b00};

            // Prediction passthrough
            dec_predicted_target[i] = predicted_target[i];
            dec_predicted_taken[i] = predicted_taken[i];
        end
    end

    // Parallel decode logic - one decoder per slot
    always_comb begin
        for (int i = 0; i < DECODE_WIDTH; i++) begin
            // Defaults
            dec_valid[i] = valid_in[i] && !flush && queue_ready;
            dec_reg_write[i] = 1'b0;
            dec_mem_to_reg[i] = 1'b0;
            dec_mem_write[i] = 1'b0;
            dec_mem_read[i] = 1'b0;
            dec_branch[i] = 1'b0;
            dec_alu_control[i] = 4'b0000;
            dec_alu_src[i] = 2'b00;
            dec_reg_dst[i] = 1'b0;
            dec_dest[i] = 5'b0;
            dec_imm[i] = sign_ext_imm[i];
            dec_branch_type[i] = `BRANCH_TYPE_NONE;

            if (valid_in[i] && !flush) begin
                if (opcode[i] == `RTYPE) begin
                    case (funct[i])
                        `ADD, `ADDU: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0000;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `SUBU: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0001;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `AND: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0010;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `OR: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0011;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `NOR: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0100;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `XOR: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0101;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `SLL: begin
                            // NOP if rd == 0
                            if (dec_rd[i] != 5'b0) begin
                                dec_reg_write[i] = 1'b1;
                                dec_alu_control[i] = 4'b0110;
                                dec_alu_src[i] = 2'b01;  // Use shamt
                                dec_reg_dst[i] = 1'b1;
                                dec_dest[i] = dec_rd[i];
                            end
                        end
                        `SRA: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0111;
                            dec_alu_src[i] = 2'b01;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `SRL: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b1000;
                            dec_alu_src[i] = 2'b01;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `SLT: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b1001;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `SLTU: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b1010;
                            dec_alu_src[i] = 2'b00;
                            dec_reg_dst[i] = 1'b1;
                            dec_dest[i] = dec_rd[i];
                        end
                        `JR: begin
                            dec_branch[i] = 1'b1;
                            dec_alu_control[i] = 4'b1110;
                            dec_branch_type[i] = `BRANCH_TYPE_JR;
                        end
                        default: begin
                            // Unknown R-type instruction
                        end
                    endcase
                end else begin
                    case (opcode[i])
                        `ADDIU: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0000;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `ANDI: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0010;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = zero_ext_imm[i];
                        end
                        `ORI: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0011;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = zero_ext_imm[i];
                        end
                        `XORI: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0101;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = zero_ext_imm[i];
                        end
                        `SLTI: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b1001;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `SLTIU: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b1010;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `LUI: begin
                            dec_reg_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b1011;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = {insts[i][15:0], 16'b0}; // Upper immediate
                        end
                        `BEQ: begin
                            dec_branch[i] = 1'b1;
                            dec_alu_control[i] = 4'b0001;
                            dec_alu_src[i] = 2'b00;
                            dec_branch_type[i] = `BRANCH_TYPE_BEQ;
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `BNE: begin
                            dec_branch[i] = 1'b1;
                            dec_alu_control[i] = 4'b0001;
                            dec_alu_src[i] = 2'b00;
                            dec_branch_type[i] = `BRANCH_TYPE_BNE;
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `BGEZ: begin
                            dec_branch[i] = 1'b1;
                            dec_alu_control[i] = 4'b1110;
                            dec_alu_src[i] = 2'b00;
                            // Check rt field for BGEZ vs BLTZ
                            if (dec_rt[i] == 5'b00001) begin
                                dec_branch_type[i] = `BRANCH_TYPE_BGEZ;
                            end else begin
                                dec_branch_type[i] = `BRANCH_TYPE_BLTZ;
                            end
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `LW: begin
                            dec_reg_write[i] = 1'b1;
                            dec_mem_to_reg[i] = 1'b1;
                            dec_mem_read[i] = 1'b1;
                            dec_alu_control[i] = 4'b0000;
                            dec_alu_src[i] = 2'b10;
                            dec_reg_dst[i] = 1'b0;
                            dec_dest[i] = dec_rt[i];
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `SW: begin
                            dec_mem_write[i] = 1'b1;
                            dec_alu_control[i] = 4'b0000;
                            dec_alu_src[i] = 2'b10;
                            dec_imm[i] = sign_ext_imm[i];
                        end
                        `J: begin
                            dec_branch[i] = 1'b1;
                            dec_branch_type[i] = `BRANCH_TYPE_J;
                        end
                        `JAL: begin
                            dec_reg_write[i] = 1'b1;
                            dec_branch[i] = 1'b1;
                            dec_alu_control[i] = 4'b1100;  // Pass PC+8
                            dec_branch_type[i] = `BRANCH_TYPE_JAL;
                            dec_dest[i] = 5'd31;  // $ra
                        end
                        default: begin
                            // Unknown instruction - treat as NOP
                        end
                    endcase
                end
            end
        end
    end

    // Intra-group dependency detection
    // dep_matrix[i][j] = 1 means instruction i depends on instruction j (j < i)
    always_comb begin
        for (int i = 0; i < DECODE_WIDTH; i++) begin
            for (int j = 0; j < DECODE_WIDTH; j++) begin
                dep_matrix[i][j] = 1'b0;

                if (i > j && dec_valid[i] && dec_valid[j]) begin
                    // Check if instruction j writes to a register that instruction i reads
                    if (dec_reg_write[j] && dec_dest[j] != 5'b0) begin
                        // RAW dependency on rs
                        if (dec_rs[i] == dec_dest[j]) begin
                            dep_matrix[i][j] = 1'b1;
                        end
                        // RAW dependency on rt (if rt is a source)
                        if (dec_rt[i] == dec_dest[j] && 
                            (dec_alu_src[i] == 2'b00 || dec_mem_write[i])) begin
                            dep_matrix[i][j] = 1'b1;
                        end
                    end
                end
            end
        end
    end

    // Count decoded instructions
    always_comb begin
        decoded_count = '0;
        for (int i = 0; i < DECODE_WIDTH; i++) begin
            if (dec_valid[i]) begin
                decoded_count = decoded_count + 1;
            end
        end
    end

    // Acknowledge to queue - consume instructions when not stalled
    assign decode_ack = queue_ready && !stall && !flush;

endmodule
