/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */

`include "Icode.svh"
`include "BranchTypes.svh"

module Decode(
    input   logic   rst,
    input   logic   clk,
    input   logic   [31:0]inst,
    input   logic   [31:0]pc,
    input   logic   [4:0]write_reg,
    input   logic   [31:0]write_data,
    input   logic   write_enabled,
    input   logic   branch_continue,
    input   logic   dmem_continue,
    output  logic   branch_stall,
    output  logic   dmem_stall,
    output  logic   reg_write_d,
    output  logic   mem_to_reg_d,
    output  logic   mem_write_d,
    output  logic   branch_d,
    output  logic   [3:0]alu_control_d,
    output  logic   [1:0]alu_src_d,
    output  logic   reg_dst_d,
    output  logic   [31:0]rd1_d,
    output  logic   [31:0]rd2_d,
    output  logic   [4:0]rs_d,
    output  logic   [4:0]rt_d,
    output  logic   [4:0]rd_d,
    output  logic   [31:0]imm_d,
    output  logic   [4:0]shamt_d,
    output  logic   [31:0]pc_plus_4d,
    output  logic   [31:0]jump_addr_d,
    output  logic   [3:0]branch_type_d,
    output  logic   mem_access_d,
    output  logic   magic
);
    logic [5:0] op, funct;
    logic [4:0] rs, rt, rd;
    logic [15:0] imm;

    assign op = inst[31:26];
    assign funct = inst[5:0];
    assign rs = inst[25:21];
    assign rt = inst[20:16];
    assign rd = inst[15:11];

    always_ff @(posedge clk) begin
        shamt_d <= inst[10:6];
        rs_d <= inst[25:21];
        rt_d <= inst[20:16];
        rd_d <= inst[15:11];
        imm <= inst[15:0];
        pc_plus_4d <= pc + 4;
        jump_addr_d <= {pc[31:28], inst[25:0], 2'b0};

        /*
         * Branch Configurations.
         *
         * - If the incoming instruction involves branching (jump or conditional branch),
         *   the decoder should determine its type, which will be used in the memory stage.
         *   This is important because different types of instructions lead to different sources
         *   for the next PC (e.g., jr vs j).
         *
         * - If the pipeline is stalled due to a previous branch instruction and the branch
         *   is now resolved, the stall signal should be cleared.
         */
        if (branch_stall && branch_continue) begin
            branch_stall <= 1'b0;
        end else if (op == `BEQ || op == `BNE || op == `BGEZ || op == `J || op == `JAL || (op == `RTYPE && funct == `JR)) begin
            branch_stall <= 1'b1;
        end
        
        if (op == `BEQ || op == `BNE || op == `BGEZ || op == `J || op == `JAL || (op == `RTYPE && funct == `JR)) begin
            if (op == `J) begin
                branch_type_d <= `BRANCH_TYPE_J;
            end else if (op == `JAL) begin
                branch_type_d <= `BRANCH_TYPE_JAL;
            end else if (op == `RTYPE && funct == `JR) begin
                branch_type_d <= `BRANCH_TYPE_JR;
            end else if (op == `BEQ) begin
                branch_type_d <= `BRANCH_TYPE_BEQ;
            end else if (op == `BNE) begin
                branch_type_d <= `BRANCH_TYPE_BNE;
            end else if (op == `BGEZ && inst[20:16] == 5'b1) begin
                branch_type_d <= `BRANCH_TYPE_BGEZ;
            end else if (op == `BGEZ && inst[20:16] != 5'b1) begin
                branch_type_d <= `BRANCH_TYPE_BLTZ;
            end
        end else begin
            branch_type_d <= `BRANCH_TYPE_NONE;
        end

        /*
         * Stall the pipeline on instructions involving memory operations.
         */

        if (dmem_stall && dmem_continue) begin
            dmem_stall <= 1'b0;
        end else if (op == `SW  || op == `LW) begin
            dmem_stall <= 1'b1;
        end

        if (rst) begin
            reg_write_d <= 1'b0;
            mem_to_reg_d <= 1'b0;
            mem_write_d <= 1'b0;
            branch_d <= 1'b0;
            alu_control_d <= 4'b0000;
            alu_src_d <= 2'b00;
            reg_dst_d <= 1'b1;
            mem_access_d <= 1'b0;
        end 
        else if (op == `RTYPE) begin
            case(funct)
                /* 
                 * Currently, our CPU doesn't support exception and therefore add is simply
                 * an alias of addu
                 */
                `ADD: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0000;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `ADDU: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0000;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `SUBU: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0001;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `AND: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0010;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `OR: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0011;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `NOR: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0100;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `XOR: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0101;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end     
                `SLL: begin
                    if (inst[15:11] == 5'b0) begin
                        reg_write_d <= 1'b0;
                        mem_to_reg_d <= 1'b0;
                        mem_write_d <= 1'b0;
                        branch_d <= 1'b0;
                        alu_control_d <= 4'b0000;
                        alu_src_d <= 2'b00;
                        reg_dst_d <= 1'b0;
                        mem_access_d <= 1'b0;
                    end else begin
                        reg_write_d <= 1'b1;
                        mem_to_reg_d <= 1'b0;
                        mem_write_d <= 1'b0;
                        branch_d <= 1'b0;
                        alu_control_d <= 4'b0110;
                        alu_src_d <= 2'b01;
                        reg_dst_d <= 1'b1;
                        mem_access_d <= 1'b0;
                    end
                end
                `SRA: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0111;
                    alu_src_d <= 2'b01;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `SRL: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b1000;
                    alu_src_d <= 2'b01;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `SLT: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b1001;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `SLTU: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b1010;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                `JR: begin
                    reg_write_d <= 1'b0;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b1;
                    alu_control_d <= 4'b1110;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b1;
                    mem_access_d <= 1'b0;
                end
                default: begin
                end
            endcase
        end else begin
            case(op)
                `ADDIU: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0000;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `ANDI: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0010;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `ORI: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0011;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `XORI: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0101;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `SLTI: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b1001;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `SLTIU: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b1010;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `LUI: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b1011;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `BEQ: begin
                    reg_write_d <= 1'b0;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b1;
                    alu_control_d <= 4'b0001;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `BNE: begin
                    reg_write_d <= 1'b0;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b1;
                    alu_control_d <= 4'b0001;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `BGEZ: begin
                    reg_write_d <= 1'b0;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b1;
                    alu_control_d <= 4'b1110;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `SW: begin
                    reg_write_d <= 1'b0;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b1;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0000;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b1;
                end
                `LW: begin
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b1;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b0;
                    alu_control_d <= 4'b0000;
                    alu_src_d <= 2'b10;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b1;
                end
                `J: begin
                    reg_write_d <= 1'b0;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b1;
                    alu_control_d <= 4'b0000;
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;
                    mem_access_d <= 1'b0;
                end
                `JAL: begin
                    // Jump and link: write PC+8 to $ra (reg 31)
                    reg_write_d <= 1'b1;
                    mem_to_reg_d <= 1'b0;
                    mem_write_d <= 1'b0;
                    branch_d <= 1'b1;
                    alu_control_d <= 4'b1100;  // Pass through PC+8
                    alu_src_d <= 2'b00;
                    reg_dst_d <= 1'b0;  // Will need special handling for $31
                    mem_access_d <= 1'b0;
                end
                default: begin
                    $display("[PC: %0x] instruction %0x not implemented. ", pc, inst);
                end
            endcase
        end
    end

    RegFile reg_file(
        .rst(rst),
        .clk(clk),
        .read_addr_1(rs),
        .read_addr_2(rt),
        .write_addr(write_reg),
        .write_data(write_data),
        .write_enabled(write_enabled),
        .data_1(rd1_d),
        .data_2(rd2_d),
        .magic(magic)
    );

    SignExtend extend(
        .in(imm),
        .out(imm_d)
    );

endmodule
