module Execute(
    input   logic   clk,
    input   logic   rst,
    input   logic   [4:0] shamt_d,
    input   logic   [31:0] rd1_d,
    input   logic   [31:0] rd2_d,
    input   logic   [4:0] rt_d,
    input   logic   [4:0] rd_d,
    input   logic   [31:0] imm_d,
    input   logic   reg_write_d,
    input   logic   mem_to_reg_d,
    input   logic   mem_write_d,
    input   logic   branch_d,
    input   logic   [3:0] alu_control_d,
    input   logic   [1:0] alu_src_d,
    input   logic   reg_dst_d,
    input   logic   [31:0] pc_plus_4d,
    input   logic   mem_access_d,
    input   logic   forward_src_a_enabled,
    input   logic   [31:0] forward_src_a,
    input   logic   forward_src_b_enabled,
    input   logic   [31:0] forward_src_b,
    input   logic   [31:0] jump_addr_d,
    input   logic   [3:0] branch_type_d,
    output  logic   [31:0] alu_out_e,
    output  logic   [31:0] write_data_e,
    output  logic   [4:0] write_reg_e,
    output  logic   reg_write_e,
    output  logic   mem_to_reg_e,
    output  logic   mem_write_e,
    output  logic   branch_e,
    output  logic   zero_e,
    output  logic   [31:0] pc_branch_e,
    output  logic   [31:0] jump_addr_e,
    output  logic   [3:0] branch_type_e,
    output  logic   mem_access_e
);
    logic [3:0]alu_control_e;
    logic [1:0]alu_src_e;
    logic reg_dst_e;
    logic [31:0]rd1_e;
    logic [31:0]rd2_e;
    logic [4:0]rt_e;
    logic [4:0]rd_e;
    logic [31:0]imm_e;
    logic [31:0]src1_e;
    logic [31:0]src2_e;
    logic [4:0]shamt_e;
    logic [31:0]pc_plus_4e;  // For JAL return address
    logic [31:0]alu_out_raw; // Raw ALU output before JAL override

    assign src1_e = alu_src_e[0] ? {27'b0, shamt_e} : rd1_e;
    assign src2_e = alu_src_e[1] ? imm_e : rd2_e;
    assign write_data_e = rd2_e;
    // For JAL (branch_type 0010), write to $ra (register 31)
    assign write_reg_e = (branch_type_e == 4'b0010) ? 5'd31 : (reg_dst_e ? rd_e : rt_e);

    always_ff @(posedge clk) begin
        if (rst) begin
            reg_write_e <= 1'b0;
            mem_to_reg_e <= 1'b0;
            mem_write_e <= 1'b0;
            branch_e <= 1'b0;
            alu_control_e <= 4'b0;
            alu_src_e <= 2'b0;
            reg_dst_e <= 1'b0;
            rd1_e <= 32'b0;
            rd2_e <= 32'b0;
            rt_e <= 5'b0;
            rd_e <= 5'b0;
            imm_e <= 32'b0;
            shamt_e <= 5'b0;
            pc_branch_e <= 32'b0;
            pc_plus_4e <= 32'b0;
            jump_addr_e <= 32'b0;
            branch_type_e <= 4'b0;
            mem_access_e <= 1'b0;
        end else begin
            reg_write_e <= reg_write_d;
            mem_to_reg_e <= mem_to_reg_d;
            mem_write_e <= mem_write_d;
            branch_e <= branch_d;
            alu_control_e <= alu_control_d;
            alu_src_e <= alu_src_d;
            reg_dst_e <= reg_dst_d;
            pc_branch_e <= pc_plus_4d + (imm_d << 2);
            pc_plus_4e <= pc_plus_4d;  // Save for JAL return address
            jump_addr_e <= jump_addr_d;
            branch_type_e <= branch_type_d;
            mem_access_e <= mem_access_d;

            if (forward_src_a_enabled) begin
                rd1_e <= forward_src_a;
            end else begin
                rd1_e <= rd1_d;
            end

            if (forward_src_b_enabled) begin
                rd2_e <= forward_src_b;
            end else begin
                rd2_e <= rd2_d;
            end

            rt_e <= rt_d;
            rd_e <= rd_d;
            imm_e <= imm_d;
            shamt_e <= shamt_d;
        end
    end

    ALU alu(
        .alu_ctrl(alu_control_e),
        .src1(src1_e),
        .src2(src2_e),
        .out(alu_out_raw)
    );
    
    // For JAL (branch_type 0010), output return address (PC+8)
    // Otherwise use raw ALU output
    assign alu_out_e = (branch_type_e == 4'b0010) ? (pc_plus_4e + 32'd4) : alu_out_raw;

    assign zero_e = (alu_out_raw == 32'b0) ? 1 : 0;

endmodule
