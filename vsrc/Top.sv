`include "Config.svh"

module Top (
    input	logic	rst,
    input	logic	clk,
    output	logic	halt,
    output	logic	err,
    output	logic	[31:0] pc,
    output	logic	[31:0] system_counter,
    output	logic	[31:0] last_pc,
    output	logic	[31:0] last_inst
);

    /*
     * This block is for testing purposes. Test cases will return to `EXIT_ADDR`
     * with the return value MAGIC (see Icode.svh) if they pass. Otherwise, they
     * will return with the value -1, and the err signal will be asserted to
     * indicate an issue to the test framework.
     */
    logic magic;

    always_comb begin
        if (pc == `EXIT_ADDR) begin
            halt = 1'b1;
            err = ~magic;
        end else begin
            halt = 1'b0;
            err = 1'b0;
        end
    end

    /*
     * These signals relate to the stall and continuation of the pipeline. The
     * pipeline stalls on branches (as we have yet to support branch prediction)
     * and memory accesses (we simulate memory access latencies in InstMemory and
     * DataMemory). The continuation of memory-related stalls is based on the
     * status code, indicating the completion of memory accesses.
     */
    logic branch_stall;
    logic imem_stall;
    logic dmem_stall;
    logic branch_continue;
    logic dmem_continue;
    logic stall;

    assign imem_stall = ~(imem_status == 2'b10);
    assign branch_continue = branch_e;
    assign dmem_continue = dmem_status == 2'b10;
    assign stall = branch_stall || imem_stall || dmem_stall;


    /* Hazard resolution unit. */
    logic forward_src_a_enabled;
    logic [31:0] forward_src_a;
    logic forward_src_b_enabled;
    logic [31:0] forward_src_b;

    Hazard hazard(
        .clk(clk),
        .rst(rst),
        .reg_write_e(reg_write_e),
        .reg_write_m(reg_write_m),
        .reg_write_w(reg_write_w),
        .rs_d(rs_d),
        .rt_d(rt_d),
        .write_reg_e(write_reg_e),
        .write_reg_m(write_reg_m),
        .write_reg_w(write_reg_w),
        .alu_out_e(alu_out_e),
        .alu_out_m(alu_out_m),
        .result_w(result_w),
        .forward_src_a_enabled(forward_src_a_enabled),
        .forward_src_a(forward_src_a),
        .forward_src_b_enabled(forward_src_b_enabled),
        .forward_src_b(forward_src_b)
    );
    
    /* 
     * Instruction Fetch Stage.
     * 
     * `inst`: the current instruction with MIPS encoding specification.
     * `if_pc_src`: select the next PC.
     * `if_pc_branch_in`: the next PC provided by branch instructions.
     * `imem_status`: the status code of the instruction memory.
     */
    logic [31:0] inst;
    logic [1:0] if_pc_src;
    logic [31:0] if_pc_branch_in;
    logic [1:0] imem_status;

    always_ff @(posedge clk) begin
        if (rst) begin
            pc <= `TEXT_BASE;
        end else begin
            last_pc <= pc;
            case (if_pc_src)
                2'h0: begin
                    pc <= stall ? pc : pc + 32'd4;
                end
                2'h1: begin
                    pc <= if_pc_branch_in;
                end
                default: begin
                    pc <= `TEXT_BASE;
                end
            endcase  
        end
    end

    /* Cache statistics (optional debug signals) */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [31:0] icache_hit_count;
    logic [31:0] icache_miss_count;
    /* verilator lint_on UNUSEDSIGNAL */

    L1ICache inst_cache (
        .clk(clk),
        .rst(rst),
        .stall(dmem_stall || branch_stall),
        .addr(pc),
        .r_data(inst),
        .r_data_status(imem_status),
        .hit_count(icache_hit_count),
        .miss_count(icache_miss_count)
    );

    /*
     * Instruction Decoding Stage.
     *
     * The `mem_access_d` signal indicates that the instruction being decoded
     * involves memory operations (e.g., sw and lw). If this signal is asserted,
     * the pipeline will stall.
     */

    logic reg_write_d;
    logic mem_to_reg_d;
    logic mem_write_d;
    logic branch_d;
    logic [3:0]alu_control_d;
    logic [1:0]alu_src_d;
    logic reg_dst_d;
    logic [31:0]rd1_d;
    logic [31:0]rd2_d;
    logic [4:0]rs_d;
    logic [4:0]rt_d;
    logic [4:0]rd_d;
    logic [31:0]imm_d;
    logic [4:0]shamt_d;
    logic [31:0]pc_plus_4d;
    logic [31:0]jump_addr_d;
    logic [3:0] branch_type_d;
    logic mem_access_d;

    Decode decode(
        .inst(inst),
        .rst(rst),
        .clk(clk),
        .pc(pc),
        .reg_write_d(reg_write_d),
        .mem_to_reg_d(mem_to_reg_d),
        .mem_write_d(mem_write_d),
        .branch_d(branch_d),
        .alu_control_d(alu_control_d),
        .alu_src_d(alu_src_d),
        .reg_dst_d(reg_dst_d),
        .rd1_d(rd1_d),
        .rd2_d(rd2_d),
        .rs_d(rs_d),
        .rt_d(rt_d),
        .rd_d(rd_d),
        .imm_d(imm_d),
        .shamt_d(shamt_d),
        .write_reg(write_reg_w),
        .write_data(result_w),
        .write_enabled(reg_write_w),
        .branch_stall(branch_stall),
        .dmem_stall(dmem_stall),
        .pc_plus_4d(pc_plus_4d),
        .branch_continue(branch_continue),
        .dmem_continue(dmem_continue),
        .jump_addr_d(jump_addr_d),
        .branch_type_d(branch_type_d),
        .magic(magic),
        .mem_access_d(mem_access_d)
    );

    /* EX stage. */
    logic [31:0] alu_out_e;
    logic [31:0] write_data_e;
    logic [4:0] write_reg_e;
    logic reg_write_e;
    logic mem_to_reg_e;
    logic mem_write_e;
    logic branch_e;
    logic zero_e;
    logic [31:0] pc_branch_e;
    logic [31:0]jump_addr_e;
    logic [3:0] branch_type_e;
    logic mem_access_e;

    Execute execute (
        .clk(clk),
        .rst(rst),
        .shamt_d(shamt_d),
        .rd1_d(rd1_d),
        .rd2_d(rd2_d),
        .rt_d(rt_d),
        .rd_d(rd_d),
        .imm_d(imm_d),
        .reg_write_d(reg_write_d),
        .mem_to_reg_d(mem_to_reg_d),
        .mem_write_d(mem_write_d),
        .branch_d(branch_d),
        .alu_control_d(alu_control_d),
        .alu_src_d(alu_src_d),
        .reg_dst_d(reg_dst_d),
        .jump_addr_d(jump_addr_d),
        .branch_type_d(branch_type_d),
        .mem_access_d(mem_access_d),
        .forward_src_a_enabled(forward_src_a_enabled),
        .forward_src_a(forward_src_a),
        .forward_src_b_enabled(forward_src_b_enabled),
        .forward_src_b(forward_src_b),
        .alu_out_e(alu_out_e),
        .write_data_e(write_data_e),
        .write_reg_e(write_reg_e),
        .reg_write_e(reg_write_e),
        .mem_to_reg_e(mem_to_reg_e),
        .mem_write_e(mem_write_e),
        .branch_e(branch_e),
        .zero_e(zero_e),
        .pc_plus_4d(pc_plus_4d),
        .pc_branch_e(pc_branch_e),
        .jump_addr_e(jump_addr_e),
        .branch_type_e(branch_type_e),
        .mem_access_e(mem_access_e)
    );

    /* MEM stage. */
    logic [31:0] read_data_m;
    logic [31:0] alu_out_m;
    logic reg_write_m;
    logic mem_to_reg_m;
    logic [4:0] write_reg_m;
    logic [1:0] dmem_status;

    Memory memory(
        .clk(clk),
        .rst(rst),
        .mem_access_e(mem_access_e),
        .reg_write_e(reg_write_e),
        .mem_to_reg_e(mem_to_reg_e),
        .mem_write_e(mem_write_e),
        .branch_e(branch_e),
        .alu_out_e(alu_out_e),
        .write_data_e(write_data_e),
        .write_reg_e(write_reg_e),
        .zero_e(zero_e),
        .read_data_m(read_data_m),
        .alu_out_m(alu_out_m),
        .reg_write_m(reg_write_m),
        .mem_to_reg_m(mem_to_reg_m),
        .write_reg_m(write_reg_m),
        .if_pc_branch_in(if_pc_branch_in),
        .if_pc_src(if_pc_src),
        .pc_branch_e(pc_branch_e),
        .jump_addr_e(jump_addr_e),
        .branch_type_e(branch_type_e),
        .dmem_status(dmem_status)
    );

    logic [31:0] result_w;
    logic [4:0] write_reg_w;
    logic reg_write_w;

    WriteBack write_back(
        .rst(rst),
        .clk(clk),
        .reg_write_m(reg_write_m),
        .mem_to_reg_m(mem_to_reg_m),
        .alu_out_m(alu_out_m),
        .read_data_m(read_data_m),
        .write_reg_m(write_reg_m),
        .write_reg_w(write_reg_w),
        .result_w(result_w),
        .reg_write_w(reg_write_w)
    );

    /* Legacy Debug signals. */
    assign last_inst = inst;
    always @(posedge clk) begin
        if (rst) begin
            system_counter <= 32'b0;
        end
        else begin
            system_counter <= system_counter + 32'd1;
        end
    end

endmodule
