`include "Config.svh"

module Top (
    input	logic	rst,
    input	logic	clk,
    output	logic	halt,
    output	logic	err,
    output	logic	[31:0] pc,
    output	logic	[31:0] system_counter,
    output	logic	[31:0] last_pc,
    output	logic	[31:0] last_inst,
    // Branch prediction statistics
    output  logic   [31:0] bp_total_branches_out,
    output  logic   [31:0] bp_mispredictions_out
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
     * Pipeline control signals with branch prediction.
     * 
     * With branch prediction:
     * - When BTB hits on a branch in decode, use predicted target instead of stalling
     * - When BTB misses, fall back to stalling behavior
     * - On misprediction, flush speculative instructions
     */
    logic branch_stall;
    logic imem_stall;
    logic dmem_stall;
    logic branch_continue;
    logic dmem_continue;
    logic stall;
    logic use_prediction;  // BTB hit, use predicted target
    logic misprediction_flush;  // Flush due to misprediction

    assign imem_stall = ~(imem_status == 2'b10);
    assign branch_continue = branch_e || use_prediction;  // Prediction counts as "continue"
    assign dmem_continue = dmem_status == 2'b10;
    assign stall = (branch_stall && !use_prediction) || imem_stall || dmem_stall;

    /* 
     * Branch Prediction Unit
     * Provides predictions to reduce branch penalty.
     * When BTB hits, pipeline continues with predicted target instead of stalling.
     */
    localparam GHR_BITS = 10;
    
    logic [31:0] predicted_pc;
    logic prediction_valid;
    /* verilator lint_off UNUSEDSIGNAL */
    logic mispredicted;  // Used internally for recovery control
    /* verilator lint_on UNUSEDSIGNAL */
    logic [31:0] bp_total_branches;
    logic [31:0] bp_mispredictions;
    logic [31:0] correct_pc;
    logic flush_pipeline;
    
    // Export branch prediction statistics
    assign bp_total_branches_out = bp_total_branches;
    assign bp_mispredictions_out = bp_mispredictions;
    
    // Signals for branch resolution
    logic branch_resolve_valid;
    logic [31:0] branch_resolve_pc;
    logic [31:0] branch_resolve_target;
    logic branch_resolve_taken;
    logic [3:0] branch_resolve_type;
    
    // Track prediction info through pipeline
    logic [31:0] predicted_target_d, predicted_target_e;
    logic predicted_taken_d, predicted_taken_e;
    logic [GHR_BITS-1:0] saved_ghr_d, saved_ghr_e;
    logic [8:0] saved_ras_checkpoint_d, saved_ras_checkpoint_e;
    
    // GHR and RAS checkpoint from branch unit
    logic [GHR_BITS-1:0] current_ghr;
    logic [8:0] ras_checkpoint_out;
    
    // RAS recovery signals
    logic ras_recover;
    logic [8:0] ras_recover_checkpoint;

    BranchUnit #(
        .GHR_BITS(GHR_BITS)
    ) branch_unit (
        .clk(clk),
        .rst(rst),
        .if_pc(pc),
        .predicted_pc(predicted_pc),
        .prediction_valid(prediction_valid),
        .branch_valid(branch_resolve_valid),
        .branch_pc(branch_resolve_pc),
        .actual_target(branch_resolve_target),
        .actual_taken(branch_resolve_taken),
        .branch_type(branch_resolve_type),
        .predicted_target_ex(predicted_target_e),
        .predicted_taken_ex(predicted_taken_e),
        .saved_ghr(saved_ghr_e),
        .mispredicted(mispredicted),
        .correct_pc(correct_pc),
        .flush_pipeline(flush_pipeline),
        .current_ghr(current_ghr),
        .ras_checkpoint_out(ras_checkpoint_out),
        .ras_recover(ras_recover),
        .ras_recover_checkpoint(ras_recover_checkpoint),
        .total_branches(bp_total_branches),
        .mispredictions(bp_mispredictions)
    );
    
    // RAS recovery on misprediction
    assign ras_recover = misprediction_flush;
    assign ras_recover_checkpoint = saved_ras_checkpoint_e;
    
    // Use prediction when BTB hits and we're looking at a branch
    assign use_prediction = prediction_valid && branch_stall;
    assign misprediction_flush = flush_pipeline;

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
     * PC update priority:
     * 1. Misprediction flush: redirect to correct target
     * 2. Branch resolution (if_pc_src=1): actual branch target
     * 3. Prediction (use_prediction): predicted target
     * 4. Normal: PC + 4 (or stall)
     */
    logic [31:0] inst;
    logic [1:0] if_pc_src;
    logic [31:0] if_pc_branch_in;
    logic [1:0] imem_status;
    
    // Track fetch PC through pipeline stages
    logic [31:0] fetch_pc_d;  // PC of instruction in decode
    logic [31:0] fetch_pc_e;  // PC of instruction in execute

    always_ff @(posedge clk) begin
        if (rst) begin
            pc <= `TEXT_BASE;
            fetch_pc_d <= `TEXT_BASE;
        end else if (misprediction_flush) begin
            // Misprediction: redirect to correct target
            pc <= correct_pc;
        end else begin
            last_pc <= pc;
            case (if_pc_src)
                2'h0: begin
                    if (use_prediction) begin
                        // BTB hit: use predicted target
                        pc <= predicted_pc;
                    end else begin
                        pc <= stall ? pc : pc + 32'd4;
                    end
                end
                2'h1: begin
                    // Branch resolved: use actual target
                    pc <= if_pc_branch_in;
                end
                default: begin
                    pc <= `TEXT_BASE;
                end
            endcase
        end
        
        // Track fetch PC for prediction tracking
        if (!stall) begin
            fetch_pc_d <= pc;
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
        .stall(dmem_stall || (branch_stall && !use_prediction) || misprediction_flush),
        .addr(pc),
        .r_data(inst),
        .r_data_status(imem_status),
        .hit_count(icache_hit_count),
        .miss_count(icache_miss_count)
    );

    /*
     * Instruction Decoding Stage.
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

    // Flush decode on misprediction
    logic decode_flush;
    assign decode_flush = misprediction_flush;
    
    Decode decode(
        .inst(decode_flush ? 32'b0 : inst),  // NOP on flush
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
    
    // Track prediction info through decode stage
    always_ff @(posedge clk) begin
        if (rst || decode_flush) begin
            predicted_target_d <= 32'b0;
            predicted_taken_d <= 1'b0;
            saved_ghr_d <= '0;
            saved_ras_checkpoint_d <= 9'b0;
        end else if (!stall) begin
            // Save prediction for when branch reaches EX stage
            predicted_target_d <= use_prediction ? predicted_pc : (pc + 32'd4);
            predicted_taken_d <= use_prediction;
            // Save GHR at time of prediction for proper PHT updates
            saved_ghr_d <= current_ghr;
            // Save RAS checkpoint for recovery on misprediction
            saved_ras_checkpoint_d <= ras_checkpoint_out;
        end
    end

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

    // Flush execute on misprediction
    logic execute_flush;
    assign execute_flush = misprediction_flush;
    
    Execute execute (
        .clk(clk),
        .rst(rst || execute_flush),
        .shamt_d(shamt_d),
        .rd1_d(rd1_d),
        .rd2_d(rd2_d),
        .rt_d(rt_d),
        .rd_d(rd_d),
        .imm_d(imm_d),
        .reg_write_d(execute_flush ? 1'b0 : reg_write_d),
        .mem_to_reg_d(execute_flush ? 1'b0 : mem_to_reg_d),
        .mem_write_d(execute_flush ? 1'b0 : mem_write_d),
        .branch_d(execute_flush ? 1'b0 : branch_d),
        .alu_control_d(alu_control_d),
        .alu_src_d(alu_src_d),
        .reg_dst_d(reg_dst_d),
        .jump_addr_d(jump_addr_d),
        .branch_type_d(execute_flush ? 4'b0 : branch_type_d),
        .mem_access_d(execute_flush ? 1'b0 : mem_access_d),
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
    
    // Track prediction info and PC through execute stage
    always_ff @(posedge clk) begin
        if (rst || execute_flush) begin
            predicted_target_e <= 32'b0;
            predicted_taken_e <= 1'b0;
            saved_ghr_e <= '0;
            saved_ras_checkpoint_e <= 9'b0;
            fetch_pc_e <= 32'b0;
        end else if (!stall) begin
            predicted_target_e <= predicted_target_d;
            predicted_taken_e <= predicted_taken_d;
            saved_ghr_e <= saved_ghr_d;
            saved_ras_checkpoint_e <= saved_ras_checkpoint_d;
            fetch_pc_e <= fetch_pc_d;
        end
    end

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
    
    // Branch resolution - compute actual outcomes for predictor and misprediction detection
    logic branch_taken_e;
    logic [31:0] actual_target_e;
    
    always_comb begin
        // Use Memory module's computed branch decision
        branch_taken_e = (if_pc_src == 2'b1);
        actual_target_e = if_pc_branch_in;
    end
    
    // Connect branch resolution to branch unit
    assign branch_resolve_valid = branch_e;
    assign branch_resolve_pc = fetch_pc_e;  // PC of branch instruction in EX
    assign branch_resolve_target = actual_target_e;
    assign branch_resolve_taken = branch_taken_e;
    assign branch_resolve_type = branch_type_e;

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
