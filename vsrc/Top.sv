/* verilator lint_off UNUSEDPARAM */
`include "Config.svh"
`include "BranchTypes.svh"

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
     * Configuration Parameters for OoO Frontend (Milestone 3)
     * Set OOO_ENABLED to 1 when backend (M4/M5) is implemented
     */
    localparam OOO_ENABLED = 0;             // 0 = in-order, 1 = OoO frontend
    localparam FETCH_WIDTH = 4;             // Instructions per fetch
    localparam DECODE_WIDTH = 4;            // Instructions per decode
    localparam NUM_PHYS_REGS = 64;          // Physical registers
    localparam QUEUE_DEPTH = 32;            // Instruction queue depth
    localparam CHECKPOINT_COUNT = 8;        // RAT checkpoints

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

    /*
     * ========================================================================
     * Out-of-Order Frontend (Milestone 3)
     * ========================================================================
     * 
     * The following modules implement the OoO frontend pipeline:
     * - FreeList: Physical register management
     * - PhysRegFile: Extended register file (64 physical registers)
     * - InstructionQueue: Fetch buffer (32 entries)
     * - DecodeUnit: Parallel decode (4-wide)
     * - RenameUnit: Register renaming with RAT
     *
     * These components are instantiated and ready for integration with the
     * OoO backend (Milestones 4 and 5). When OOO_ENABLED=1, these will be
     * connected to the main pipeline.
     */

    /* verilator lint_off UNUSEDSIGNAL */

    // ========== Free List - Physical Register Management ==========
    logic [FETCH_WIDTH-1:0] fl_alloc_req;
    logic [5:0] fl_alloc_preg [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] fl_alloc_valid;
    logic [FETCH_WIDTH-1:0] fl_free_req;
    logic [5:0] fl_free_preg [FETCH_WIDTH-1:0];
    logic [6:0] fl_free_count;
    logic fl_empty, fl_full;
    logic fl_checkpoint_save, fl_checkpoint_restore;
    logic [2:0] fl_checkpoint_slot, fl_next_checkpoint_slot;

    FreeList #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .NUM_ARCH_REGS(32),
        .ALLOC_WIDTH(FETCH_WIDTH),
        .FREE_WIDTH(FETCH_WIDTH),
        .CHECKPOINT_COUNT(CHECKPOINT_COUNT)
    ) ooo_free_list (
        .clk(clk),
        .rst(rst),
        .alloc_req(fl_alloc_req),
        .alloc_preg(fl_alloc_preg),
        .alloc_valid(fl_alloc_valid),
        .free_req(fl_free_req),
        .free_preg(fl_free_preg),
        .free_count(fl_free_count),
        .empty(fl_empty),
        .full(fl_full),
        .checkpoint_save(fl_checkpoint_save),
        .checkpoint_slot(fl_checkpoint_slot),
        .checkpoint_restore(fl_checkpoint_restore),
        .next_checkpoint_slot(fl_next_checkpoint_slot)
    );

    // ========== Physical Register File ==========
    logic [5:0] prf_read_addr [7:0];
    logic [31:0] prf_read_data [7:0];
    logic [7:0] prf_read_ready;
    logic [3:0] prf_write_en;
    logic [5:0] prf_write_addr [3:0];
    logic [31:0] prf_write_data [3:0];
    logic [3:0] prf_set_ready;
    logic [5:0] prf_clear_ready_addr [3:0];
    logic [3:0] prf_clear_ready;
    logic [31:0] prf_bypass_data [7:0];
    logic [7:0] prf_bypass_valid;
    logic [31:0] prf_arch_regs [31:0];
    logic prf_magic;

    PhysRegFile #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .NUM_ARCH_REGS(32),
        .READ_PORTS(8),
        .WRITE_PORTS(4)
    ) phys_reg_file (
        .clk(clk),
        .rst(rst),
        .read_addr(prf_read_addr),
        .read_data(prf_read_data),
        .read_ready(prf_read_ready),
        .write_en(prf_write_en),
        .write_addr(prf_write_addr),
        .write_data(prf_write_data),
        .set_ready(prf_set_ready),
        .clear_ready_addr(prf_clear_ready_addr),
        .clear_ready(prf_clear_ready),
        .bypass_data(prf_bypass_data),
        .bypass_valid(prf_bypass_valid),
        .arch_regs(prf_arch_regs),
        .magic(prf_magic)
    );

    // ========== Instruction Queue ==========
    logic [31:0] iq_enq_insts [FETCH_WIDTH-1:0];
    logic [31:0] iq_enq_pcs [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] iq_enq_valid;
    logic [2:0] iq_enq_count;
    logic iq_enq_ready;
    logic [31:0] iq_enq_predicted_target;
    logic iq_enq_predicted_taken;
    logic [31:0] iq_deq_insts [DECODE_WIDTH-1:0];
    logic [31:0] iq_deq_pcs [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] iq_deq_valid;
    logic [2:0] iq_deq_count;
    logic iq_deq_ready;
    logic iq_deq_ack;
    logic [31:0] iq_deq_predicted_target [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] iq_deq_predicted_taken;
    logic iq_flush;
    logic [5:0] iq_queue_count;
    logic iq_queue_empty, iq_queue_full;
    logic [5:0] iq_free_slots;
    logic iq_fetch_stall;

    InstructionQueue #(
        .QUEUE_DEPTH(QUEUE_DEPTH),
        .ENQUEUE_WIDTH(FETCH_WIDTH),
        .DEQUEUE_WIDTH(DECODE_WIDTH)
    ) inst_queue (
        .clk(clk),
        .rst(rst),
        .enq_insts(iq_enq_insts),
        .enq_pcs(iq_enq_pcs),
        .enq_valid(iq_enq_valid),
        .enq_count(iq_enq_count),
        .enq_ready(iq_enq_ready),
        .enq_predicted_target(iq_enq_predicted_target),
        .enq_predicted_taken(iq_enq_predicted_taken),
        .deq_insts(iq_deq_insts),
        .deq_pcs(iq_deq_pcs),
        .deq_valid(iq_deq_valid),
        .deq_count(iq_deq_count),
        .deq_ready(iq_deq_ready),
        .deq_ack(iq_deq_ack),
        .deq_predicted_target(iq_deq_predicted_target),
        .deq_predicted_taken(iq_deq_predicted_taken),
        .flush(iq_flush),
        .queue_count(iq_queue_count),
        .queue_empty(iq_queue_empty),
        .queue_full(iq_queue_full),
        .free_slots(iq_free_slots),
        .fetch_stall(iq_fetch_stall)
    );

    // ========== Decode Unit ==========
    logic [DECODE_WIDTH-1:0] du_dec_valid;
    logic [DECODE_WIDTH-1:0] du_dec_reg_write;
    logic [DECODE_WIDTH-1:0] du_dec_mem_to_reg;
    logic [DECODE_WIDTH-1:0] du_dec_mem_write;
    logic [DECODE_WIDTH-1:0] du_dec_mem_read;
    logic [DECODE_WIDTH-1:0] du_dec_branch;
    logic [3:0] du_dec_alu_control [DECODE_WIDTH-1:0];
    logic [1:0] du_dec_alu_src [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] du_dec_reg_dst;
    logic [4:0] du_dec_rs [DECODE_WIDTH-1:0];
    logic [4:0] du_dec_rt [DECODE_WIDTH-1:0];
    logic [4:0] du_dec_rd [DECODE_WIDTH-1:0];
    logic [4:0] du_dec_dest [DECODE_WIDTH-1:0];
    logic [31:0] du_dec_imm [DECODE_WIDTH-1:0];
    logic [4:0] du_dec_shamt [DECODE_WIDTH-1:0];
    logic [31:0] du_dec_pc [DECODE_WIDTH-1:0];
    logic [31:0] du_dec_pc_plus_4 [DECODE_WIDTH-1:0];
    logic [31:0] du_dec_branch_target [DECODE_WIDTH-1:0];
    logic [31:0] du_dec_jump_target [DECODE_WIDTH-1:0];
    logic [3:0] du_dec_branch_type [DECODE_WIDTH-1:0];
    logic [31:0] du_dec_predicted_target [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] du_dec_predicted_taken;
    logic [DECODE_WIDTH-1:0][DECODE_WIDTH-1:0] du_dep_matrix;
    logic du_decode_ack;
    logic [2:0] du_decoded_count;
    logic du_stall;
    logic du_flush;

    DecodeUnit #(
        .DECODE_WIDTH(DECODE_WIDTH)
    ) decode_unit (
        .clk(clk),
        .rst(rst),
        .insts(iq_deq_insts),
        .pcs(iq_deq_pcs),
        .valid_in(iq_deq_valid),
        .queue_ready(iq_deq_ready),
        .predicted_target(iq_deq_predicted_target),
        .predicted_taken(iq_deq_predicted_taken),
        .stall(du_stall),
        .flush(du_flush),
        .dec_valid(du_dec_valid),
        .dec_reg_write(du_dec_reg_write),
        .dec_mem_to_reg(du_dec_mem_to_reg),
        .dec_mem_write(du_dec_mem_write),
        .dec_mem_read(du_dec_mem_read),
        .dec_branch(du_dec_branch),
        .dec_alu_control(du_dec_alu_control),
        .dec_alu_src(du_dec_alu_src),
        .dec_reg_dst(du_dec_reg_dst),
        .dec_rs(du_dec_rs),
        .dec_rt(du_dec_rt),
        .dec_rd(du_dec_rd),
        .dec_dest(du_dec_dest),
        .dec_imm(du_dec_imm),
        .dec_shamt(du_dec_shamt),
        .dec_pc(du_dec_pc),
        .dec_pc_plus_4(du_dec_pc_plus_4),
        .dec_branch_target(du_dec_branch_target),
        .dec_jump_target(du_dec_jump_target),
        .dec_branch_type(du_dec_branch_type),
        .dec_predicted_target(du_dec_predicted_target),
        .dec_predicted_taken(du_dec_predicted_taken),
        .dep_matrix(du_dep_matrix),
        .decode_ack(du_decode_ack),
        .decoded_count(du_decoded_count)
    );

    // ========== Rename Unit ==========
    logic [DECODE_WIDTH-1:0] ru_ren_valid;
    logic [5:0] ru_ren_prs1 [DECODE_WIDTH-1:0];
    logic [5:0] ru_ren_prs2 [DECODE_WIDTH-1:0];
    logic [5:0] ru_ren_prd [DECODE_WIDTH-1:0];
    logic [5:0] ru_ren_old_prd [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] ru_ren_prs1_ready;
    logic [DECODE_WIDTH-1:0] ru_ren_prs2_ready;
    logic [DECODE_WIDTH-1:0] ru_ren_reg_write;
    logic [DECODE_WIDTH-1:0] ru_ren_mem_to_reg;
    logic [DECODE_WIDTH-1:0] ru_ren_mem_write;
    logic [DECODE_WIDTH-1:0] ru_ren_mem_read;
    logic [DECODE_WIDTH-1:0] ru_ren_branch;
    logic [3:0] ru_ren_alu_control [DECODE_WIDTH-1:0];
    logic [1:0] ru_ren_alu_src [DECODE_WIDTH-1:0];
    logic [31:0] ru_ren_imm [DECODE_WIDTH-1:0];
    logic [4:0] ru_ren_shamt [DECODE_WIDTH-1:0];
    logic [31:0] ru_ren_pc [DECODE_WIDTH-1:0];
    logic [31:0] ru_ren_pc_plus_4 [DECODE_WIDTH-1:0];
    logic [31:0] ru_ren_branch_target [DECODE_WIDTH-1:0];
    logic [31:0] ru_ren_jump_target [DECODE_WIDTH-1:0];
    logic [3:0] ru_ren_branch_type [DECODE_WIDTH-1:0];
    logic [31:0] ru_ren_predicted_target [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] ru_ren_predicted_taken;
    logic [2:0] ru_ren_checkpoint_id [DECODE_WIDTH-1:0];
    logic [DECODE_WIDTH-1:0] ru_ren_checkpoint_valid;
    logic ru_rename_stall;
    logic [2:0] ru_renamed_count;
    logic ru_stall;
    logic ru_flush;
    logic [2:0] ru_flush_checkpoint;
    logic [DECODE_WIDTH-1:0] ru_commit_valid;
    logic [5:0] ru_commit_old_preg [DECODE_WIDTH-1:0];

    RenameUnit #(
        .RENAME_WIDTH(DECODE_WIDTH),
        .NUM_ARCH_REGS(32),
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .CHECKPOINT_COUNT(CHECKPOINT_COUNT)
    ) rename_unit (
        .clk(clk),
        .rst(rst),
        .dec_valid(du_dec_valid),
        .dec_rs(du_dec_rs),
        .dec_rt(du_dec_rt),
        .dec_dest(du_dec_dest),
        .dec_reg_write(du_dec_reg_write),
        .dec_branch(du_dec_branch),
        .dec_branch_type(du_dec_branch_type),
        .dec_mem_to_reg(du_dec_mem_to_reg),
        .dec_mem_write(du_dec_mem_write),
        .dec_mem_read(du_dec_mem_read),
        .dec_alu_control(du_dec_alu_control),
        .dec_alu_src(du_dec_alu_src),
        .dec_imm(du_dec_imm),
        .dec_shamt(du_dec_shamt),
        .dec_pc(du_dec_pc),
        .dec_pc_plus_4(du_dec_pc_plus_4),
        .dec_branch_target(du_dec_branch_target),
        .dec_jump_target(du_dec_jump_target),
        .dec_predicted_target(du_dec_predicted_target),
        .dec_predicted_taken(du_dec_predicted_taken),
        .stall(ru_stall),
        .flush(ru_flush),
        .flush_checkpoint(ru_flush_checkpoint),
        .freelist_alloc_req(fl_alloc_req),
        .freelist_alloc_preg(fl_alloc_preg),
        .freelist_alloc_valid(fl_alloc_valid),
        .freelist_checkpoint_save(fl_checkpoint_save),
        .freelist_checkpoint_slot(fl_checkpoint_slot),
        .freelist_checkpoint_restore(fl_checkpoint_restore),
        .commit_valid(ru_commit_valid),
        .commit_old_preg(ru_commit_old_preg),
        .freelist_free_req(fl_free_req),
        .freelist_free_preg(fl_free_preg),
        .ren_valid(ru_ren_valid),
        .ren_prs1(ru_ren_prs1),
        .ren_prs2(ru_ren_prs2),
        .ren_prd(ru_ren_prd),
        .ren_old_prd(ru_ren_old_prd),
        .ren_prs1_ready(ru_ren_prs1_ready),
        .ren_prs2_ready(ru_ren_prs2_ready),
        .ren_reg_write(ru_ren_reg_write),
        .ren_mem_to_reg(ru_ren_mem_to_reg),
        .ren_mem_write(ru_ren_mem_write),
        .ren_mem_read(ru_ren_mem_read),
        .ren_branch(ru_ren_branch),
        .ren_alu_control(ru_ren_alu_control),
        .ren_alu_src(ru_ren_alu_src),
        .ren_imm(ru_ren_imm),
        .ren_shamt(ru_ren_shamt),
        .ren_pc(ru_ren_pc),
        .ren_pc_plus_4(ru_ren_pc_plus_4),
        .ren_branch_target(ru_ren_branch_target),
        .ren_jump_target(ru_ren_jump_target),
        .ren_branch_type(ru_ren_branch_type),
        .ren_predicted_target(ru_ren_predicted_target),
        .ren_predicted_taken(ru_ren_predicted_taken),
        .ren_checkpoint_id(ru_ren_checkpoint_id),
        .ren_checkpoint_valid(ru_ren_checkpoint_valid),
        .rename_stall(ru_rename_stall),
        .renamed_count(ru_renamed_count)
    );

    // OoO Frontend control signals (active when OOO_ENABLED=1)
    // Currently tied off since backend (M4/M5) is not yet implemented
    assign iq_enq_insts = '{default: 32'b0};
    assign iq_enq_pcs = '{default: 32'b0};
    assign iq_enq_valid = '0;
    assign iq_enq_count = '0;
    assign iq_enq_ready = 1'b0;
    assign iq_enq_predicted_target = 32'b0;
    assign iq_enq_predicted_taken = 1'b0;
    assign iq_deq_ack = 1'b0;
    assign iq_flush = 1'b0;
    assign du_stall = 1'b0;
    assign du_flush = 1'b0;
    assign ru_stall = 1'b0;
    assign ru_flush = 1'b0;
    assign ru_flush_checkpoint = 3'b0;
    assign ru_commit_valid = '0;
    assign ru_commit_old_preg = '{default: 6'b0};
    
    // Physical register file control (tied off until OoO backend)
    assign prf_read_addr = '{default: 6'b0};
    assign prf_write_en = '0;
    assign prf_write_addr = '{default: 6'b0};
    assign prf_write_data = '{default: 32'b0};
    assign prf_set_ready = '0;
    assign prf_clear_ready_addr = '{default: 6'b0};
    assign prf_clear_ready = '0;

    /*
     * ========================================================================
     * Out-of-Order Backend (Milestone 4)
     * ========================================================================
     * 
     * The following modules implement the OoO backend pipeline:
     * - ReservationStation: Dynamic scheduling with wake-up logic
     * - IssueUnit: Instruction selection and dispatch to FUs
     * - IntegerUnits: ALU, multiplier, divider
     * - BranchExecute: OoO branch resolution
     * - CDB: Result broadcast network
     * - BypassNetwork: Operand forwarding
     *
     * These components are instantiated and ready for integration.
     * When OOO_ENABLED=1 and M5 (ROB) is implemented, these will be
     * connected to the main pipeline.
     */

    // ========== Reservation Station ==========
    localparam RS_ENTRIES = 16;
    localparam RS_ISSUE_WIDTH = 2;
    localparam CDB_WIDTH = 4;
    localparam NUM_ALU = 2;
    localparam NUM_MEM = 1;
    localparam NUM_BRANCH = 1;

    // RS dispatch interface (from RenameUnit)
    logic [FETCH_WIDTH-1:0] rs_dispatch_valid;
    logic [5:0] rs_dispatch_prs1 [FETCH_WIDTH-1:0];
    logic [5:0] rs_dispatch_prs2 [FETCH_WIDTH-1:0];
    logic [5:0] rs_dispatch_prd [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_prs1_value [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_prs2_value [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rs_dispatch_prs1_ready;
    logic [FETCH_WIDTH-1:0] rs_dispatch_prs2_ready;
    logic [3:0] rs_dispatch_alu_control [FETCH_WIDTH-1:0];
    logic [1:0] rs_dispatch_alu_src [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_imm [FETCH_WIDTH-1:0];
    logic [4:0] rs_dispatch_shamt [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rs_dispatch_reg_write;
    logic [FETCH_WIDTH-1:0] rs_dispatch_mem_to_reg;
    logic [FETCH_WIDTH-1:0] rs_dispatch_mem_write;
    logic [FETCH_WIDTH-1:0] rs_dispatch_mem_read;
    logic [FETCH_WIDTH-1:0] rs_dispatch_branch;
    logic [31:0] rs_dispatch_pc [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_pc_plus_4 [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_branch_target [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_jump_target [FETCH_WIDTH-1:0];
    logic [3:0] rs_dispatch_branch_type [FETCH_WIDTH-1:0];
    logic [31:0] rs_dispatch_predicted_target [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rs_dispatch_predicted_taken;
    logic [6:0] rs_dispatch_rob_idx [FETCH_WIDTH-1:0];

    // RS issue interface (to IssueUnit)
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_valid;
    logic [5:0] rs_issue_prd [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_src1 [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_src2 [RS_ISSUE_WIDTH-1:0];
    logic [3:0] rs_issue_alu_control [RS_ISSUE_WIDTH-1:0];
    logic [1:0] rs_issue_alu_src [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_imm [RS_ISSUE_WIDTH-1:0];
    logic [4:0] rs_issue_shamt [RS_ISSUE_WIDTH-1:0];
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_reg_write;
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_mem_to_reg;
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_mem_write;
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_mem_read;
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_branch;
    logic [31:0] rs_issue_pc [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_pc_plus_4 [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_branch_target [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_jump_target [RS_ISSUE_WIDTH-1:0];
    logic [3:0] rs_issue_branch_type [RS_ISSUE_WIDTH-1:0];
    logic [31:0] rs_issue_predicted_target [RS_ISSUE_WIDTH-1:0];
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_predicted_taken;
    logic [6:0] rs_issue_rob_idx [RS_ISSUE_WIDTH-1:0];
    logic [$clog2(RS_ENTRIES)-1:0] rs_issue_idx [RS_ISSUE_WIDTH-1:0];
    logic [RS_ISSUE_WIDTH-1:0] rs_issue_ack;
    logic [$clog2(RS_ENTRIES):0] rs_free_count;
    logic rs_full, rs_empty;

    // CDB signals
    logic [CDB_WIDTH-1:0] cdb_valid;
    logic [5:0] cdb_tag [CDB_WIDTH-1:0];
    logic [31:0] cdb_data [CDB_WIDTH-1:0];
    logic [6:0] cdb_rob_idx [CDB_WIDTH-1:0];

    // RS flush
    logic rs_flush;
    assign rs_flush = flush_pipeline;

    ReservationStation #(
        .NUM_ENTRIES(RS_ENTRIES),
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .DISPATCH_WIDTH(FETCH_WIDTH),
        .ISSUE_WIDTH(RS_ISSUE_WIDTH),
        .CDB_WIDTH(CDB_WIDTH)
    ) reservation_station (
        .clk(clk),
        .rst(rst),
        .dispatch_valid(rs_dispatch_valid),
        .dispatch_prs1(rs_dispatch_prs1),
        .dispatch_prs2(rs_dispatch_prs2),
        .dispatch_prd(rs_dispatch_prd),
        .dispatch_prs1_value(rs_dispatch_prs1_value),
        .dispatch_prs2_value(rs_dispatch_prs2_value),
        .dispatch_prs1_ready(rs_dispatch_prs1_ready),
        .dispatch_prs2_ready(rs_dispatch_prs2_ready),
        .dispatch_alu_control(rs_dispatch_alu_control),
        .dispatch_alu_src(rs_dispatch_alu_src),
        .dispatch_imm(rs_dispatch_imm),
        .dispatch_shamt(rs_dispatch_shamt),
        .dispatch_reg_write(rs_dispatch_reg_write),
        .dispatch_mem_to_reg(rs_dispatch_mem_to_reg),
        .dispatch_mem_write(rs_dispatch_mem_write),
        .dispatch_mem_read(rs_dispatch_mem_read),
        .dispatch_branch(rs_dispatch_branch),
        .dispatch_pc(rs_dispatch_pc),
        .dispatch_pc_plus_4(rs_dispatch_pc_plus_4),
        .dispatch_branch_target(rs_dispatch_branch_target),
        .dispatch_jump_target(rs_dispatch_jump_target),
        .dispatch_branch_type(rs_dispatch_branch_type),
        .dispatch_predicted_target(rs_dispatch_predicted_target),
        .dispatch_predicted_taken(rs_dispatch_predicted_taken),
        .dispatch_rob_idx(rs_dispatch_rob_idx),
        .cdb_valid(cdb_valid),
        .cdb_tag(cdb_tag),
        .cdb_data(cdb_data),
        .issue_valid(rs_issue_valid),
        .issue_prd(rs_issue_prd),
        .issue_src1(rs_issue_src1),
        .issue_src2(rs_issue_src2),
        .issue_alu_control(rs_issue_alu_control),
        .issue_alu_src(rs_issue_alu_src),
        .issue_imm(rs_issue_imm),
        .issue_shamt(rs_issue_shamt),
        .issue_reg_write(rs_issue_reg_write),
        .issue_mem_to_reg(rs_issue_mem_to_reg),
        .issue_mem_write(rs_issue_mem_write),
        .issue_mem_read(rs_issue_mem_read),
        .issue_branch(rs_issue_branch),
        .issue_pc(rs_issue_pc),
        .issue_pc_plus_4(rs_issue_pc_plus_4),
        .issue_branch_target(rs_issue_branch_target),
        .issue_jump_target(rs_issue_jump_target),
        .issue_branch_type(rs_issue_branch_type),
        .issue_predicted_target(rs_issue_predicted_target),
        .issue_predicted_taken(rs_issue_predicted_taken),
        .issue_rob_idx(rs_issue_rob_idx),
        .issue_rs_idx(rs_issue_idx),
        .issue_ack(rs_issue_ack),
        .flush(rs_flush),
        .free_count(rs_free_count),
        .full(rs_full),
        .empty(rs_empty)
    );

    // ========== Issue Unit ==========
    // IssueUnit outputs to functional units
    logic [NUM_ALU-1:0] iu_alu_valid;
    logic [5:0] iu_alu_prd [NUM_ALU-1:0];
    logic [31:0] iu_alu_src1 [NUM_ALU-1:0];
    logic [31:0] iu_alu_src2 [NUM_ALU-1:0];
    logic [3:0] iu_alu_control [NUM_ALU-1:0];
    logic [1:0] iu_alu_src_sel [NUM_ALU-1:0];
    logic [31:0] iu_alu_imm [NUM_ALU-1:0];
    logic [4:0] iu_alu_shamt [NUM_ALU-1:0];
    logic [NUM_ALU-1:0] iu_alu_reg_write;
    logic [31:0] iu_alu_pc [NUM_ALU-1:0];
    logic [31:0] iu_alu_pc_plus_4 [NUM_ALU-1:0];
    logic [6:0] iu_alu_rob_idx [NUM_ALU-1:0];

    logic [NUM_MEM-1:0] iu_mem_valid;
    logic [5:0] iu_mem_prd [NUM_MEM-1:0];
    logic [31:0] iu_mem_addr [NUM_MEM-1:0];
    logic [31:0] iu_mem_data [NUM_MEM-1:0];
    logic [NUM_MEM-1:0] iu_mem_write;
    logic [NUM_MEM-1:0] iu_mem_read;
    logic [6:0] iu_mem_rob_idx [NUM_MEM-1:0];

    logic [NUM_BRANCH-1:0] iu_branch_valid;
    logic [31:0] iu_branch_src1 [NUM_BRANCH-1:0];
    logic [31:0] iu_branch_src2 [NUM_BRANCH-1:0];
    logic [31:0] iu_branch_pc [NUM_BRANCH-1:0];
    logic [31:0] iu_branch_target [NUM_BRANCH-1:0];
    logic [31:0] iu_branch_jump_target [NUM_BRANCH-1:0];
    logic [3:0] iu_branch_type [NUM_BRANCH-1:0];
    logic [31:0] iu_branch_predicted_target [NUM_BRANCH-1:0];
    logic [NUM_BRANCH-1:0] iu_branch_predicted_taken;
    logic [6:0] iu_branch_rob_idx [NUM_BRANCH-1:0];

    // FU availability
    logic [NUM_ALU-1:0] fu_alu_available;
    logic [NUM_MEM-1:0] fu_mem_available;
    logic [NUM_BRANCH-1:0] fu_branch_available;

    IssueUnit #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .RS_ENTRIES(RS_ENTRIES),
        .ISSUE_WIDTH(RS_ISSUE_WIDTH),
        .NUM_ALU(NUM_ALU),
        .NUM_MEM(NUM_MEM),
        .NUM_BRANCH(NUM_BRANCH)
    ) issue_unit (
        .clk(clk),
        .rst(rst),
        .int_rs_valid(rs_issue_valid),
        .int_rs_prd(rs_issue_prd),
        .int_rs_src1(rs_issue_src1),
        .int_rs_src2(rs_issue_src2),
        .int_rs_alu_control(rs_issue_alu_control),
        .int_rs_alu_src(rs_issue_alu_src),
        .int_rs_imm(rs_issue_imm),
        .int_rs_shamt(rs_issue_shamt),
        .int_rs_reg_write(rs_issue_reg_write),
        .int_rs_mem_to_reg(rs_issue_mem_to_reg),
        .int_rs_mem_write(rs_issue_mem_write),
        .int_rs_mem_read(rs_issue_mem_read),
        .int_rs_branch(rs_issue_branch),
        .int_rs_pc(rs_issue_pc),
        .int_rs_pc_plus_4(rs_issue_pc_plus_4),
        .int_rs_branch_target(rs_issue_branch_target),
        .int_rs_jump_target(rs_issue_jump_target),
        .int_rs_branch_type(rs_issue_branch_type),
        .int_rs_predicted_target(rs_issue_predicted_target),
        .int_rs_predicted_taken(rs_issue_predicted_taken),
        .int_rs_rob_idx(rs_issue_rob_idx),
        .int_rs_idx(rs_issue_idx),
        .int_rs_ack(rs_issue_ack),
        .alu_available(fu_alu_available),
        .mem_available(fu_mem_available),
        .branch_available(fu_branch_available),
        .alu_issue_valid(iu_alu_valid),
        .alu_issue_prd(iu_alu_prd),
        .alu_issue_src1(iu_alu_src1),
        .alu_issue_src2(iu_alu_src2),
        .alu_issue_alu_control(iu_alu_control),
        .alu_issue_alu_src(iu_alu_src_sel),
        .alu_issue_imm(iu_alu_imm),
        .alu_issue_shamt(iu_alu_shamt),
        .alu_issue_reg_write(iu_alu_reg_write),
        .alu_issue_pc(iu_alu_pc),
        .alu_issue_pc_plus_4(iu_alu_pc_plus_4),
        .alu_issue_rob_idx(iu_alu_rob_idx),
        .mem_issue_valid(iu_mem_valid),
        .mem_issue_prd(iu_mem_prd),
        .mem_issue_addr(iu_mem_addr),
        .mem_issue_data(iu_mem_data),
        .mem_issue_mem_write(iu_mem_write),
        .mem_issue_mem_read(iu_mem_read),
        .mem_issue_rob_idx(iu_mem_rob_idx),
        .branch_issue_valid(iu_branch_valid),
        .branch_issue_src1(iu_branch_src1),
        .branch_issue_src2(iu_branch_src2),
        .branch_issue_pc(iu_branch_pc),
        .branch_issue_branch_target(iu_branch_target),
        .branch_issue_jump_target(iu_branch_jump_target),
        .branch_issue_branch_type(iu_branch_type),
        .branch_issue_predicted_target(iu_branch_predicted_target),
        .branch_issue_predicted_taken(iu_branch_predicted_taken),
        .branch_issue_rob_idx(iu_branch_rob_idx),
        .flush(rs_flush)
    );

    // ========== Integer Units ==========
    logic [NUM_ALU-1:0] int_alu_result_valid;
    logic [5:0] int_alu_result_prd [NUM_ALU-1:0];
    logic [31:0] int_alu_result_data [NUM_ALU-1:0];
    logic [6:0] int_alu_result_rob_idx [NUM_ALU-1:0];

    // Multiplier (tied off for now - no MUL instructions in test set)
    logic mul_valid_tie;
    logic [5:0] mul_prd_tie;
    logic [31:0] mul_src1_tie, mul_src2_tie;
    logic mul_signed_tie;
    logic [6:0] mul_rob_idx_tie;
    logic mul_available;
    logic mul_result_valid;
    logic [5:0] mul_result_prd;
    logic [31:0] mul_result_data;
    logic [6:0] mul_result_rob_idx;

    // Divider (tied off for now - no DIV instructions in test set)
    logic div_valid_tie;
    logic [5:0] div_prd_tie;
    logic [31:0] div_src1_tie, div_src2_tie;
    logic div_signed_tie, div_remainder_tie;
    logic [6:0] div_rob_idx_tie;
    logic div_available;
    logic div_result_valid;
    logic [5:0] div_result_prd;
    logic [31:0] div_result_data;
    logic [6:0] div_result_rob_idx;

    assign mul_valid_tie = 1'b0;
    assign mul_prd_tie = '0;
    assign mul_src1_tie = '0;
    assign mul_src2_tie = '0;
    assign mul_signed_tie = 1'b0;
    assign mul_rob_idx_tie = '0;
    assign div_valid_tie = 1'b0;
    assign div_prd_tie = '0;
    assign div_src1_tie = '0;
    assign div_src2_tie = '0;
    assign div_signed_tie = 1'b0;
    assign div_remainder_tie = 1'b0;
    assign div_rob_idx_tie = '0;

    IntegerUnits #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .NUM_ALU(NUM_ALU),
        .CDB_WIDTH(CDB_WIDTH)
    ) integer_units (
        .clk(clk),
        .rst(rst),
        .alu_valid(iu_alu_valid),
        .alu_prd(iu_alu_prd),
        .alu_src1(iu_alu_src1),
        .alu_src2(iu_alu_src2),
        .alu_control(iu_alu_control),
        .alu_src_sel(iu_alu_src_sel),
        .alu_imm(iu_alu_imm),
        .alu_shamt(iu_alu_shamt),
        .alu_reg_write(iu_alu_reg_write),
        .alu_pc(iu_alu_pc),
        .alu_pc_plus_4(iu_alu_pc_plus_4),
        .alu_rob_idx(iu_alu_rob_idx),
        .alu_available(fu_alu_available),
        .alu_result_valid(int_alu_result_valid),
        .alu_result_prd(int_alu_result_prd),
        .alu_result_data(int_alu_result_data),
        .alu_result_rob_idx(int_alu_result_rob_idx),
        .mul_valid(mul_valid_tie),
        .mul_prd(mul_prd_tie),
        .mul_src1(mul_src1_tie),
        .mul_src2(mul_src2_tie),
        .mul_signed_op(mul_signed_tie),
        .mul_rob_idx(mul_rob_idx_tie),
        .mul_available(mul_available),
        .mul_result_valid(mul_result_valid),
        .mul_result_prd(mul_result_prd),
        .mul_result_data(mul_result_data),
        .mul_result_rob_idx(mul_result_rob_idx),
        .div_valid(div_valid_tie),
        .div_prd(div_prd_tie),
        .div_src1(div_src1_tie),
        .div_src2(div_src2_tie),
        .div_signed_op(div_signed_tie),
        .div_remainder(div_remainder_tie),
        .div_rob_idx(div_rob_idx_tie),
        .div_available(div_available),
        .div_result_valid(div_result_valid),
        .div_result_prd(div_result_prd),
        .div_result_data(div_result_data),
        .div_result_rob_idx(div_result_rob_idx),
        .flush(rs_flush)
    );

    // ========== Branch Execute Unit ==========
    logic [NUM_BRANCH-1:0] be_result_valid;
    logic [NUM_BRANCH-1:0] be_actual_taken;
    logic [31:0] be_actual_target [NUM_BRANCH-1:0];
    logic [NUM_BRANCH-1:0] be_mispredicted;
    logic [31:0] be_correct_pc [NUM_BRANCH-1:0];
    logic [6:0] be_result_rob_idx [NUM_BRANCH-1:0];

    BranchExecute #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .NUM_BRANCH(NUM_BRANCH)
    ) branch_execute (
        .clk(clk),
        .rst(rst),
        .branch_valid(iu_branch_valid),
        .branch_src1(iu_branch_src1),
        .branch_src2(iu_branch_src2),
        .branch_pc(iu_branch_pc),
        .branch_target(iu_branch_target),
        .jump_target(iu_branch_jump_target),
        .branch_type(iu_branch_type),
        .predicted_target(iu_branch_predicted_target),
        .predicted_taken(iu_branch_predicted_taken),
        .branch_rob_idx(iu_branch_rob_idx),
        .branch_available(fu_branch_available),
        .branch_result_valid(be_result_valid),
        .branch_actual_taken(be_actual_taken),
        .branch_actual_target(be_actual_target),
        .branch_mispredicted(be_mispredicted),
        .branch_correct_pc(be_correct_pc),
        .branch_result_rob_idx(be_result_rob_idx),
        .flush(rs_flush)
    );

    // ========== Common Data Bus ==========
    // Memory results (tied off until M5 LSQ)
    logic [NUM_MEM-1:0] mem_result_valid_tie;
    logic [5:0] mem_result_prd_tie [NUM_MEM-1:0];
    logic [31:0] mem_result_data_tie [NUM_MEM-1:0];
    logic [6:0] mem_result_rob_idx_tie [NUM_MEM-1:0];
    logic [NUM_MEM-1:0] mem_result_ack;
    
    assign mem_result_valid_tie = '0;
    assign mem_result_prd_tie = '{default: 6'b0};
    assign mem_result_data_tie = '{default: 32'b0};
    assign mem_result_rob_idx_tie = '{default: 7'b0};

    // CDB acknowledgments
    logic [NUM_ALU-1:0] cdb_alu_ack;
    logic cdb_mul_ack;
    logic cdb_div_ack;
    logic [NUM_BRANCH-1:0] cdb_branch_ack;

    // Branch completion signals from CDB
    logic [NUM_BRANCH-1:0] cdb_branch_complete;
    logic [NUM_BRANCH-1:0] cdb_branch_mispredicted;
    logic [31:0] cdb_branch_correct_pc [NUM_BRANCH-1:0];
    logic [6:0] cdb_branch_rob_idx [NUM_BRANCH-1:0];

    // Memory availability (tied off until M5)
    assign fu_mem_available = {NUM_MEM{1'b1}};

    CDB #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .CDB_WIDTH(CDB_WIDTH),
        .NUM_ALU(NUM_ALU),
        .NUM_MEM(NUM_MEM),
        .NUM_BRANCH(NUM_BRANCH)
    ) cdb_unit (
        .clk(clk),
        .rst(rst),
        .alu_result_valid(int_alu_result_valid),
        .alu_result_prd(int_alu_result_prd),
        .alu_result_data(int_alu_result_data),
        .alu_result_rob_idx(int_alu_result_rob_idx),
        .alu_result_ack(cdb_alu_ack),
        .mul_result_valid(mul_result_valid),
        .mul_result_prd(mul_result_prd),
        .mul_result_data(mul_result_data),
        .mul_result_rob_idx(mul_result_rob_idx),
        .mul_result_ack(cdb_mul_ack),
        .div_result_valid(div_result_valid),
        .div_result_prd(div_result_prd),
        .div_result_data(div_result_data),
        .div_result_rob_idx(div_result_rob_idx),
        .div_result_ack(cdb_div_ack),
        .mem_result_valid(mem_result_valid_tie),
        .mem_result_prd(mem_result_prd_tie),
        .mem_result_data(mem_result_data_tie),
        .mem_result_rob_idx(mem_result_rob_idx_tie),
        .mem_result_ack(mem_result_ack),
        .branch_result_valid(be_result_valid),
        .branch_mispredicted(be_mispredicted),
        .branch_correct_pc(be_correct_pc),
        .branch_result_rob_idx(be_result_rob_idx),
        .branch_result_ack(cdb_branch_ack),
        .cdb_valid(cdb_valid),
        .cdb_tag(cdb_tag),
        .cdb_data(cdb_data),
        .cdb_rob_idx(cdb_rob_idx),
        .cdb_branch_complete(cdb_branch_complete),
        .cdb_branch_mispredicted(cdb_branch_mispredicted),
        .cdb_branch_correct_pc(cdb_branch_correct_pc),
        .cdb_branch_rob_idx(cdb_branch_rob_idx),
        .flush(rs_flush)
    );

    // ========== Bypass Network ==========
    localparam BYPASS_PORTS = 8;
    
    logic [5:0] bypass_req_tag [BYPASS_PORTS-1:0];
    logic [BYPASS_PORTS-1:0] bypass_req_valid;
    logic [31:0] bypass_prf_data [BYPASS_PORTS-1:0];
    logic [BYPASS_PORTS-1:0] bypass_prf_ready;
    logic [31:0] bypass_fwd_data [BYPASS_PORTS-1:0];
    logic [BYPASS_PORTS-1:0] bypass_fwd_valid;

    // Bypass network for same-cycle forwarding (tied off until OoO enabled)
    assign bypass_req_tag = '{default: 6'b0};
    assign bypass_req_valid = '0;
    assign bypass_prf_data = '{default: 32'b0};
    assign bypass_prf_ready = '0;

    BypassNetwork #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .CDB_WIDTH(CDB_WIDTH),
        .NUM_ALU(NUM_ALU),
        .FORWARD_PORTS(BYPASS_PORTS)
    ) bypass_network (
        .clk(clk),
        .rst(rst),
        .req_tag(bypass_req_tag),
        .req_valid(bypass_req_valid),
        .cdb_valid(cdb_valid),
        .cdb_tag(cdb_tag),
        .cdb_data(cdb_data),
        .alu_valid(int_alu_result_valid),
        .alu_prd(int_alu_result_prd),
        .alu_data(int_alu_result_data),
        .prf_read_data(bypass_prf_data),
        .prf_read_ready(bypass_prf_ready),
        .fwd_data(bypass_fwd_data),
        .fwd_valid(bypass_fwd_valid)
    );

    // M4 Backend control signals (active when OOO_ENABLED=1 and M5 ROB is implemented)
    // Currently tied off since OOO backend is not yet fully integrated
    assign rs_dispatch_valid = '0;
    assign rs_dispatch_prs1 = '{default: 6'b0};
    assign rs_dispatch_prs2 = '{default: 6'b0};
    assign rs_dispatch_prd = '{default: 6'b0};
    assign rs_dispatch_prs1_value = '{default: 32'b0};
    assign rs_dispatch_prs2_value = '{default: 32'b0};
    assign rs_dispatch_prs1_ready = '0;
    assign rs_dispatch_prs2_ready = '0;
    assign rs_dispatch_alu_control = '{default: 4'b0};
    assign rs_dispatch_alu_src = '{default: 2'b0};
    assign rs_dispatch_imm = '{default: 32'b0};
    assign rs_dispatch_shamt = '{default: 5'b0};
    assign rs_dispatch_reg_write = '0;
    assign rs_dispatch_mem_to_reg = '0;
    assign rs_dispatch_mem_write = '0;
    assign rs_dispatch_mem_read = '0;
    assign rs_dispatch_branch = '0;
    assign rs_dispatch_pc = '{default: 32'b0};
    assign rs_dispatch_pc_plus_4 = '{default: 32'b0};
    assign rs_dispatch_branch_target = '{default: 32'b0};
    assign rs_dispatch_jump_target = '{default: 32'b0};
    assign rs_dispatch_branch_type = '{default: 4'b0};
    assign rs_dispatch_predicted_target = '{default: 32'b0};
    assign rs_dispatch_predicted_taken = '0;
    assign rs_dispatch_rob_idx = '{default: 7'b0};

    /*
     * ========================================================================
     * Out-of-Order Backend - Commit & Memory (Milestone 5)
     * ========================================================================
     * 
     * The following modules implement the commit and memory subsystem:
     * - ROB: Reorder buffer for in-order commit
     * - CommitUnit: Commit logic coordinator
     * - MemoryDisambiguation: Load/Store queue with forwarding
     * - RecoveryUnit: Misprediction and exception recovery
     *
     * These components are instantiated and ready for integration.
     * When OOO_ENABLED=1 and full integration is done, these will be
     * connected to complete the OoO pipeline.
     */

    // ========== ROB Parameters ==========
    localparam ROB_ENTRIES = 64;
    localparam COMMIT_WIDTH = 4;

    // ========== ROB Signals ==========
    // Dispatch interface
    logic [FETCH_WIDTH-1:0] rob_dispatch_valid;
    logic [4:0] rob_dispatch_arch_rd [FETCH_WIDTH-1:0];
    logic [5:0] rob_dispatch_phys_rd [FETCH_WIDTH-1:0];
    logic [5:0] rob_dispatch_old_phys_rd [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rob_dispatch_reg_write;
    logic [FETCH_WIDTH-1:0] rob_dispatch_mem_write;
    logic [FETCH_WIDTH-1:0] rob_dispatch_mem_read;
    logic [FETCH_WIDTH-1:0] rob_dispatch_branch;
    logic [3:0] rob_dispatch_branch_type [FETCH_WIDTH-1:0];
    logic [31:0] rob_dispatch_pc [FETCH_WIDTH-1:0];
    logic [31:0] rob_dispatch_predicted_target [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rob_dispatch_predicted_taken;
    logic [2:0] rob_dispatch_checkpoint_id [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rob_dispatch_checkpoint_valid;
    logic [6:0] rob_alloc_idx [FETCH_WIDTH-1:0];
    logic [FETCH_WIDTH-1:0] rob_alloc_valid;
    logic rob_full_flag;

    // Commit interface
    logic [COMMIT_WIDTH-1:0] rob_commit_valid;
    logic [4:0] rob_commit_arch_rd [COMMIT_WIDTH-1:0];
    logic [5:0] rob_commit_phys_rd [COMMIT_WIDTH-1:0];
    logic [5:0] rob_commit_old_phys_rd [COMMIT_WIDTH-1:0];
    logic [COMMIT_WIDTH-1:0] rob_commit_reg_write;
    logic [COMMIT_WIDTH-1:0] rob_commit_store;
    logic [COMMIT_WIDTH-1:0] rob_commit_load;
    logic [6:0] rob_commit_rob_idx [COMMIT_WIDTH-1:0];
    logic [31:0] rob_commit_pc [COMMIT_WIDTH-1:0];

    // Flush interface
    logic rob_flush_out;
    logic [31:0] rob_flush_pc;
    logic [6:0] rob_flush_rob_idx;
    logic [2:0] rob_flush_checkpoint;

    // Store commit signals
    logic [COMMIT_WIDTH-1:0] rob_store_commit_valid;
    logic [6:0] rob_store_commit_rob_idx [COMMIT_WIDTH-1:0];

    // LSQ integration
    logic rob_load_complete_valid;
    logic [6:0] rob_load_complete_rob_idx;
    logic rob_store_addr_valid;
    logic [6:0] rob_store_addr_rob_idx;

    // Status
    logic [$clog2(ROB_ENTRIES):0] rob_count;
    logic rob_empty;
    logic [6:0] rob_head_out;
    logic [6:0] rob_tail_out;

    // Tie off dispatch inputs (will be connected when OOO_ENABLED=1)
    assign rob_dispatch_valid = '0;
    assign rob_dispatch_arch_rd = '{default: 5'b0};
    assign rob_dispatch_phys_rd = '{default: 6'b0};
    assign rob_dispatch_old_phys_rd = '{default: 6'b0};
    assign rob_dispatch_reg_write = '0;
    assign rob_dispatch_mem_write = '0;
    assign rob_dispatch_mem_read = '0;
    assign rob_dispatch_branch = '0;
    assign rob_dispatch_branch_type = '{default: 4'b0};
    assign rob_dispatch_pc = '{default: 32'b0};
    assign rob_dispatch_predicted_target = '{default: 32'b0};
    assign rob_dispatch_predicted_taken = '0;
    assign rob_dispatch_checkpoint_id = '{default: 3'b0};
    assign rob_dispatch_checkpoint_valid = '0;
    assign rob_load_complete_valid = 1'b0;
    assign rob_load_complete_rob_idx = '0;
    assign rob_store_addr_valid = 1'b0;
    assign rob_store_addr_rob_idx = '0;

    ROB #(
        .ROB_ENTRIES(ROB_ENTRIES),
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .DISPATCH_WIDTH(FETCH_WIDTH),
        .COMMIT_WIDTH(COMMIT_WIDTH),
        .CDB_WIDTH(CDB_WIDTH)
    ) reorder_buffer (
        .clk(clk),
        .rst(rst),
        // Dispatch
        .dispatch_valid(rob_dispatch_valid),
        .dispatch_arch_rd(rob_dispatch_arch_rd),
        .dispatch_phys_rd(rob_dispatch_phys_rd),
        .dispatch_old_phys_rd(rob_dispatch_old_phys_rd),
        .dispatch_reg_write(rob_dispatch_reg_write),
        .dispatch_mem_write(rob_dispatch_mem_write),
        .dispatch_mem_read(rob_dispatch_mem_read),
        .dispatch_branch(rob_dispatch_branch),
        .dispatch_branch_type(rob_dispatch_branch_type),
        .dispatch_pc(rob_dispatch_pc),
        .dispatch_predicted_target(rob_dispatch_predicted_target),
        .dispatch_predicted_taken(rob_dispatch_predicted_taken),
        .dispatch_checkpoint_id(rob_dispatch_checkpoint_id),
        .dispatch_checkpoint_valid(rob_dispatch_checkpoint_valid),
        .dispatch_rob_idx(rob_alloc_idx),
        .dispatch_rob_valid(rob_alloc_valid),
        .rob_full(rob_full_flag),
        // CDB
        .cdb_valid(cdb_valid),
        .cdb_tag(cdb_tag),
        .cdb_data(cdb_data),
        .cdb_rob_idx(cdb_rob_idx),
        .cdb_branch_complete(cdb_branch_complete[0]),
        .cdb_branch_mispredicted(cdb_branch_mispredicted[0]),
        .cdb_branch_correct_pc(cdb_branch_correct_pc[0]),
        .cdb_branch_rob_idx(cdb_branch_rob_idx[0]),
        // Commit
        .commit_valid(rob_commit_valid),
        .commit_arch_rd(rob_commit_arch_rd),
        .commit_phys_rd(rob_commit_phys_rd),
        .commit_old_phys_rd(rob_commit_old_phys_rd),
        .commit_reg_write(rob_commit_reg_write),
        .commit_store(rob_commit_store),
        .commit_load(rob_commit_load),
        .commit_rob_idx(rob_commit_rob_idx),
        .commit_pc(rob_commit_pc),
        // Flush
        .flush(rob_flush_out),
        .flush_pc(rob_flush_pc),
        .flush_rob_idx(rob_flush_rob_idx),
        .flush_checkpoint_id(rob_flush_checkpoint),
        // Store commit
        .store_commit_valid(rob_store_commit_valid),
        .store_commit_rob_idx(rob_store_commit_rob_idx),
        // LSQ integration
        .load_complete_valid(rob_load_complete_valid),
        .load_complete_rob_idx(rob_load_complete_rob_idx),
        .store_addr_valid(rob_store_addr_valid),
        .store_addr_rob_idx(rob_store_addr_rob_idx),
        // Status
        .rob_count(rob_count),
        .rob_empty(rob_empty),
        .rob_head(rob_head_out),
        .rob_tail(rob_tail_out)
    );

    // ========== Commit Unit ==========
    logic [COMMIT_WIDTH-1:0] cu_commit_valid;
    logic [4:0] cu_commit_arch_rd [COMMIT_WIDTH-1:0];
    logic [5:0] cu_commit_phys_rd [COMMIT_WIDTH-1:0];
    logic [COMMIT_WIDTH-1:0] cu_commit_reg_write;
    logic [COMMIT_WIDTH-1:0] cu_free_req;
    logic [5:0] cu_free_preg [COMMIT_WIDTH-1:0];
    logic [COMMIT_WIDTH-1:0] cu_store_commit;
    logic [6:0] cu_store_commit_rob_idx [COMMIT_WIDTH-1:0];
    logic cu_flush_pipeline;
    logic [31:0] cu_flush_target_pc;
    logic [2:0] cu_flush_checkpoint;
    logic [31:0] cu_committed_insts;
    logic [31:0] cu_committed_stores;
    logic [31:0] cu_committed_loads;

    CommitUnit #(
        .COMMIT_WIDTH(COMMIT_WIDTH),
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .ROB_ENTRIES(ROB_ENTRIES)
    ) commit_unit (
        .clk(clk),
        .rst(rst),
        // ROB interface
        .rob_commit_valid(rob_commit_valid),
        .rob_commit_arch_rd(rob_commit_arch_rd),
        .rob_commit_phys_rd(rob_commit_phys_rd),
        .rob_commit_old_phys_rd(rob_commit_old_phys_rd),
        .rob_commit_reg_write(rob_commit_reg_write),
        .rob_commit_store(rob_commit_store),
        .rob_commit_load(rob_commit_load),
        .rob_commit_rob_idx(rob_commit_rob_idx),
        .rob_commit_pc(rob_commit_pc),
        .rob_flush(rob_flush_out),
        .rob_flush_pc(rob_flush_pc),
        .rob_flush_rob_idx(rob_flush_rob_idx),
        .rob_flush_checkpoint_id(rob_flush_checkpoint),
        // RenameUnit interface
        .commit_valid(cu_commit_valid),
        .commit_arch_rd(cu_commit_arch_rd),
        .commit_phys_rd(cu_commit_phys_rd),
        .commit_reg_write(cu_commit_reg_write),
        // FreeList interface
        .free_req(cu_free_req),
        .free_preg(cu_free_preg),
        // Store Queue interface
        .store_commit(cu_store_commit),
        .store_commit_rob_idx(cu_store_commit_rob_idx),
        // Recovery interface
        .flush_pipeline(cu_flush_pipeline),
        .flush_target_pc(cu_flush_target_pc),
        .flush_checkpoint(cu_flush_checkpoint),
        // Performance counters
        .committed_insts(cu_committed_insts),
        .committed_stores(cu_committed_stores),
        .committed_loads(cu_committed_loads)
    );

    // ========== Memory Disambiguation Unit (integrates LoadQueue + StoreQueue) ==========
    localparam LQ_ENTRIES = 32;
    localparam SQ_ENTRIES = 32;

    // Dispatch interface
    logic [FETCH_WIDTH-1:0] lsq_dispatch_valid;
    logic [FETCH_WIDTH-1:0] lsq_dispatch_is_load;
    logic [FETCH_WIDTH-1:0] lsq_dispatch_is_store;
    logic [5:0] lsq_dispatch_prd [FETCH_WIDTH-1:0];
    logic [6:0] lsq_dispatch_rob_idx [FETCH_WIDTH-1:0];
    logic [31:0] lsq_dispatch_pc [FETCH_WIDTH-1:0];     // PC for memory violation recovery
    logic [FETCH_WIDTH-1:0] lsq_dispatch_lsq_valid;
    logic lsq_full_flag;

    // Issue interface
    logic lsq_mem_issue_valid;
    logic lsq_mem_issue_is_load;
    logic lsq_mem_issue_is_store;
    logic [5:0] lsq_mem_issue_prd;
    logic [31:0] lsq_mem_issue_addr;
    logic [31:0] lsq_mem_issue_data;
    logic [6:0] lsq_mem_issue_rob_idx;

    // Cache interface
    logic lsq_cache_req_valid;
    logic lsq_cache_req_write;
    logic [31:0] lsq_cache_req_addr;
    logic [31:0] lsq_cache_req_data;

    // Result interface
    logic lsq_mem_result_valid;
    logic [5:0] lsq_mem_result_prd;
    logic [31:0] lsq_mem_result_data;
    logic [6:0] lsq_mem_result_rob_idx;
    logic lsq_mem_result_ack;

    // ROB interface
    logic lsq_rob_mem_complete_valid;
    logic [6:0] lsq_rob_mem_complete_rob_idx;
    logic lsq_rob_store_addr_valid;
    logic [6:0] lsq_rob_store_addr_rob_idx;

    // Recovery interface
    logic lsq_memory_violation;
    logic [6:0] lsq_violation_rob_idx;
    logic [31:0] lsq_violation_pc;          // PC of violating load for re-execution
    logic lsq_flush;
    logic [6:0] lsq_flush_rob_idx;

    // Status
    logic [$clog2(LQ_ENTRIES):0] lsq_lq_count;
    logic [$clog2(SQ_ENTRIES):0] lsq_sq_count;

    // Tie off inputs (will be connected when OOO_ENABLED=1)
    assign lsq_dispatch_valid = '0;
    assign lsq_dispatch_is_load = '0;
    assign lsq_dispatch_is_store = '0;
    assign lsq_dispatch_prd = '{default: 6'b0};
    assign lsq_dispatch_rob_idx = '{default: 7'b0};
    assign lsq_dispatch_pc = '{default: 32'b0};
    assign lsq_mem_issue_valid = 1'b0;
    assign lsq_mem_issue_is_load = 1'b0;
    assign lsq_mem_issue_is_store = 1'b0;
    assign lsq_mem_issue_prd = '0;
    assign lsq_mem_issue_addr = '0;
    assign lsq_mem_issue_data = '0;
    assign lsq_mem_issue_rob_idx = '0;
    assign lsq_mem_result_ack = 1'b0;
    assign lsq_flush = 1'b0;
    assign lsq_flush_rob_idx = '0;

    MemoryDisambiguation #(
        .LQ_ENTRIES(LQ_ENTRIES),
        .SQ_ENTRIES(SQ_ENTRIES),
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .DISPATCH_WIDTH(FETCH_WIDTH),
        .COMMIT_WIDTH(COMMIT_WIDTH)
    ) mem_disambig (
        .clk(clk),
        .rst(rst),
        // Dispatch
        .dispatch_valid(lsq_dispatch_valid),
        .dispatch_is_load(lsq_dispatch_is_load),
        .dispatch_is_store(lsq_dispatch_is_store),
        .dispatch_prd(lsq_dispatch_prd),
        .dispatch_rob_idx(lsq_dispatch_rob_idx),
        .dispatch_pc(lsq_dispatch_pc),
        .dispatch_lsq_valid(lsq_dispatch_lsq_valid),
        .lsq_full(lsq_full_flag),
        // Issue
        .mem_issue_valid(lsq_mem_issue_valid),
        .mem_issue_is_load(lsq_mem_issue_is_load),
        .mem_issue_is_store(lsq_mem_issue_is_store),
        .mem_issue_prd(lsq_mem_issue_prd),
        .mem_issue_addr(lsq_mem_issue_addr),
        .mem_issue_data(lsq_mem_issue_data),
        .mem_issue_rob_idx(lsq_mem_issue_rob_idx),
        // Cache interface (directly to L1DCache would go here)
        .cache_req_valid(lsq_cache_req_valid),
        .cache_req_write(lsq_cache_req_write),
        .cache_req_addr(lsq_cache_req_addr),
        .cache_req_data(lsq_cache_req_data),
        .cache_status(2'b00),  // Tied off for now
        .cache_read_data(32'b0),
        // Result
        .mem_result_valid(lsq_mem_result_valid),
        .mem_result_prd(lsq_mem_result_prd),
        .mem_result_data(lsq_mem_result_data),
        .mem_result_rob_idx(lsq_mem_result_rob_idx),
        .mem_result_ack(lsq_mem_result_ack),
        // Commit
        .commit_valid(rob_commit_valid),
        .commit_is_store(rob_commit_store),
        .commit_rob_idx(rob_commit_rob_idx),
        // ROB interface
        .rob_mem_complete_valid(lsq_rob_mem_complete_valid),
        .rob_mem_complete_rob_idx(lsq_rob_mem_complete_rob_idx),
        .rob_store_addr_valid(lsq_rob_store_addr_valid),
        .rob_store_addr_rob_idx(lsq_rob_store_addr_rob_idx),
        // Recovery
        .memory_violation(lsq_memory_violation),
        .violation_rob_idx(lsq_violation_rob_idx),
        .violation_pc(lsq_violation_pc),
        .flush(lsq_flush),
        .flush_rob_idx(lsq_flush_rob_idx),
        // ROB head for age comparison (handles wraparound)
        .rob_head_idx(rob_head_out),
        // Status
        .lq_count(lsq_lq_count),
        .sq_count(lsq_sq_count)
    );

    // ========== Recovery Unit ==========
    logic recov_flush;
    logic [31:0] recov_flush_pc;
    logic [6:0] recov_flush_rob_idx_out;
    logic [2:0] recov_flush_checkpoint_id;
    logic recov_redirect_valid;
    logic [31:0] recov_redirect_pc;
    logic recov_rat_restore;
    logic [2:0] recov_rat_restore_checkpoint;
    logic recov_freelist_restore;
    logic [2:0] recov_freelist_restore_checkpoint;
    logic recov_rs_flush;
    logic [6:0] recov_rs_flush_rob_idx;
    logic recov_lsq_flush;
    logic [6:0] recov_lsq_flush_rob_idx;
    logic recov_iq_flush;
    logic recov_recovery_in_progress;
    logic [1:0] recov_recovery_type;

    RecoveryUnit #(
        .NUM_PHYS_REGS(NUM_PHYS_REGS),
        .ROB_ENTRIES(ROB_ENTRIES),
        .CHECKPOINT_COUNT(CHECKPOINT_COUNT)
    ) recovery_unit (
        .clk(clk),
        .rst(rst),
        // ROB recovery
        .rob_flush(rob_flush_out),
        .rob_flush_pc(rob_flush_pc),
        .rob_flush_rob_idx(rob_flush_rob_idx),
        .rob_flush_checkpoint_id(rob_flush_checkpoint),
        // Memory violation
        .mem_violation(lsq_memory_violation),
        .mem_violation_rob_idx(lsq_violation_rob_idx),
        .mem_violation_pc(lsq_violation_pc),
        // Branch (unused, handled by ROB)
        .branch_mispredicted(1'b0),
        .branch_correct_pc(32'b0),
        .branch_rob_idx(7'b0),
        .branch_checkpoint_id(3'b0),
        // Outputs
        .flush(recov_flush),
        .flush_pc(recov_flush_pc),
        .flush_rob_idx(recov_flush_rob_idx_out),
        .flush_checkpoint_id(recov_flush_checkpoint_id),
        .redirect_valid(recov_redirect_valid),
        .redirect_pc(recov_redirect_pc),
        .rat_restore(recov_rat_restore),
        .rat_restore_checkpoint(recov_rat_restore_checkpoint),
        .freelist_restore(recov_freelist_restore),
        .freelist_restore_checkpoint(recov_freelist_restore_checkpoint),
        .rs_flush(recov_rs_flush),
        .rs_flush_rob_idx(recov_rs_flush_rob_idx),
        .lsq_flush(recov_lsq_flush),
        .lsq_flush_rob_idx(recov_lsq_flush_rob_idx),
        .iq_flush(recov_iq_flush),
        .recovery_in_progress(recov_recovery_in_progress),
        .recovery_type(recov_recovery_type)
    );

    /* verilator lint_on UNUSEDSIGNAL */

endmodule
