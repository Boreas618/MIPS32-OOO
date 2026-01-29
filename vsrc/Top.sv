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

    /* verilator lint_on UNUSEDSIGNAL */

endmodule
