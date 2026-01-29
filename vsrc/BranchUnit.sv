`include "Config.svh"
`include "BranchTypes.svh"

/*
 * Branch Unit - Resolution and Recovery Logic
 * 
 * Coordinates branch prediction components and handles:
 * - Prediction generation (combining BTB, direction predictor, RAS)
 * - Misprediction detection  
 * - Pipeline flush and recovery on misprediction
 * - Predictor updates
 */
module BranchUnit #(
    parameter GHR_BITS = 10
)(
    input   logic           clk,
    input   logic           rst,
    
    // IF stage interface - Prediction
    input   logic   [31:0]  if_pc,
    output  logic   [31:0]  predicted_pc,     // Next PC prediction
    output  logic           prediction_valid, // Prediction is valid
    
    // EX/MEM stage interface - Resolution
    input   logic           branch_valid,     // Branch instruction in EX/MEM
    input   logic   [31:0]  branch_pc,        // PC of branch instruction
    input   logic   [31:0]  actual_target,    // Actual computed target
    input   logic           actual_taken,     // Branch was actually taken
    input   logic   [3:0]   branch_type,      // Branch type from decode
    input   logic   [31:0]  predicted_target_ex, // What we predicted
    input   logic           predicted_taken_ex,  // Whether we predicted taken
    input   logic   [GHR_BITS-1:0] saved_ghr, // GHR at prediction time
    
    // Recovery signals
    output  logic           mispredicted,     // Misprediction detected
    output  logic   [31:0]  correct_pc,       // Correct PC to fetch
    output  logic           flush_pipeline,   // Flush IF/ID stages
    
    // GHR output for pipeline tracking
    output  logic   [GHR_BITS-1:0] current_ghr,
    
    // RAS checkpoint output for saving with prediction
    output  logic   [8:0]   ras_checkpoint_out,
    
    // RAS recovery input (from pipeline on misprediction)
    input   logic           ras_recover,
    input   logic   [8:0]   ras_recover_checkpoint,
    
    // Statistics (optional)
    output  logic   [31:0]  total_branches,
    output  logic   [31:0]  mispredictions
);

    // BTB type encoding - using centralized constants from BranchTypes.svh
    localparam BTB_COND   = `BTB_TYPE_COND;    // Conditional branch
    localparam BTB_JUMP   = `BTB_TYPE_JUMP;    // Unconditional jump (J)
    localparam BTB_CALL   = `BTB_TYPE_CALL;    // Call (JAL)
    localparam BTB_RETURN = `BTB_TYPE_RETURN;  // Return (JR $ra)
    
    // Internal signals
    logic btb_hit;
    logic [31:0] btb_target;
    logic [1:0] btb_type;
    logic direction_pred;
    logic [31:0] ras_target;
    logic ras_valid;
    
    // BTB update signals
    logic btb_update;
    logic [31:0] btb_update_pc;
    logic [31:0] btb_update_target;
    logic [1:0] btb_update_type;
    
    // RAS signals
    logic ras_push;
    logic ras_pop;
    logic [31:0] ras_push_addr;
    
    // Direction predictor update
    logic dir_update;
    logic dir_update_taken;
    
    // BTB instance
    BTB #(
        .NUM_ENTRIES(512)
    ) btb_inst (
        .clk(clk),
        .rst(rst),
        .lookup_pc(if_pc),
        .hit(btb_hit),
        .predicted_target(btb_target),
        .branch_type(btb_type),
        .update_valid(btb_update),
        .update_pc(btb_update_pc),
        .update_target(btb_update_target),
        .update_type(btb_update_type)
    );
    
    // Direction predictor instance
    BranchPredictor #(
        .GHR_BITS(GHR_BITS),
        .PHT_SIZE(1024)
    ) dir_pred_inst (
        .clk(clk),
        .rst(rst),
        .lookup_pc(if_pc),
        .predicted_taken(direction_pred),
        .update_valid(dir_update),
        .update_pc(branch_pc),
        .update_taken(dir_update_taken),
        .update_ghr(saved_ghr),
        .current_ghr_out(current_ghr)
    );
    
    // RAS instance
    // Checkpoint width: 2*PTR_BITS + 1 = 2*4 + 1 = 9 bits for DEPTH=16
    logic [8:0] ras_checkpoint;
    
    RAS #(
        .DEPTH(16)
    ) ras_inst (
        .clk(clk),
        .rst(rst),
        .pop(ras_pop),
        .predicted_target(ras_target),
        .valid(ras_valid),
        .push(ras_push),
        .push_addr(ras_push_addr),
        .recover(ras_recover),
        .recover_ptr(ras_recover_checkpoint),
        .checkpoint(ras_checkpoint)
    );
    
    // Export checkpoint for pipeline to save with prediction
    assign ras_checkpoint_out = ras_checkpoint;
    
    // Prediction logic
    // Determine predicted next PC based on BTB hit and type
    always_comb begin
        predicted_pc = if_pc + 32'd4;  // Default: sequential
        prediction_valid = 1'b0;
        ras_pop = 1'b0;
        
        if (btb_hit) begin
            case (btb_type)
                BTB_COND: begin
                    // Conditional: use direction predictor
                    if (direction_pred) begin
                        predicted_pc = btb_target;
                        prediction_valid = 1'b1;
                    end else begin
                        predicted_pc = if_pc + 32'd4;
                        prediction_valid = 1'b1;
                    end
                end
                BTB_JUMP: begin
                    // Unconditional jump: always taken
                    predicted_pc = btb_target;
                    prediction_valid = 1'b1;
                end
                BTB_CALL: begin
                    // Call: always taken, push return address
                    predicted_pc = btb_target;
                    prediction_valid = 1'b1;
                end
                BTB_RETURN: begin
                    // Return: use RAS if valid
                    if (ras_valid) begin
                        predicted_pc = ras_target;
                        prediction_valid = 1'b1;
                        ras_pop = 1'b1;
                    end
                end
                default: begin
                    predicted_pc = if_pc + 32'd4;
                end
            endcase
        end
    end
    
    // RAS push logic (on call instructions)
    always_comb begin
        ras_push = 1'b0;
        ras_push_addr = 32'b0;
        
        // Push when we predict a call (BTB hit with call type)
        if (btb_hit && btb_type == BTB_CALL) begin
            ras_push = 1'b1;
            ras_push_addr = if_pc + 32'd8;  // Return address (after delay slot)
        end
    end
    
    // Misprediction detection and recovery
    always_comb begin
        mispredicted = 1'b0;
        correct_pc = 32'b0;
        flush_pipeline = 1'b0;
        
        if (branch_valid) begin
            // Check if prediction was correct
            if (actual_taken) begin
                // Branch taken: check if we predicted taken to correct target
                if (!predicted_taken_ex || (predicted_target_ex != actual_target)) begin
                    mispredicted = 1'b1;
                    correct_pc = actual_target;
                    flush_pipeline = 1'b1;
                end
            end else begin
                // Branch not taken: check if we predicted not taken
                if (predicted_taken_ex) begin
                    mispredicted = 1'b1;
                    correct_pc = branch_pc + 32'd4;  // Fall through
                    flush_pipeline = 1'b1;
                end
            end
        end
    end
    
    // BTB and direction predictor updates
    always_comb begin
        btb_update = 1'b0;
        btb_update_pc = branch_pc;
        btb_update_target = actual_target;
        btb_update_type = BTB_COND;
        dir_update = 1'b0;
        dir_update_taken = 1'b0;
        
        if (branch_valid) begin
            // Determine BTB type from branch_type using centralized constants
            case (branch_type)
                `BRANCH_TYPE_J:    btb_update_type = BTB_JUMP;
                `BRANCH_TYPE_JAL:  btb_update_type = BTB_CALL;
                `BRANCH_TYPE_JR:   btb_update_type = BTB_RETURN;
                `BRANCH_TYPE_BEQ, `BRANCH_TYPE_BNE, 
                `BRANCH_TYPE_BGEZ, `BRANCH_TYPE_BLTZ: btb_update_type = BTB_COND;
                default: btb_update_type = BTB_COND;
            endcase
            
            // Update BTB on taken branches (or always for jumps)
            if (actual_taken || branch_type == `BRANCH_TYPE_J || branch_type == `BRANCH_TYPE_JAL) begin
                btb_update = 1'b1;
            end
            
            // Update direction predictor on conditional branches
            if (branch_type == `BRANCH_TYPE_BEQ || branch_type == `BRANCH_TYPE_BNE || 
                branch_type == `BRANCH_TYPE_BGEZ || branch_type == `BRANCH_TYPE_BLTZ) begin
                dir_update = 1'b1;
                dir_update_taken = actual_taken;
            end
        end
    end
    
    // Statistics counters
    always_ff @(posedge clk) begin
        if (rst) begin
            total_branches <= 32'b0;
            mispredictions <= 32'b0;
        end else if (branch_valid) begin
            total_branches <= total_branches + 1;
            if (mispredicted) begin
                mispredictions <= mispredictions + 1;
            end
        end
    end

endmodule
