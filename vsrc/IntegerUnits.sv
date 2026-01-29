/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off CASEINCOMPLETE */
`include "Config.svh"

/**
 * Integer Execution Units - ALU, Multiplier, Divider
 *
 * Contains multiple functional units for integer execution:
 * - 2 simple ALUs: 1-cycle latency (add, sub, logic, shift, compare)
 * - 1 multiplier: 4-cycle pipelined latency
 * - 1 divider: 16-cycle non-pipelined latency
 *
 * Each unit produces results on the CDB when complete.
 */
module IntegerUnits #(
    parameter NUM_PHYS_REGS = 64,
    parameter NUM_ALU = 2,
    parameter CDB_WIDTH = 4
) (
    input   logic                           clk,
    input   logic                           rst,

    // ALU inputs from IssueUnit
    input   logic   [NUM_ALU-1:0]           alu_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] alu_prd [NUM_ALU-1:0],
    input   logic   [31:0]                  alu_src1 [NUM_ALU-1:0],
    input   logic   [31:0]                  alu_src2 [NUM_ALU-1:0],
    input   logic   [3:0]                   alu_control [NUM_ALU-1:0],
    input   logic   [1:0]                   alu_src_sel [NUM_ALU-1:0],
    input   logic   [31:0]                  alu_imm [NUM_ALU-1:0],
    input   logic   [4:0]                   alu_shamt [NUM_ALU-1:0],
    input   logic   [NUM_ALU-1:0]           alu_reg_write,
    input   logic   [31:0]                  alu_pc [NUM_ALU-1:0],
    input   logic   [31:0]                  alu_pc_plus_4 [NUM_ALU-1:0],
    input   logic   [6:0]                   alu_rob_idx [NUM_ALU-1:0],

    // ALU availability signals
    output  logic   [NUM_ALU-1:0]           alu_available,

    // Result outputs (to CDB arbiter)
    output  logic   [NUM_ALU-1:0]           alu_result_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] alu_result_prd [NUM_ALU-1:0],
    output  logic   [31:0]                  alu_result_data [NUM_ALU-1:0],
    output  logic   [6:0]                   alu_result_rob_idx [NUM_ALU-1:0],

    // Multiplier inputs
    input   logic                           mul_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] mul_prd,
    input   logic   [31:0]                  mul_src1,
    input   logic   [31:0]                  mul_src2,
    input   logic                           mul_signed_op,  // Signed multiplication
    input   logic   [6:0]                   mul_rob_idx,
    output  logic                           mul_available,
    output  logic                           mul_result_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] mul_result_prd,
    output  logic   [31:0]                  mul_result_data,
    output  logic   [6:0]                   mul_result_rob_idx,

    // Divider inputs
    input   logic                           div_valid,
    input   logic   [$clog2(NUM_PHYS_REGS)-1:0] div_prd,
    input   logic   [31:0]                  div_src1,
    input   logic   [31:0]                  div_src2,
    input   logic                           div_signed_op,  // Signed division
    input   logic                           div_remainder,  // 1 = remainder, 0 = quotient
    input   logic   [6:0]                   div_rob_idx,
    output  logic                           div_available,
    output  logic                           div_result_valid,
    output  logic   [$clog2(NUM_PHYS_REGS)-1:0] div_result_prd,
    output  logic   [31:0]                  div_result_data,
    output  logic   [6:0]                   div_result_rob_idx,

    // Flush interface
    input   logic                           flush
);

    localparam PREG_BITS = $clog2(NUM_PHYS_REGS);

    // ========== Simple ALU Units (1-cycle latency) ==========
    
    // ALU operand selection
    logic [31:0] alu_op1 [NUM_ALU-1:0];
    logic [31:0] alu_op2 [NUM_ALU-1:0];
    logic [31:0] alu_raw_result [NUM_ALU-1:0];

    // Generate ALU units
    generate
        for (genvar a = 0; a < NUM_ALU; a++) begin : alu_gen
            // Operand selection
            always_comb begin
                // alu_src_sel[0]: 1 = shamt, 0 = src1
                // alu_src_sel[1]: 1 = imm, 0 = src2
                alu_op1[a] = alu_src_sel[a][0] ? {27'b0, alu_shamt[a]} : alu_src1[a];
                alu_op2[a] = alu_src_sel[a][1] ? alu_imm[a] : alu_src2[a];
            end

            // ALU computation (combinational)
            always_comb begin
                case (alu_control[a])
                    4'b0000: alu_raw_result[a] = alu_op1[a] + alu_op2[a];                    // ADD
                    4'b0001: alu_raw_result[a] = alu_op1[a] - alu_op2[a];                    // SUB
                    4'b0010: alu_raw_result[a] = alu_op1[a] & alu_op2[a];                    // AND
                    4'b0011: alu_raw_result[a] = alu_op1[a] | alu_op2[a];                    // OR
                    4'b0100: alu_raw_result[a] = ~(alu_op1[a] | alu_op2[a]);                 // NOR
                    4'b0101: alu_raw_result[a] = alu_op1[a] ^ alu_op2[a];                    // XOR
                    4'b0110: alu_raw_result[a] = alu_op2[a] << alu_op1[a][4:0];              // SLL
                    4'b0111: alu_raw_result[a] = $signed(alu_op2[a]) >>> alu_op1[a][4:0];    // SRA
                    4'b1000: alu_raw_result[a] = alu_op2[a] >> alu_op1[a][4:0];              // SRL
                    4'b1001: alu_raw_result[a] = {31'b0, $signed(alu_op1[a]) < $signed(alu_op2[a])}; // SLT
                    4'b1010: alu_raw_result[a] = {31'b0, alu_op1[a] < alu_op2[a]};           // SLTU
                    4'b1011: alu_raw_result[a] = {alu_op2[a][15:0], 16'b0};                  // LUI
                    4'b1100: alu_raw_result[a] = alu_pc_plus_4[a] + 32'd4;                   // JAL return addr (PC+8)
                    4'b1110: alu_raw_result[a] = alu_op1[a];                                 // Pass src1 (JR)
                    4'b1111: alu_raw_result[a] = alu_op2[a];                                 // Pass src2
                    default: alu_raw_result[a] = 32'b0;
                endcase
            end
        end
    endgenerate

    // ALU is always available (1-cycle)
    assign alu_available = {NUM_ALU{1'b1}};

    // ALU pipeline registers (single stage for 1-cycle latency)
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            for (int a = 0; a < NUM_ALU; a++) begin
                alu_result_valid[a] <= 1'b0;
                alu_result_prd[a] <= '0;
                alu_result_data[a] <= '0;
                alu_result_rob_idx[a] <= '0;
            end
        end else begin
            for (int a = 0; a < NUM_ALU; a++) begin
                alu_result_valid[a] <= alu_valid[a] && alu_reg_write[a];
                alu_result_prd[a] <= alu_prd[a];
                alu_result_data[a] <= alu_raw_result[a];
                alu_result_rob_idx[a] <= alu_rob_idx[a];
            end
        end
    end

    // ========== Multiplier (4-cycle pipelined) ==========
    
    localparam MUL_STAGES = 4;
    
    // Pipeline registers for multiplier
    logic mul_pipe_valid [MUL_STAGES-1:0];
    logic [PREG_BITS-1:0] mul_pipe_prd [MUL_STAGES-1:0];
    logic [6:0] mul_pipe_rob_idx [MUL_STAGES-1:0];
    logic [63:0] mul_pipe_result [MUL_STAGES-1:0];

    // Multiplier is available when pipeline isn't full
    // For simplicity, always available (pipelined)
    assign mul_available = 1'b1;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            for (int s = 0; s < MUL_STAGES; s++) begin
                mul_pipe_valid[s] <= 1'b0;
                mul_pipe_prd[s] <= '0;
                mul_pipe_rob_idx[s] <= '0;
                mul_pipe_result[s] <= '0;
            end
            mul_result_valid <= 1'b0;
            mul_result_prd <= '0;
            mul_result_data <= '0;
            mul_result_rob_idx <= '0;
        end else begin
            // Stage 0: Input
            mul_pipe_valid[0] <= mul_valid;
            mul_pipe_prd[0] <= mul_prd;
            mul_pipe_rob_idx[0] <= mul_rob_idx;
            if (mul_signed_op) begin
                mul_pipe_result[0] <= $signed(mul_src1) * $signed(mul_src2);
            end else begin
                mul_pipe_result[0] <= mul_src1 * mul_src2;
            end

            // Pipeline stages 1 to MUL_STAGES-1
            for (int s = 1; s < MUL_STAGES; s++) begin
                mul_pipe_valid[s] <= mul_pipe_valid[s-1];
                mul_pipe_prd[s] <= mul_pipe_prd[s-1];
                mul_pipe_rob_idx[s] <= mul_pipe_rob_idx[s-1];
                mul_pipe_result[s] <= mul_pipe_result[s-1];
            end

            // Output from final stage
            mul_result_valid <= mul_pipe_valid[MUL_STAGES-1];
            mul_result_prd <= mul_pipe_prd[MUL_STAGES-1];
            mul_result_data <= mul_pipe_result[MUL_STAGES-1][31:0]; // Low 32 bits
            mul_result_rob_idx <= mul_pipe_rob_idx[MUL_STAGES-1];
        end
    end

    // ========== Divider (16-cycle non-pipelined) ==========
    
    localparam DIV_LATENCY = 16;
    
    // Divider state
    typedef enum logic [1:0] {
        DIV_IDLE,
        DIV_BUSY,
        DIV_DONE
    } div_state_t;
    
    div_state_t div_state;
    logic [4:0] div_counter;
    logic [PREG_BITS-1:0] div_saved_prd;
    logic [6:0] div_saved_rob_idx;
    logic [31:0] div_saved_src1, div_saved_src2;
    logic div_saved_signed, div_saved_remainder;
    logic [31:0] div_quotient, div_remainder_val;

    assign div_available = (div_state == DIV_IDLE);

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            div_state <= DIV_IDLE;
            div_counter <= '0;
            div_result_valid <= 1'b0;
            div_result_prd <= '0;
            div_result_data <= '0;
            div_result_rob_idx <= '0;
            div_saved_prd <= '0;
            div_saved_rob_idx <= '0;
            div_saved_src1 <= '0;
            div_saved_src2 <= '0;
            div_saved_signed <= 1'b0;
            div_saved_remainder <= 1'b0;
        end else begin
            div_result_valid <= 1'b0;
            
            case (div_state)
                DIV_IDLE: begin
                    if (div_valid && div_src2 != 32'b0) begin
                        div_state <= DIV_BUSY;
                        div_counter <= DIV_LATENCY - 1;
                        div_saved_prd <= div_prd;
                        div_saved_rob_idx <= div_rob_idx;
                        div_saved_src1 <= div_src1;
                        div_saved_src2 <= div_src2;
                        div_saved_signed <= div_signed_op;
                        div_saved_remainder <= div_remainder;
                    end
                end
                
                DIV_BUSY: begin
                    if (div_counter == 0) begin
                        div_state <= DIV_DONE;
                        // Compute division result
                        if (div_saved_signed) begin
                            div_quotient <= $signed(div_saved_src1) / $signed(div_saved_src2);
                            div_remainder_val <= $signed(div_saved_src1) % $signed(div_saved_src2);
                        end else begin
                            div_quotient <= div_saved_src1 / div_saved_src2;
                            div_remainder_val <= div_saved_src1 % div_saved_src2;
                        end
                    end else begin
                        div_counter <= div_counter - 1;
                    end
                end
                
                DIV_DONE: begin
                    div_state <= DIV_IDLE;
                    div_result_valid <= 1'b1;
                    div_result_prd <= div_saved_prd;
                    div_result_rob_idx <= div_saved_rob_idx;
                    div_result_data <= div_saved_remainder ? div_remainder_val : div_quotient;
                end
            endcase
        end
    end

endmodule
