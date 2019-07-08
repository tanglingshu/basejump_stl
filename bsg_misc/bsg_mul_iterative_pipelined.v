// -------------------------------------------------------
// bsg_mul_iterative_pipelined.v
// sqlin16@fudan.edu.cn 07/06/2019
// -------------------------------------------------------
// This is a pipelined multiplier with configurable stride for each level, which in essential is a unrolling of bsg_mul_iterative.

// -------------------------------------------------------

module bsg_mul_iterative_pipelined #(
  parameter integer width_p = "inv"
  ,parameter integer iter_step_p = "inv"
  ,parameter bit debug_p = 1
)(
  input clk_i
  ,input reset_i

  ,input [width_p-1:0] opA_i
  ,input [width_p-1:0] opB_i
  ,input signed_i
  ,input v_i

  ,output [2*width_p-1:0] res_o
  ,output v_o
);

localparam integer level_lp = width_p / iter_step_p;

reg [level_lp-1:0][width_p-1:0] opA_r;
reg [level_lp-1:0][width_p-1:0] opB_r;

reg opA_sign_r;
reg opB_sign_r;

reg [level_lp+1:0] v_i_r;


// Update entrance operands register
always_ff @(posedge clk_i) begin
  if(reset_i) begin
    opA_r[0] <= '0;
    opB_r[0] <= '0;
    opA_sign_r <= '0;
    opB_sign_r <= '0;
    v_i_r[0] <= '0;
  end
  else begin
    if(v_i) begin
      opA_r[0] <= opA_i;
      opB_r[0] <= opB_i;
      opA_sign_r <= signed_i & opA_i[width_p-1];
      opB_sign_r <= signed_i & opB_i[width_p-1];
      v_i_r[0] <= v_i;
    end
    else begin
      opA_r[0] <= '0;
      opB_r[0] <= '0;
      opA_sign_r <= '0;
      opB_sign_r <= '0;
      v_i_r[0] <= '0;
    end
  end
end

// Update other level operands register
for(genvar i = 1; i < level_lp; ++i) begin: OPERANDS_REGISTER
  always_ff @(posedge clk_i) begin
    if(reset_i) begin
      opA_r[i] <= '0;
      opB_r[i] <= '0;
    end
    else begin
      opA_r[i] <= opA_r[i-1];
      opB_r[i] <= opB_r[i-1] >> iter_step_p;
    end
  end
end

for(genvar i = 1; i <= level_lp+1; ++i) begin: V_I_R
  always_ff @(posedge clk_i) begin
    if(reset_i) v_i_r[i] <= '0;
    else v_i_r[i] <= v_i_r[i-1];
  end
end

wire [width_p-1:0] sign_modification_n = (opA_r[0] & {width_p{opB_sign_r}}) + (opB_r[0] & {width_p{opA_sign_r}});

reg [level_lp-1:0][width_p-1:0] sign_modification_r;

for(genvar i = 0; i < level_lp; ++i) begin: SIGN_FIXING_REGISTER
  always_ff @(posedge clk_i) begin
    if(reset_i) sign_modification_r[i] <= '0;
    else if(i == 0) sign_modification_r[i] <= sign_modification_n;
    else sign_modification_r[i] <= sign_modification_r[i-1];
  end
end

reg [level_lp:0][width_p-1:0] csa_res_r;
reg [level_lp-1:0][width_p-1:0] csa_car_r;
reg [level_lp:0][width_p-1:0] lower_bits_r;

for(genvar i = 0; i < level_lp; ++i) begin: CSA
  wire [iter_step_p:0][width_p-1:0] cascade_csa_res;
  wire [iter_step_p:0][width_p-1:0] cascade_csa_car;

  if(i == 0) begin
    assign cascade_csa_res[0] = (width_p)'(0);
    assign cascade_csa_car[0] = (width_p)'(0);
  end
  else begin
    assign cascade_csa_res[0] = csa_res_r[i-1];
    assign cascade_csa_car[0] = csa_car_r[i-1];
  end

  wire [iter_step_p-1:0] lower_bits_resolved;
  wire [width_p-1:0] lower_bits_n;

  for(genvar j = 0; j < iter_step_p; ++j) begin: CSA_FLEX
    bsg_adder_carry_save #(
      .width_p(width_p)
    ) csa_adder (
      .opA_i({1'b0, cascade_csa_res[j][width_p-1:1]})
      ,.opB_i(cascade_csa_car[j])
      ,.opC_i(opA_r[i] & {width_p{opB_r[i][j]}})
      ,.res_o(cascade_csa_res[j+1])
      ,.car_o(cascade_csa_car[j+1])
    );
    assign lower_bits_resolved[j] = cascade_csa_res[j+1][0];
  end

  if(iter_step_p == width_p) 
    assign lower_bits_n = lower_bits_resolved;
  else if(i == 0)
    assign lower_bits_n = {lower_bits_resolved, (width_p-iter_step_p)'(0)};
  else 
    assign lower_bits_n = {lower_bits_resolved, lower_bits_r[i-1][width_p-1:iter_step_p]};

  wire [width_p-1:0] csa_res_n;
  wire [width_p-1:0] csa_car_n;

  always_ff @(posedge clk_i) begin
    if(debug_p) $display("lower_bits_n:%b",lower_bits_n);
  end

  if(i == level_lp-1) begin
    bsg_adder_carry_save #(
      .width_p(width_p)
    ) csa_sign_fixer (
      .opA_i({1'b0, cascade_csa_res[iter_step_p][width_p-1:1]})
      ,.opB_i(cascade_csa_car[iter_step_p])
      ,.opC_i(~sign_modification_r[i])
      ,.res_o(csa_res_n)
      ,.car_o(csa_car_n)
    );
  end
  else begin
    assign csa_res_n = cascade_csa_res[iter_step_p];
    assign csa_car_n = cascade_csa_car[iter_step_p];
  end

  always_ff @(posedge clk_i) begin
    if(reset_i) begin
      csa_res_r[i] <= '0;
      csa_car_r[i] <= '0;
      lower_bits_r[i] <= '0;
    end
    else begin
      csa_res_r[i] <= csa_res_n;
      csa_car_r[i] <= csa_car_n;
      lower_bits_r[i] <= lower_bits_n;
    end
  end
end

wire [width_p-1:0] high_result = {csa_car_r[level_lp-1], 1'b1} + csa_res_r[level_lp-1];

always_ff @(posedge clk_i) begin
  if(reset_i) begin
    lower_bits_r[level_lp] <= '0;
    csa_res_r[level_lp] <= '0;
    v_i_r[level_lp+1] <= '0;
  end
  else begin
    csa_res_r[level_lp] <= high_result;
    lower_bits_r[level_lp] <= lower_bits_r[level_lp-1];
    v_i_r[level_lp+1] <= v_i_r[level_lp];
  end
end

assign res_o = {csa_res_r[level_lp], lower_bits_r[level_lp]};
assign v_o = v_i_r[level_lp+1];

if(debug_p) begin
  always_ff @(posedge clk_i) begin
    $display("==========================================================");
    $display("============ First Level ============");
    $display("opA_sign_r:%b", opA_sign_r);
    $display("opB_sign_r:%b", opB_sign_r);
    $display("v_i_r:%b", v_i_r[0]);
    $display("opA_r:%b", opA_r[0]);
    $display("opB_r:%b", opB_r[0]);
    for(logic [31:0] i = 0; i < level_lp; ++i) begin
      $display("============Level %d===========", i);
      $display("csa_res_r:%b", csa_res_r[i]);
      $display("csa_car_r:%b", csa_car_r[i]);
      $display("lower_bits_r:%b", lower_bits_r[i]);
      $display("sign_modification_r:%b", sign_modification_r[i]);
      $display("v_i_r:%b", v_i_r[i+1]);
      if(i > 0) begin
        $display("opA_r:%b", opA_r[i-1]);
        $display("opB_r:%b", opB_r[i-1]);
      end
    end
    $display("============ Last Level ============");
    $display("csa_res_r:%b",csa_res_r[level_lp]);
    $display("lower_bits_r:%b", lower_bits_r[level_lp]);
    $display("v_i_r:%b", v_i_r[level_lp+1]);
  end
end

endmodule

