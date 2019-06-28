//This file is automatically generated by python script ./bsg_adder_wallace_tree_generator.py with N = 8. Do not modify.
module bsg_adder_wallace_tree_8 #(
  parameter integer width_p = "inv"
)(
  input [7:0][width_p-1:0] ops_i
  ,output [2+width_p:0] resA_o
  ,output [2+width_p:0] resB_o
);
wire [width_p-1:0] csa_0_0_res_o;
wire [width_p-1:0] csa_0_0_car_o;
wire [width_p-1:0] csa_0_1_res_o;
wire [width_p-1:0] csa_0_1_car_o;
wire [width_p+0:0] csa_1_0_res_o;
wire [width_p+0:0] csa_1_0_car_o;
wire [width_p+0:0] csa_1_1_res_o;
wire [width_p+0:0] csa_1_1_car_o;
wire [width_p+1:0] csa_2_0_res_o;
wire [width_p+1:0] csa_2_0_car_o;
wire [width_p+2:0] csa_3_0_res_o;
wire [width_p+2:0] csa_3_0_car_o;

bsg_adder_carry_save#(
  .width_p(width_p+0)
) csa_0_0 (
  .opA_i(ops_i[0])
  ,.opB_i(ops_i[1])
  ,.opC_i(ops_i[2])
  ,.res_o(csa_0_0_res_o)
  ,.car_o(csa_0_0_car_o)
);

wire [width_p+0:0] csa_internal_wire_0 = {1'b0, csa_0_0_res_o};
wire [width_p+0:0] csa_internal_wire_1 = {csa_0_0_car_o,1'b0};
bsg_adder_carry_save#(
  .width_p(width_p+0)
) csa_0_1 (
  .opA_i(ops_i[3])
  ,.opB_i(ops_i[4])
  ,.opC_i(ops_i[5])
  ,.res_o(csa_0_1_res_o)
  ,.car_o(csa_0_1_car_o)
);

wire [width_p+0:0] csa_internal_wire_2 = {1'b0, csa_0_1_res_o};
wire [width_p+0:0] csa_internal_wire_3 = {csa_0_1_car_o,1'b0};
bsg_adder_carry_save#(
  .width_p(width_p+1)
) csa_1_0 (
  .opA_i({1'b0, ops_i[6]})
  ,.opB_i({1'b0, ops_i[7]})
  ,.opC_i(csa_internal_wire_0)
  ,.res_o(csa_1_0_res_o)
  ,.car_o(csa_1_0_car_o)
);

wire [width_p+1:0] csa_internal_wire_4 = {1'b0, csa_1_0_res_o};
wire [width_p+1:0] csa_internal_wire_5 = {csa_1_0_car_o,1'b0};
bsg_adder_carry_save#(
  .width_p(width_p+1)
) csa_1_1 (
  .opA_i(csa_internal_wire_1)
  ,.opB_i(csa_internal_wire_2)
  ,.opC_i(csa_internal_wire_3)
  ,.res_o(csa_1_1_res_o)
  ,.car_o(csa_1_1_car_o)
);

wire [width_p+1:0] csa_internal_wire_6 = {1'b0, csa_1_1_res_o};
wire [width_p+1:0] csa_internal_wire_7 = {csa_1_1_car_o,1'b0};
bsg_adder_carry_save#(
  .width_p(width_p+2)
) csa_2_0 (
  .opA_i(csa_internal_wire_4)
  ,.opB_i(csa_internal_wire_5)
  ,.opC_i(csa_internal_wire_6)
  ,.res_o(csa_2_0_res_o)
  ,.car_o(csa_2_0_car_o)
);

wire [width_p+2:0] csa_internal_wire_8 = {1'b0, csa_2_0_res_o};
wire [width_p+2:0] csa_internal_wire_9 = {csa_2_0_car_o,1'b0};
bsg_adder_carry_save#(
  .width_p(width_p+3)
) csa_3_0 (
  .opA_i({1'b0, csa_internal_wire_7})
  ,.opB_i(csa_internal_wire_8)
  ,.opC_i(csa_internal_wire_9)
  ,.res_o(csa_3_0_res_o)
  ,.car_o(csa_3_0_car_o)
);

wire [width_p+3:0] csa_internal_wire_10 = {1'b0, csa_3_0_res_o};
wire [width_p+3:0] csa_internal_wire_11 = {csa_3_0_car_o,1'b0};

assign resA_o = csa_internal_wire_10;
assign resB_o = csa_internal_wire_11;
endmodule

