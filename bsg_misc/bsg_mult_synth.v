module bsg_mult_synth #(parameter width_p=8)
	(input signed [width_p-1:0] in_a
	,input signed [width_p-1:0] in_b
	,output signed [2*width_p-1:0] out
	);

	assign out = in_a * in_b;

endmodule
