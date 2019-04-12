module bsg_reduce_segmented #(parameter segments_p = "inv"
                              ,parameter segment_width_p = "inv"

                  , parameter xor_p = 0
                  , parameter and_p = 0
                  , parameter or_p = 0                         
                  )
  (input    [segments_p*segment_width_p-1:0] i
   , output [segments_p-1:0] o
    );

   // synopsys translate_off
   initial
      assert( $countones({xor_p & 1'b1, and_p & 1'b1, or_p & 1'b1}) == 1)
        else $error("%m: exactly one function may be selected\n");

  // synopsys translate_on

  
  genvar j;
  
  for (j = 0; j < segments_p; j=j+1)
  begin: rof2
  if (xor_p)
    assign o[j] = ^i[(j+1)*segment_width_p:j*segment_width_p];
   else if (and_p)
     assign o[j] = &i[(j+1)*segment_width_p:j*segment_width_p];
   else if (or_p)
     assign o[j] = |i[(j+1)*segment_width_p:j*segment_width_p];
  end
    
endmodule
