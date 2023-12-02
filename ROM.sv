module ROM(
	output [13:0] Rom_data_out, 
	input [10:0] Rom_addr_in
);
//---------
    
    logic [13:0] data;
    always_comb
        begin
            case (Rom_addr_in)
				10'h0:	data = 14'h01A5;		
				10'h1:	data = 14'h0103;		
				10'h2:	data = 14'h3007;		
				10'h3:	data = 14'h07A5;
				10'h4:	data = 14'h3005;				
				10'h5:	data = 14'h0AA5;
				10'h6:	data = 14'h04A5;
				10'h7:	data = 14'h00A4;
				10'h8:	data = 14'h0225;
				10'h9:	data = 14'h0825;
				10'ha:	data = 14'h06A4;		
				
				10'hb:	data = 14'h3400;
				10'hc:	data = 14'h3400;		
				default:data = 14'h0;
            endcase
        end
     assign Rom_data_out = data;

endmodule