module Program_Rom(
	output logic [13:0] Rom_data_out, 
	input [10:0] Rom_addr_in
);

    logic [13:0] data;
    always_comb
        begin
            case (Rom_addr_in)
                10'h0 : data = 14'h01A5;
                10'h1 : data = 14'h01A4;
                10'h2 : data = 14'h01A3;
                10'h3 : data = 14'h3024;
                10'h4 : data = 14'h00A5;
                10'h5 : data = 14'h3015;
                10'h6 : data = 14'h00A4;
                10'h7 : data = 14'h0825;
                10'h8 : data = 14'h00A3;
                10'h9 : data = 14'h0824;
                10'ha : data = 14'h02A5;
                10'hb : data = 14'h1FA5;
                10'hc : data = 14'h2807;
                10'hd : data = 14'h280E;
                10'he : data = 14'h0824;
                10'hf : data = 14'h00A5;
                10'h10 : data = 14'h0823;
                10'h11 : data = 14'h00A4;
                10'h12 : data = 14'h03A3;
                10'h13 : data = 14'h1FA3;
                10'h14 : data = 14'h2809;
                10'h15 : data = 14'h0825;
                10'h16 : data = 14'h008D;
                10'h17 : data = 14'h3400;
                10'h18 : data = 14'h3400;
                default: data = 14'h0;   
            endcase
        end

     assign Rom_data_out = data;

endmodule
