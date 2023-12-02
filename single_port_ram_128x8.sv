module single_port_ram_128x8(
	input [7:0] data,
	input [6:0] addr,
	input ram_en,
	input clk,
	output [7:0] q
);

logic [7:0] ram[127:0];

always_ff@(posedge clk)
begin
	if(ram_en)
		ram[addr] <= data;
end

assign q = ram[addr];
endmodule
