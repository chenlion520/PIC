module Stack(
	output logic [10:0] stack_out,
	input [10:0] stack_in,
	input	push,
	input pop,
	input	reset,
	input clk
);

logic [3:0] stk_ptr;
logic [10:0] stack [15:0];
logic [3:0] stk_index;

assign stk_index = stk_ptr + 1;
assign stack_out = stack[stk_ptr];

always_ff @(posedge clk)
begin
	if(reset)
		stk_ptr <= 4'b1111;
	else if(push)
	begin
		stack[stk_index] <= stack_in;
		stk_ptr <= stk_ptr + 1;
	end
	else if(pop)
		stk_ptr <= stk_ptr - 1;
end
endmodule
