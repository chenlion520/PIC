module cpu(
	input clk,
	input rst,
	output logic [7:0]port_b_out
	);
logic [10:0]pc_next,pc_q,mar_q,stack_q,pc_n,w_change,k_change;
logic [13:0]ir,rom_out;
logic [3:0]sel_pc;
logic load_pc,load_mar,load_ir,rst_ir,load_w,sel_alu,ram_en,d,sel_bus,load_port_b;
logic pop,push;
logic btfsc_skip_bit,btfss_skip_bit,btfsc_btfss_skip_bit;
logic [1:0]sel_bit,sel_ram_mux;
logic [7:0]alu_q,mux1_out,ram_out,data_bus,bcf_mux,bsf_mux,ram_mux_out,w;
logic [4:0]ps,ns;
logic [4:0]alu_op;
logic movelw,addlw,sublw,iorlw,andlw,xorlw;
//----pc
assign pc_n = pc_q + 1;
assign w_change = {3'b0,w} - 1;
assign k_change = {ir[8],ir[8],ir[8:0]} - 1;
always_comb
begin
	case(sel_pc)
		0:	pc_next = pc_n;
		1:	pc_next = ir[10:0];
		2:	pc_next = stack_q;//new
		3:	pc_next = pc_q + k_change;//new
		4:	pc_next = pc_q + w_change;//new
	endcase
end
always_ff @(posedge clk)
begin
	if(rst) pc_q <= #1 0;
	else if(load_pc) pc_q <= #1 pc_next; 
end
//----stack
Stack s1( stack_q, pc_q, push, pop, rst, clk);//new
//----mar
always_ff @(posedge clk)
begin
	
	if(rst) mar_q <= 0;
	else if(load_mar) mar_q <= pc_q;
end
//----rom
Program_Rom ROM(
	.Rom_data_out(rom_out),
	.Rom_addr_in (mar_q)
);
//----ram
single_port_ram_128x8 sram(data_bus, ir[6:0], ram_en, clk, ram_out);
//----ir

always_ff @(posedge clk)
begin
	if(rst_ir) ir <= #1 0;
	else if(load_ir) ir <= #1 rom_out;
end
//----ALU
assign alu_zero = (alu_q == 0) ? 1'b1: 1'b0; //choose the alu zero 
always_comb
begin
	case(sel_alu)
		0:	mux1_out = ir[7:0];
		1:	mux1_out = ram_mux_out;//add the new variable to send to mux1_out
	endcase
end
assign data_bus = sel_bus ? w : alu_q;

always_comb 
begin
	case(alu_op)
		4'h0:	alu_q = mux1_out + w;
		4'h1:	alu_q = mux1_out - w;
		4'h2:	alu_q = mux1_out & w;
		4'h3:	alu_q = mux1_out | w;
		4'h4:	alu_q = mux1_out ^ w;
		4'h5:	alu_q = mux1_out;
		4'h6:	alu_q = mux1_out + 1;
		4'h7: 	alu_q = mux1_out - 1;
		4'h8:	alu_q = 0;
		4'h9:	alu_q = ~mux1_out;
		4'hA:	alu_q = {mux1_out[7],mux1_out[7:1]};
		4'hB:	alu_q = {mux1_out[6:0],1'b0};
		4'hC:	alu_q = {1'b0,mux1_out[7:1]};
		4'hD:	alu_q = {mux1_out[6:0],mux1_out[7]};
		4'hE:	alu_q = {mux1_out[0],mux1_out[7:1]};
		4'hF:	alu_q = {mux1_out[3:0],mux1_out[7:4]};
		default alu_q = mux1_out + w;
	endcase
end

always_ff @(posedge clk)
begin
	if(rst) w <=  0;
	else if(load_w) w <=  alu_q;
end
//----bsf抬f

assign sel_bit = ir[9:7];
always_comb//case of bcf (and)
begin
	case(sel_bit)
		3'b000:bcf_mux = ram_out & 8'b11111110;
		3'b001:bcf_mux = ram_out & 8'b11111101;
		3'b010:bcf_mux = ram_out & 8'b11111011;
		3'b011:bcf_mux = ram_out & 8'b11110111;
		3'b100:bcf_mux = ram_out & 8'b11101111;
		3'b101:bcf_mux = ram_out & 8'b11011111;
		3'b110:bcf_mux = ram_out & 8'b10111111;
		3'b111:bcf_mux = ram_out & 8'b01111111;
	endcase
end
always_comb//case of bsf(or)
begin
	case(sel_bit)
		3'b000:bsf_mux = ram_out | 8'b00000001;
		3'b001:bsf_mux = ram_out | 8'b00000010;
		3'b010:bsf_mux = ram_out | 8'b00000100;
		3'b011:bsf_mux = ram_out | 8'b00001000;
		3'b100:bsf_mux = ram_out | 8'b00010000;
		3'b101:bsf_mux = ram_out | 8'b00100000;
		3'b110:bsf_mux = ram_out | 8'b01000000;
		3'b111:bsf_mux = ram_out | 8'b10000000;
	endcase
end

//----ram_mux
always_comb
begin
	case(sel_ram_mux)//three case 1.for ram_out 2.for bcf_mux(from line 97) 3.for bsf_mux(from line 110)
		0:ram_mux_out = ram_out;
		1:ram_mux_out = bcf_mux;
		2:ram_mux_out = bsf_mux;
	endcase
end
//----port_b
assign addr_port_b = (ir[6:0] == 7'h0d);
always_ff @(posedge clk)
begin
	if(rst) port_b_out <= 0;
	else if(load_port_b) port_b_out <= data_bus;
end
//----controller
assign movelw	=	(ir[13:8] == 6'h30);
assign addlw 	=	(ir[13:8] == 6'h3E);
assign andlw 	=	(ir[13:8] == 6'h39);
assign iorlw 	=	(ir[13:8] == 6'h38);
assign sublw 	=	(ir[13:8] == 6'h3C);
assign xorlw 	=	(ir[13:8] == 6'h3A);
assign addwf 	=	(ir[13:8] == 6'h07);
assign goto 	=	(ir[13:11] == 3'b101);
assign clrf 	=	(ir[13:7] == 7'h03);
assign clrw 	=	(ir[13:2] == 12'h040);
assign andwf 	= 	(ir[13:8] == 6'h05);
assign comf 	= 	(ir[13:8] == 6'h09);
assign decf 	= 	(ir[13:8] == 6'h03);
assign incf		=	(ir[13:8] == 6'h0A);
assign iorwf	=	(ir[13:8] == 6'h04);
assign movf		=	(ir[13:8] == 6'h08);
assign movwf	=	(ir[13:7] == 7'h01);
assign subwf	=	(ir[13:8] == 6'h02);
assign xorwf	=	(ir[13:8] == 6'h06);
assign bcf		= 	(ir[13:10] == 4'h4);
assign bsf		= 	(ir[13:10] == 4'h5);
assign btfsc	= 	(ir[13:10] == 4'h6);
assign btfss	= 	(ir[13:10] == 4'h7);
assign decfsz	=	(ir[13:8] == 6'h0b);
assign incfsz	=	(ir[13:8] == 6'h0f);
assign asrf		=	(ir[13:8] == 6'h37);
assign lslf		=	(ir[13:8] == 6'h35);
assign lsrf		=	(ir[13:8] == 6'h36);
assign rlf		=	(ir[13:8] == 6'h0d);
assign rrf		=	(ir[13:8] == 6'h0c);
assign swapf	=	(ir[13:8] == 6'h0e);
assign call		=	(ir[13:11] == 3'b1_00);//new
assign RETURN	=	(ir[13:0] == 14'b00_0000_0000_1000);//new
assign bra		= 	(ir[13:9] == 5'b11001);//new
assign brw		=	(ir[13:0] == 14'b00000000001011);//new
assign nop		=	(ir[13:0] == 14'b00000000000000);//new

parameter T0 = 0;
parameter T1 = 1;
parameter T2 = 2;
parameter T3 = 3;
parameter T4 = 4;
parameter T5 = 5;
parameter T6 = 6;
//----btfsc抬ss choose the bit of both
assign btfsc_skip_bit = ram_out[ir[9:7]] == 0;//these two are almost same
assign btfss_skip_bit = ram_out[ir[9:7]] == 1;//
assign btfsc_btfss_skip_bit = (btfsc&btfsc_skip_bit)|(btfss&btfss_skip_bit);
//----
always_ff @(posedge clk)
begin
	if(rst) ps <=  0;
	else ps <=  ns;
	
end
assign ps_out = ps;
assign d = ir[7];

always_comb //all controller case like movelw addlw...
begin
	load_ir = 0;
	load_mar = 0;
	load_pc = 0;
	load_w = 0;
	ram_en = 0;
	sel_pc = 0;
	rst_ir = 0;
	sel_alu = 0;
	ns = 0;
	alu_op = 5;
	sel_bus = 0;
	sel_ram_mux = 0;
	load_port_b = 0;
	push = 0;
	pop = 0;
	case(ps)
		T0:
			begin
			ns = T1;
			end
		T1:
			begin
			load_mar = 1;
			load_pc = 1;
			ns = T2;
			end
		T2:
			begin
			
			ns = T3;
			end
		T3:
			begin
			load_ir = 1;
			ns = T4;
			end
		T4:
			begin
			load_mar = 1;
			load_pc = 1;
			sel_pc = 0;
			if(movelw)
			begin
				load_w = 1;
				alu_op = 5;
			end
			else if(addlw)
			begin
				load_w = 1;
				alu_op = 0;
			end
			else if(andlw)
			begin		
				load_w = 1;
				alu_op = 2;
			end
			else if(iorlw)
			begin 	
				load_w = 1;
				alu_op = 3;
			end
			else if(sublw)
			begin
				load_w = 1;
				alu_op = 1;
			end
			else if(xorlw)
			begin
				load_w = 1;
				alu_op = 4;
			end
			
			else if(addwf)
			begin
				alu_op = 0;
				sel_alu = 1;
				if(d)	ram_en = 1;
				else	load_w = 1;
			end
			else if(andwf)
			begin
				alu_op = 2;
				sel_alu = 1;
				if(d)	ram_en = 1;
				else	load_w = 1;
			end
			else if(clrf)
			begin
				alu_op = 8;
				ram_en = 1;
			end
			else if(clrw)
			begin
				alu_op = 8;
				load_w = 1;
			end
			else if(comf)
			begin
				alu_op = 9;
				sel_alu = 1;
				ram_en = 1;
			end
			else if(decf)
			begin
				alu_op = 7;
				sel_alu = 1;
				ram_en = 1;
			end
			else if(incf)
			begin
				alu_op = 6;
				sel_alu = 1;
				if(d) 
				begin
					ram_en = 1;
					sel_bus = 0;
				end
				else load_w = 1;
			end
			else if(iorwf)
			begin
				alu_op = 3;
				sel_alu = 1;
				if(d) 
				begin
					ram_en = 1;
					sel_bus = 0;
				end
				else load_w = 1;
			end
			else if(movf)
			begin
				alu_op = 5;
				sel_alu = 1;
				if(d) 
				begin
					ram_en = 1;
					sel_bus = 0;
				end
				else load_w = 1;
			end
			else if(movwf)
			begin
				sel_bus = 1;
				if(addr_port_b) load_port_b = 1;
				else if(addr_port_b == 0) ram_en = 1;
			end
			else if(subwf)
			begin
				alu_op = 1;
				sel_alu = 1;
				if(d) 
				begin
					ram_en = 1;
					sel_bus = 0;
				end
				else load_w = 1;
			end
			else if(xorwf)
			begin
				alu_op = 4;
				sel_alu = 1;
				if(d) 
				begin
					ram_en = 1;
					sel_bus = 0;
				end
				else load_w = 1;
			end
			else if(bcf)
			begin
				alu_op = 5;
				sel_alu = 1;
				sel_ram_mux = 1;
				ram_en = 1;
			end
			else if(bsf)
			begin
				alu_op = 5;
				sel_alu = 1;
				sel_ram_mux = 2;
				ram_en = 1;
			end
			
			
			else if(asrf)
			begin
				sel_alu = 1;
				sel_ram_mux = 0;
				alu_op = 4'hA;
				if(d)
				begin
					sel_bus = 0;
					ram_en = 1;
				end
				else load_w = 1;
			end
			else if(lslf)
			begin
				sel_alu = 1;
				sel_ram_mux = 0;
				alu_op = 4'hB;
				if(d)
				begin
					sel_bus = 0;
					ram_en = 1;
				end
				else load_w = 1;
			end
			else if(lsrf)
			begin
				sel_alu = 1;
				sel_ram_mux = 0;
				alu_op = 4'hC;
				if(d)
				begin
					sel_bus = 0;
					ram_en = 1;
				end
				else load_w = 1;
			end
			else if(rlf)
			begin
				sel_alu = 1;
				sel_ram_mux = 0;
				alu_op = 4'hD;
				if(d)
				begin
					sel_bus = 0;
					ram_en = 1;
				end
				else load_w = 1;
			end
			else if(rrf)
			begin
				sel_alu = 1;
				sel_ram_mux = 0;
				alu_op = 4'hE;
				if(d)
				begin
					sel_bus = 0;
					ram_en = 1;
				end
				else load_w = 1;
			end
			else if(swapf)
			begin
				sel_alu = 1;
				sel_ram_mux = 0;
				alu_op = 4'hF;
				if(d)
				begin
					sel_bus = 0;
					ram_en = 1;
				end
				else load_w = 1;
			end
			else if(call)
			begin
				push = 1;
			end
			
			
			ns = T5;
			end
		T5:
			begin
			if(goto)	
			begin
				sel_pc = 1;
				load_pc = 1;
			end
			else if(call)
			begin
				sel_pc = 1;
				load_pc = 1;
			end
			else if(RETURN)
			begin
				sel_pc = 2;
				load_pc = 1;
				pop = 1;
			end
			else if(bra)
			begin
				load_pc = 1;
				sel_pc = 3;
			end
			else if(brw)
			begin
				load_pc = 1;
				sel_pc = 4;
			end
			ns = T6;
			end	
		T6:
			begin
			load_ir = 1;
			if(goto)	
			begin
				rst_ir = 1;
			end
			else if(call)
			begin
				rst_ir = 1;
			end
			else if(RETURN)
			begin
				rst_ir = 1;
			end
			else if(decfsz)
			begin
				sel_alu = 1;
				alu_op = 7;
				if(d)
				begin	
					ram_en = 1;
					sel_bus = 0;
					if(alu_zero)
					begin
						rst_ir = 1;
					end
				end
				else
				begin
					load_w = 1;
					if(alu_zero)
					begin
						rst_ir = 1;
					end
				end
			end
			else if(incfsz)
			begin
				sel_alu = 1;
				alu_op = 6;
				if(d)
				begin
					
					ram_en = 1;
					sel_bus = 0;
					if(alu_zero)
					begin
						rst_ir = 1;
					end
				end
				else
				begin
					load_w = 1;
					if(alu_zero)
					begin
						rst_ir = 1;
					end
				end
			end
			else if(btfsc)
			begin
				if(btfsc_btfss_skip_bit)
				begin
					rst_ir = 1;
				end
			end
			else if(btfss)
			begin
				if(btfsc_btfss_skip_bit)
				begin
					rst_ir = 1;
				end
			end
			else if(bra)
			begin
				rst_ir = 1;
			end
			else if(brw)
			begin
				rst_ir = 1;
			end
			ns = T4;
			end				
	endcase
end
endmodule


