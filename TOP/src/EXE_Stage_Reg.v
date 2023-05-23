module EXE_Stage_Reg(
	input clk, rst,
	input wb_enable_in, mem_read_in, mem_write_in,
	input [31:0] alu_res_in, val_rm_in,
	input [3:0] dest_in,
	
	output reg wb_enable_out, mem_read_out, mem_write_out,
	output reg [31:0] alu_res_out, val_rm_out,
	output reg [3:0] dest_out
);
	always @(posedge clk, posedge rst) 
        if (rst)
			begin
				{wb_enable_out, mem_read_out, mem_write_out} <= {3'b0};
				{alu_res_out, val_rm_out} <= {64'b0};
				dest_out <= 4'b0;
			end
        else
			begin
				{wb_enable_out, mem_read_out, mem_write_out} <= {wb_enable_in, mem_read_in, mem_write_in};
				{alu_res_out, val_rm_out} <= {alu_res_in, val_rm_in};
				dest_out <= dest_in;
			end
endmodule