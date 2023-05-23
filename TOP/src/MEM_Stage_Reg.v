module MEM_Stage_Reg(
	input clk,
	input rst,
	input wb_enable_in,
	input mem_read_in,
	input [31:0] alu_res_in,
	input [31:0] data_mem_in,
	input [3:0] dest_reg_in,
	
	output reg wb_enable_out,
	output reg mem_read_out,
	output reg [31:0] alu_res_out,
	output reg [31:0] data_mem_out,
	output reg [3:0] dest_reg_out
);

	always @(posedge clk, posedge rst) begin
        if (rst) begin
			wb_enable_out <= 1'b0;
			mem_read_out <= 1'b0;
			alu_res_out <= 32'b0;
			data_mem_out <= 32'b0;
			dest_reg_out <= 32'b0;
		end
        else begin
			wb_enable_out <= wb_enable_in;
			mem_read_out <= mem_read_in;
			alu_res_out <= alu_res_in;
			data_mem_out <= data_mem_in;
			dest_reg_out <= dest_reg_in;
		end
	end
endmodule