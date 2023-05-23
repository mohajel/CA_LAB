module ID_Stage_Reg (
	input clk, rst, flush,
	input [31:0] pc_in,
	input mem_read_in, mem_write_in, wb_enable_in,
	input branch_taken_in, status_write_enable_in,
	input [3:0] alu_input_cmd_in,
	input [31:0] val_rn_in, val_rm_in,
	input immediate_in,
	input [23:0] signed_immediate_in,
	input [11:0] shift_operand_in,
	input [3:0] dest_reg_in,
	input [3:0] status_register_in,

	output reg [31:0] pc_out,
	output reg mem_read_out, mem_write_out, wb_enable_out,
	output reg branch_taken_out, status_write_enable_out,
	output reg [3:0] alu_input_cmd_out,
	output reg [31:0] val_rn_out, val_rm_out,
	output reg immediate_out,
	output reg [23:0] signed_immediate_out,
	output reg [11:0] shift_operand_out,
	output reg [3:0] dest_reg_out,
	output reg [3:0] status_register_out,	

	input [3:0] src1_in, src2_in,
	output reg [3:0] src1_out, src2_out
);

	always @(posedge clk, posedge rst) begin
		if (rst) begin
			pc_out <= 32'b0;
			{mem_read_out, mem_write_out, wb_enable_out} <= {3'b0};
			{branch_taken_out, status_write_enable_out} <= {2'b0};
			alu_input_cmd_out <= 4'b0;
			{val_rn_out, val_rm_out} <= {32'b0, 32'b0};
			immediate_out <= 1'b0;
			signed_immediate_out <= 24'b0;
			shift_operand_out <= 12'b0;
			dest_reg_out <= 4'b0;
			status_register_out <= 4'b0;
		end
		else begin
			if (flush) begin
				pc_out <= 32'b0;
				{mem_read_out, mem_write_out, wb_enable_out} <= {3'b0};
				{branch_taken_out, status_write_enable_out} <= {2'b0};
				alu_input_cmd_out <= 4'b0;
				{val_rn_out, val_rm_out} <= {32'b0, 32'b0};
				immediate_out <= 1'b0;
				signed_immediate_out <= 24'b0;
				shift_operand_out <= 12'b0;
				dest_reg_out <= 4'b0;
				status_register_out <= 4'b0;
				src1_out <= 4'b0;
				src2_out <= 4'b0;
			end
			else begin
				pc_out <= pc_in;
				{mem_read_out, mem_write_out, wb_enable_out} <= {mem_read_in, mem_write_in, wb_enable_in};
				{branch_taken_out, status_write_enable_out} <= {branch_taken_in, status_write_enable_in};
				alu_input_cmd_out <= alu_input_cmd_in;
				{val_rn_out, val_rm_out} <= {val_rn_in, val_rm_in};
				immediate_out <= immediate_in;
				signed_immediate_out <= signed_immediate_in;
				shift_operand_out <= shift_operand_in;
				dest_reg_out <= dest_reg_in;
				status_register_out <= status_register_in;
				src1_out <= src1_in;
				src2_out <= src2_in;
			end
		end
	end
		
endmodule