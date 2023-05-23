module WB_Stage (
	input mem_read_in,
	input [31:0] alu_res_in,
	input [31:0] data_mem_in,
	output [31:0] wb_value
);
	assign wb_value = mem_read_in ? data_mem_in : alu_res_in;
endmodule