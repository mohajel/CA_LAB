module MEM_Stage (
	input clk, rst,
	input mem_write_en, mem_read_en,
	input [31:0] alu_res,
	input [31:0] val_rm,
	output [31:0] mem_out
);

	Memory mem (
		.clk(clk),
		.rst(rst),
		.read_en(mem_read_en),
		.write_en(mem_write_en),
		.address(alu_res),
		.in(val_rm),
		.out(mem_out)
	);
		
endmodule

module Memory (
	input clk, rst, read_en, write_en,
	input [31:0] address,
	input [31:0] in,
	output [31:0] out
);

    reg [31:0] memory [0:63];

	wire [5:0] mem_address;
	assign mem_address = (address - 32'd1024) >> 2;

	always @(negedge clk) begin
		if (write_en)
			memory[mem_address] = in;
	end
	
	assign out = read_en ? memory[mem_address] : 0;
endmodule