`include "Instructions.v"

module EXE_Stage(
	input clk, rst,
	input [31:0] pc_in,
	input wb_enable_in, mem_read_in, mem_write_in, branch_taken_in,
	input [3:0] execute_command_in,
	input immediate_in,
	input [23:0] signed_immediate_24_in,
	input [11:0] shift_operand_in,
	input [31:0] val_rn_in, val_rm_in,
	input [3:0] status_register_in, 

	output [3:0] status_bits,
	output [31:0] alu_res,
	output [31:0] branch_address
);
    assign wb_enable_out = wb_enable_in;
    assign mem_read_out = mem_read_in;
    assign mem_write_out = mem_write_in;
	
	wire [31:0] val2out;
	
	ALU alu(
    	.alu_in1(val_rn_in), 
		.alu_in2(val2out),
    	.alu_command(execute_command_in),
    	.status_register(status_register_in),

    	.alu_out(alu_res),
    	.alu_status_register_out(status_bits)
    );

	wire is_mem_command;
	assign is_mem_command = mem_read_in | mem_write_in;
	
	val2gen v2g(
		.val_rm(val_rm_in),
        .shift_operand(shift_operand_in),
        .immediate(immediate_in), 
		.is_mem_command(is_mem_command),

        .val2_out(val2out)
		);

	wire [31:0] adder_out;
	wire [31:0] sign_immediate_extended = { {8{signed_immediate_24_in[23]}}, signed_immediate_24_in}; 
	assign adder_out = pc_in + (sign_immediate_extended << 2);	
	assign branch_address = adder_out;

endmodule 

module ALU (
	input [31:0] alu_in1, 
    input [31:0] alu_in2,
    input [3:0] alu_command,
    input [3:0] status_register,
    output reg [31:0] alu_out,
    output [3:0] alu_status_register_out
);
	wire cin;
    assign cin = status_register[1];

    wire z, n; 
    assign z = (alu_out == 32'b0) ? 1'b1 : 1'b0;
    assign n = (alu_out[31]); 
    
    reg v, cout; 
    assign alu_status_register_out = {z, cout, n, v};

    always @(*) begin
        cout = 1'b0;
        v = 1'b0;

        case (alu_command)
            `MOV: alu_out = alu_in2;
            `MVN: alu_out = ~alu_in2; 
            `ADD: begin
                {cout, alu_out} = alu_in1 + alu_in2;
                v = ((alu_in1[31] == alu_in2[31])
                        & (alu_out[31] != alu_in1[31]));
                end
            `ADC: begin
                {cout, alu_out} = alu_in1 + alu_in2 + cin;
                v = ((alu_in1[31] == alu_in2[31]) 
                    & (alu_out[31] != alu_in1[31]));
                end
            `SUB: begin
                {cout, alu_out} = alu_in1 - alu_in2;
                v = ((alu_in1[31] == ~alu_in2[31])
                    & (alu_out[31] != alu_in1[31]));
                end
            `SBC: begin
                {cout, alu_out} = alu_in1 - alu_in2 - 1 + cin;
                v = ((alu_in1[31] == alu_in2[31])
                    & (alu_out[31] != alu_in1[31]));
                end
            `AND: alu_out = alu_in1 & alu_in2;
            `ORR: alu_out = alu_in1 | alu_in2;
            `EOR: alu_out = alu_in1 ^ alu_in2;
            `CMP: begin
                {cout, alu_out} = {alu_in1[31], {alu_in1}} - {alu_in2[31], {alu_in2}};
                    v = ((alu_in1[31] == ~alu_in2[31])
                        & (alu_out[31] != alu_in1[31]));
                end

            `TST: alu_out = alu_in1 & alu_in2;
            `LDR: alu_out =	alu_in1 + alu_in2;
            `STR: alu_out = alu_in1 + alu_in2;
        endcase
    end    
endmodule


module val2gen(
		input [31:0] val_rm,
        input [11:0] shift_operand,
        input immediate, is_mem_command,
        output reg [31:0] val2_out
);

    reg [2 * 32 - 1 : 0] tmp;
    integer i;
    always @(val_rm, shift_operand, immediate, is_mem_command) begin
        val2_out = 32'b0;
        tmp = 0;
        if (is_mem_command == 1'b0) begin
            if (immediate == 1'b1) begin
                val2_out = {24'b0 ,shift_operand[7 : 0]};
                tmp = {val2_out, val2_out} >> {shift_operand[11 : 8], 1'b0};
                val2_out = tmp[31 : 0];
                
                // val2_out = {24'b0 ,shift_operand[7 : 0]};
                // val2_out = val2_out >>> {shift_operand[11 : 8], 1'b0};
            end 
            else if(immediate == 1'b0 && shift_operand[4] == 0) begin
                case(shift_operand[6:5])
                    `LSL_SHIFT : begin
                        val2_out = val_rm << shift_operand[11 : 7];
                    end
                    `LSR_SHIFT : begin
                        val2_out = val_rm >> shift_operand[11 : 7];
                    end
                    `ASR_SHIFT : begin
                        val2_out = val_rm >>> shift_operand[11 : 7];
                    end
                    `ROR_SHIFT : begin
                        tmp = {val_rm, val_rm} >> (shift_operand[11 : 7]);
                        val2_out = tmp[31 : 0];
                    end
                endcase
            end
        end 
        else begin
            val2_out = { {20{shift_operand[11]}} , shift_operand[11 : 0]}; 
        end

    end
endmodule 

module StatusRegister(
	input clk, rst, ld,
    input [3:0] in,
    output reg [3:0] out
);
    always@(negedge clk, posedge rst) begin
		if (rst) 
            out <= 0;
		else 
        if (ld) 
            out <= in;
	end
endmodule