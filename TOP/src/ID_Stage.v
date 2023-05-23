`include "Instructions.v"

module ID_Stage (
	input clk, rst,
	input [31:0] pc_in,
	input [31:0] instruction_in,
	input hazard,
	input [3:0] status_register_in,
	
	input [3:0] wb_dest,
	input [31:0] wb_value,
	input wb_enable_WB,

	output [31:0] pc,

	output mem_read_out, mem_write_out, wb_enable_out,
	output [3:0] alu_input_cmd_out,
	output branch_taken_out, status_write_enable_out,
	
	output [31:0] val_rn, val_rm,
	
	output two_src,
	output [3:0] src1_out, src2_out,
	
	output immediate_out,
	output [23:0] signed_immediate,
	output [11:0] shift_operand,
	output [3:0] dest_reg_out
);

	assign pc = pc_in;

	wire two_src_mem_write_en;
	wire control_unit_mux_enable;
	wire mem_write;
	assign two_src_mem_write_en = control_unit_mux_enable == 1'b0 ? mem_write : 1'b0;
	assign two_src = (~instruction_in[25]) | (two_src_mem_write_en);

	wire mem_read, wb_enable, branch_taken, status_write_enable;
	wire [3:0] alu_input_cmd;

	ControlUnit control_unit(
		.mode(instruction_in[27:26]),
		.opcode(instruction_in[24:21]), 
		.s(instruction_in[20]),
		.alu_input_cmd(alu_input_cmd), 
		.mem_read(mem_read), 
		.mem_write(mem_write),
		.wb_enable(wb_enable), 
		.branch_taken(branch_taken),
		.status_write_enable(status_write_enable)
	);

	wire cond_state;
	assign control_unit_mux_enable = ~cond_state | hazard;
	
	assign mem_read_out = control_unit_mux_enable == 1'b0 ? mem_read : 1'b0;
	assign mem_write_out = control_unit_mux_enable == 1'b0 ? mem_write : 1'b0;
	assign wb_enable_out = control_unit_mux_enable == 1'b0 ? wb_enable : 1'b0;
	assign branch_taken_out = control_unit_mux_enable == 1'b0 ? branch_taken : 1'b0;
	assign status_write_enable_out = control_unit_mux_enable == 1'b0 ? status_write_enable : 1'b0;
	assign alu_input_cmd_out = control_unit_mux_enable == 1'b0 ? alu_input_cmd : 4'b0;

	wire[3:0] reg_file_src1, reg_file_src2;
	
	assign reg_file_src1 = instruction_in[19:16];
	assign src1_out = reg_file_src1;

	assign reg_file_src2 = mem_write_out ? instruction_in[15:12] : instruction_in[3:0];
	assign src2_out = reg_file_src2; 

	RegisterFile register_file(
		.clk(clk), 
		.rst(rst),
    	.src1(reg_file_src1), 
		.src2(reg_file_src2),
		.dest_wb(wb_dest),
		.result_wb(wb_value),
    	.wb_en(wb_enable_WB),
		.reg1(val_rn),
		.reg2(val_rm),
	);

	ConditionCheck condition_check(
		.cond(instruction_in[31:28]),
		.status_reg(status_register_in),
		.cond_state(cond_state)
    );

	assign shift_operand = instruction_in[11:0];
	assign signed_immediate = instruction_in[23:0];
	assign dest_reg_out = instruction_in[15:12];
	assign immediate_out = instruction_in[25];		
endmodule

module RegisterFile (
	input clk, rst, wb_en,
	input [3:0] src1, src2, dest_wb,
	input [31:0] result_wb,
	output [31:0] reg1, reg2
);

	reg [31:0] reggg [0:15];

    // initial begin
	// 	//$readmemb("registerFile.txt", reggg);
	// 	reggg[0] = 32'd0;
	// end


	integer i;
	initial begin
		for (i = 0; i < 15; i=i+1)
			reggg[i] = i;
	end

	always @(posedge rst, negedge clk) begin
		if (rst) begin
		for (i = 0; i < 15; i=i+1)
			reggg[i] = i;
		end
		else if (wb_en)
			reggg[dest_wb] = result_wb;
	end
	
	assign reg1 = reggg[src1];
	assign reg2 = reggg[src2];	
endmodule

module ControlUnit (
	input[1:0] mode,
	input[3:0] opcode,
	input s,
	output reg[3:0] alu_input_cmd,
	output reg mem_read, mem_write,
	output reg wb_enable, branch_taken, status_write_enable
);
	always @(mode, opcode, s) begin
		{status_write_enable,
		alu_input_cmd,
		branch_taken,
		mem_write,
		wb_enable,
		mem_read} = 9'd0;

		case (mode)
			`ARITHMETHIC_TYPE : begin
				alu_input_cmd = opcode;
				case (opcode) 
					`MOV : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`MVN : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`ADD : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`ADC : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`SUB : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end		
					
					`SBC : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`AND : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`ORR : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end

					`EOR : begin
						wb_enable = 1'b1;
						status_write_enable = s;
					end
					
					`CMP : begin
						wb_enable = 1'b0;
						status_write_enable = 1'b1;
					end

					`TST: begin
						wb_enable = 1'b0;
						status_write_enable = 1'b1;
					end
				endcase
			end

			`MEMORY_TYPE : begin
				case (s) 
					`S_LDR: begin
						wb_enable = 1'b1;
						status_write_enable = 1'b1;
						alu_input_cmd = `LDR;
						mem_read = 1'b1;
					end

					`S_STR: begin
						wb_enable = 1'b0;
						status_write_enable = 1'b0;
						alu_input_cmd = `STR;
						mem_write = 1'b1;
					end
				endcase
			end

			`BRANCH_TYPE : begin
				branch_taken = 1'b1;
			end
		endcase
	end
	
endmodule


module ConditionCheck (
    input [3:0] cond,
    input [3:0] status_reg,
    output reg cond_state
);
    wire z, c, n, v;
    assign {z, c, n, v} = status_reg;

    always @(cond, z, c, n, v) begin
        cond_state = 1'b0;
        case(cond)
            4'b0000 : 
                cond_state = z;
            4'b0001 : 
                cond_state = ~z;
            4'b0010 : 
                cond_state = c;
            4'b0011 : 
                cond_state = ~c;
            4'b0100 : 
                cond_state = n;
            4'b0101 : 
                cond_state = ~n;
            4'b0110 : 
                cond_state = v;
            4'b0111 :
                cond_state = ~v;
            4'b1000 : 
                cond_state = c & ~z;
            4'b1001 : 
                cond_state = ~c | z;
            4'b1010 : 
                cond_state = (n & v) | (~n & ~v);
            4'b1011 : 
                cond_state = (n & ~v) | (~n & v);
            4'b1100 : 
                cond_state = ~z & ((n & v) | (~n & ~v));
            4'b1101 : 
                cond_state = z | (n & ~v) | (~n & ~v);
            4'b1110 : 
                cond_state = 1'b1;
        endcase
    end
endmodule


module HazardDetector (
    input [3:0] src1, src2,
    input [3:0] exe_wb_dest,
    input [3:0] mem_wb_dest,
    input two_src, 
    input exe_wb_enable,
    input mem_wb_enable,
    input hazard_en,
    output reg hazard
);  

    always @(*) begin
        hazard = 1'b0;
        if (hazard_en) begin
            if ((src1 == exe_wb_dest) && (exe_wb_enable == 1'b1))
                hazard = 1'b1;
            else
            if ((src1 == mem_wb_dest) && (mem_wb_enable == 1'b1))
                hazard = 1'b1;
            else
            if ((src2 == exe_wb_dest) && (exe_wb_enable == 1'b1) && (two_src == 1'b1))
                hazard = 1'b1;            
            else
            if ((src2 == mem_wb_dest) && (mem_wb_enable == 1'b1) && (two_src == 1'b1))
                hazard = 1'b1;    
            else
                hazard = 1'b0;
        end
        
        if (~hazard_en) begin
                hazard = 1'b0;
        end
    end

endmodule