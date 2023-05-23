module ARM (input clk, rst);
    wire [31:0] pc, instruction;
    wire id_reg_branch_taken_out;
	wire [31:0] exe_branch_address;
    wire freeze;

    IF_Stage if_stage (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .branch_taken(id_reg_branch_taken_out),
        .branch_address(exe_branch_address),
        .pc(pc),
        .instruction(instruction)
    );

    wire [31:0] if_reg_pc, if_reg_ins;
    IF_Stage_Reg if_stage_reg (
        .clk(clk),
        .rst(rst),
        .freeze(freeze),
        .flush(id_reg_branch_taken_out),
        .pc_in(pc),
        .instruction_in(instruction),
        .pc(if_reg_pc),
        .instruction(if_reg_ins)
    );

    wire [31:0] id_pc;
    wire id_mem_read_out, id_mem_write_out, id_wb_enable_out;
	wire [3:0] id_alu_input_cmd_out;
	wire id_branch_taken_out, id_status_write_enable_out;
	wire [31:0] id_val_rn, id_val_rm;
	wire id_two_src;
	wire [3:0] id_src1_out, id_src2_out;
	wire id_immediate_out;
	wire [23:0] id_signed_immediate;
	wire [11:0] id_shift_operand;
	wire [3:0] id_dest_reg_out;
	wire [3:0] status_register_out;
	wire [31:0] wb_value;
	wire mem_reg_wb_enable;
	wire [3:0] mem_reg_dest_reg;
    ID_Stage id_stage (
        .clk(clk),
        .rst(rst),
        .pc_in(if_reg_pc),
        .instruction_in(if_reg_ins),
	    .hazard(freeze),
	    .status_register_in(status_register_out),
	    .wb_dest(mem_reg_dest_reg),
        .wb_value(wb_value),
	    .wb_enable_WB(mem_reg_wb_enable),
	    .pc(id_pc),
	    .mem_read_out(id_mem_read_out),
        .mem_write_out(id_mem_write_out),
        .wb_enable_out(id_wb_enable_out),
	    .alu_input_cmd_out(id_alu_input_cmd_out),
	    .branch_taken_out(id_branch_taken_out),
        .status_write_enable_out(id_status_write_enable_out),
	    .val_rn(id_val_rn),
        .val_rm(id_val_rm),
	    .two_src(id_two_src),
	    .src1_out(id_src1_out),
        .src2_out(id_src2_out),
	    .immediate_out(id_immediate_out),
	    .signed_immediate(id_signed_immediate),
	    .shift_operand(id_shift_operand),
	    .dest_reg_out(id_dest_reg_out)
    );

    wire [31:0] id_reg_pc_out;
	wire id_reg_mem_read_out, id_reg_mem_write_out, id_reg_wb_enable_out;
	wire id_reg_status_write_enable_out;
	wire [3:0] id_reg_alu_input_cmd_out;
	wire [31:0] id_reg_val_rn_out, id_reg_val_rm_out;
	wire id_reg_immediate_out;
	wire [23:0] id_reg_signed_immediate_out;
	wire [11:0] id_reg_shift_operand_out;
	wire [3:0] id_reg_dest_reg_out;
	wire [3:0] id_reg_status_register_out;
	wire [3:0] id_reg_src1_out, id_reg_src2_out;
    ID_Stage_Reg id_stage_reg (
        .clk(clk),
        .rst(rst),
        .flush(id_reg_branch_taken_out),
	    .pc_in(id_pc),
	    .mem_read_in(id_mem_read_out),
        .mem_write_in(id_mem_write_out),
        .wb_enable_in(id_wb_enable_out),
	    .branch_taken_in(id_branch_taken_out),
        .status_write_enable_in(id_status_write_enable_out),
	    .alu_input_cmd_in(id_alu_input_cmd_out),
	    .val_rn_in(id_val_rn),
        .val_rm_in(id_val_rm),
	    .immediate_in(id_immediate_out),
	    .signed_immediate_in(id_signed_immediate),
	    .shift_operand_in(id_shift_operand),
	    .dest_reg_in(id_dest_reg_out),
	    .status_register_in(status_register_out),
	    .pc_out(id_reg_pc_out),
	    .mem_read_out(id_reg_mem_read_out),
        .mem_write_out(id_reg_mem_write_out), 
        .wb_enable_out(id_reg_wb_enable_out),
	    .branch_taken_out(id_reg_branch_taken_out), 
        .status_write_enable_out(id_reg_status_write_enable_out),
	    .alu_input_cmd_out(id_reg_alu_input_cmd_out),
	    .val_rn_out(id_reg_val_rn_out), 
        .val_rm_out(id_reg_val_rm_out),
	    .immediate_out(id_reg_immediate_out),
	    .signed_immediate_out(id_reg_signed_immediate_out),
	    .shift_operand_out(id_reg_shift_operand_out),
	    .dest_reg_out(id_reg_dest_reg_out),
	    .status_register_out(id_reg_status_register_out),	

	    .src1_in(id_src1_out), 
        .src2_in(id_src2_out),
	    .src1_out(id_reg_src1_out), 
        .src2_out(id_reg_src2_out)
    );

    wire exe_wb_enable_out, exe_mem_read_out, exe_mem_write_out;
	wire [3:0] exe_status_bits;
	wire [31:0] exe_alu_res;
    EXE_Stage exe_stage (
        .clk(clk),
        .rst(rst),
        .pc_in(id_reg_pc_out),
        .wb_enable_in(id_reg_wb_enable_out),
        .mem_read_in(id_reg_mem_read_out),
        .mem_write_in(id_reg_mem_write_out),
        .branch_taken_in(id_reg_branch_taken_out),
        .execute_command_in(id_reg_alu_input_cmd_out),
        .immediate_in(id_reg_immediate_out),
        .signed_immediate_24_in(id_reg_signed_immediate_out),
        .shift_operand_in(id_reg_shift_operand_out),
        .val_rn_in(id_reg_val_rn_out),
        .val_rm_in(id_reg_val_rm_out),
        .status_register_in(id_reg_status_register_out),
        .status_bits(exe_status_bits),
        .alu_res(exe_alu_res),
        .branch_address(exe_branch_address)
    );

    StatusRegister status_register (
        .clk(clk),
        .rst(rst),
        .ld(id_reg_status_write_enable_out),
        .in(exe_status_bits),
        .out(status_register_out)
    );

	wire exe_reg_wb_enable_out, exe_reg_mem_read_out, exe_reg_mem_write_out;
	wire [31:0] exe_reg_alu_res_out, exe_reg_val_rm_out;
	wire [3:0] exe_reg_dest_out;
    EXE_Stage_Reg exe_stage_reg (
        .clk(clk),
        .rst(rst),
        .wb_enable_in(id_reg_wb_enable_out),
        .mem_read_in(id_reg_mem_read_out),
        .mem_write_in(id_reg_mem_write_out),
        .alu_res_in(exe_alu_res),
        .val_rm_in(id_reg_val_rm_out),
        .dest_in(id_reg_dest_reg_out),
        .wb_enable_out(exe_reg_wb_enable_out),
        .mem_read_out(exe_reg_mem_read_out),
        .mem_write_out(exe_reg_mem_write_out),
        .alu_res_out(exe_reg_alu_res_out),
        .val_rm_out(exe_reg_val_rm_out),
        .dest_out(exe_reg_dest_out)
    );

	wire [31:0] mem_out;
    MEM_Stage mem_stage (
	    .clk(clk),
        .rst(rst),
	    .mem_write_en(exe_reg_mem_write_out),
        .mem_read_en(exe_reg_mem_read_out),
	    .alu_res(exe_reg_alu_res_out),
	    .val_rm(exe_reg_val_rm_out),
	    .mem_out(mem_out)
    );

	wire mem_reg_mem_read;
	wire [31:0] mem_reg_alu_res;
	wire [31:0] mem_reg_data_mem;
    MEM_Stage_Reg mem_stage_reg (
        .clk(clk),
        .rst(rst),
	    .wb_enable_in(exe_reg_wb_enable_out),
	    .mem_read_in(exe_reg_mem_read_out),
	    .alu_res_in(exe_reg_alu_res_out),
	    .data_mem_in(mem_out),
	    .dest_reg_in(exe_reg_dest_out),
	    .wb_enable_out(mem_reg_wb_enable),
	    .mem_read_out(mem_reg_mem_read),
	    .alu_res_out(mem_reg_alu_res),
	    .data_mem_out(mem_reg_data_mem),
	    .dest_reg_out(mem_reg_dest_reg)
    );

    WB_Stage wb_stage (
	    .mem_read_in(mem_reg_mem_read),
	    .alu_res_in(mem_reg_alu_res),
	    .data_mem_in(mem_reg_data_mem),
	    .wb_value(wb_value)
    );


    HazardDetector HazardDetector(
		.src1(id_src1_out), 
		.src2(id_src2_out),
		.exe_wb_dest(id_reg_dest_reg_out),
		.mem_wb_dest(exe_reg_dest_out), 
		.two_src(id_two_src),
		.exe_wb_enable(id_reg_wb_enable_out),
		.mem_wb_enable(exe_reg_wb_enable_out),
		.hazard_en(1),
		.hazard(freeze)
	);

endmodule