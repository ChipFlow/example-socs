module riscv_if_stage (
	clk,
	rst_n,
	m_trap_base_addr_i,
	m_trap_base_addrx_i,
	u_trap_base_addr_i,
	trap_addr_mux_i,
	boot_addr_i,
	req_i,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	instr_err_pmp_i,
	hwlp_dec_cnt_id_o,
	is_hwlp_id_o,
	instr_valid_id_o,
	instr_rdata_id_o,
	is_compressed_id_o,
	illegal_c_insn_id_o,
	pc_if_o,
	pc_id_o,
	is_fetch_failed_o,
	clear_instr_valid_i,
	pc_set_i,
	mepc_i,
	uepc_i,
	depc_i,
	pc_mux_i,
	exc_pc_mux_i,
	exc_vec_pc_mux_i,
	jump_target_id_i,
	jump_target_ex_i,
	hwlp_start_i,
	hwlp_end_i,
	hwlp_cnt_i,
	halt_if_i,
	id_ready_i,
	if_busy_o,
	perf_imiss_o
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter RDATA_WIDTH = 32;
	parameter FPU = 0;
	parameter DM_HaltAddress = 32'h1a110800;
	input wire clk;
	input wire rst_n;
	input wire [23:0] m_trap_base_addr_i;
	input wire [23:0] m_trap_base_addrx_i;
	input wire [23:0] u_trap_base_addr_i;
	input wire [1:0] trap_addr_mux_i;
	input wire [30:0] boot_addr_i;
	input wire req_i;
	output wire instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [RDATA_WIDTH - 1:0] instr_rdata_i;
	input wire instr_err_pmp_i;
	output reg [N_HWLP - 1:0] hwlp_dec_cnt_id_o;
	output wire is_hwlp_id_o;
	output reg instr_valid_id_o;
	output reg [31:0] instr_rdata_id_o;
	output reg is_compressed_id_o;
	output reg illegal_c_insn_id_o;
	output wire [31:0] pc_if_o;
	output reg [31:0] pc_id_o;
	output reg is_fetch_failed_o;
	input wire clear_instr_valid_i;
	input wire pc_set_i;
	input wire [31:0] mepc_i;
	input wire [31:0] uepc_i;
	input wire [31:0] depc_i;
	input wire [2:0] pc_mux_i;
	input wire [2:0] exc_pc_mux_i;
	input wire [4:0] exc_vec_pc_mux_i;
	input wire [31:0] jump_target_id_i;
	input wire [31:0] jump_target_ex_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_start_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_end_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_cnt_i;
	input wire halt_if_i;
	input wire id_ready_i;
	output wire if_busy_o;
	output wire perf_imiss_o;
	reg [0:0] offset_fsm_cs;
	reg [0:0] offset_fsm_ns;
	wire if_valid;
	wire if_ready;
	reg valid;
	wire prefetch_busy;
	reg branch_req;
	reg [31:0] fetch_addr_n;
	wire fetch_valid;
	reg fetch_ready;
	wire [31:0] fetch_rdata;
	wire [31:0] fetch_addr;
	reg is_hwlp_id_q;
	wire fetch_is_hwlp;
	reg [31:0] exc_pc;
	wire hwlp_jump;
	wire hwlp_branch;
	wire [31:0] hwlp_target;
	wire [N_HWLP - 1:0] hwlp_dec_cnt;
	reg [N_HWLP - 1:0] hwlp_dec_cnt_if;
	reg [23:0] trap_base_addr;
	wire fetch_failed;
	localparam riscv_defines_EXC_PC_DBD = 3'b010;
	localparam riscv_defines_EXC_PC_EXCEPTION = 3'b000;
	localparam riscv_defines_EXC_PC_IRQ = 3'b001;
	localparam riscv_defines_TRAP_MACHINE = 2'b00;
	localparam riscv_defines_TRAP_MACHINEX = 2'b10;
	localparam riscv_defines_TRAP_USER = 2'b01;
	always @(*) begin : EXC_PC_MUX
		if (_sv2v_0)
			;
		exc_pc = 1'sb0;
		case (trap_addr_mux_i)
			riscv_defines_TRAP_MACHINE: trap_base_addr = m_trap_base_addr_i;
			riscv_defines_TRAP_USER: trap_base_addr = u_trap_base_addr_i;
			riscv_defines_TRAP_MACHINEX: trap_base_addr = m_trap_base_addrx_i;
			default:
				;
		endcase
		case (exc_pc_mux_i)
			riscv_defines_EXC_PC_EXCEPTION: exc_pc = {trap_base_addr, 8'h00};
			riscv_defines_EXC_PC_IRQ: exc_pc = {trap_base_addr, 1'b0, exc_vec_pc_mux_i[4:0], 2'b00};
			riscv_defines_EXC_PC_DBD: exc_pc = {DM_HaltAddress};
			default:
				;
		endcase
	end
	localparam riscv_defines_PC_BOOT = 3'b000;
	localparam riscv_defines_PC_BRANCH = 3'b011;
	localparam riscv_defines_PC_DRET = 3'b111;
	localparam riscv_defines_PC_EXCEPTION = 3'b100;
	localparam riscv_defines_PC_FENCEI = 3'b001;
	localparam riscv_defines_PC_JUMP = 3'b010;
	localparam riscv_defines_PC_MRET = 3'b101;
	localparam riscv_defines_PC_URET = 3'b110;
	always @(*) begin
		if (_sv2v_0)
			;
		fetch_addr_n = 1'sb0;
		case (pc_mux_i)
			riscv_defines_PC_BOOT: fetch_addr_n = {boot_addr_i, 1'b0};
			riscv_defines_PC_JUMP: fetch_addr_n = jump_target_id_i;
			riscv_defines_PC_BRANCH: fetch_addr_n = jump_target_ex_i;
			riscv_defines_PC_EXCEPTION: fetch_addr_n = exc_pc;
			riscv_defines_PC_MRET: fetch_addr_n = mepc_i;
			riscv_defines_PC_URET: fetch_addr_n = uepc_i;
			riscv_defines_PC_DRET: fetch_addr_n = depc_i;
			riscv_defines_PC_FENCEI: fetch_addr_n = pc_id_o + 4;
			default:
				;
		endcase
	end
	generate
		if (RDATA_WIDTH == 32) begin : prefetch_32
			riscv_prefetch_buffer prefetch_buffer_i(
				.clk(clk),
				.rst_n(rst_n),
				.req_i(req_i),
				.branch_i(branch_req),
				.addr_i({fetch_addr_n[31:1], 1'b0}),
				.hwloop_i(hwlp_jump),
				.hwloop_target_i(hwlp_target),
				.hwlp_branch_o(hwlp_branch),
				.ready_i(fetch_ready),
				.valid_o(fetch_valid),
				.rdata_o(fetch_rdata),
				.addr_o(fetch_addr),
				.is_hwlp_o(fetch_is_hwlp),
				.instr_req_o(instr_req_o),
				.instr_addr_o(instr_addr_o),
				.instr_gnt_i(instr_gnt_i),
				.instr_rvalid_i(instr_rvalid_i),
				.instr_err_pmp_i(instr_err_pmp_i),
				.fetch_failed_o(fetch_failed),
				.instr_rdata_i(instr_rdata_i),
				.busy_o(prefetch_busy)
			);
		end
		else if (RDATA_WIDTH == 128) begin : prefetch_128
			riscv_prefetch_L0_buffer prefetch_buffer_i(
				.clk(clk),
				.rst_n(rst_n),
				.req_i(1'b1),
				.branch_i(branch_req),
				.addr_i({fetch_addr_n[31:1], 1'b0}),
				.hwloop_i(hwlp_jump),
				.hwloop_target_i(hwlp_target),
				.ready_i(fetch_ready),
				.valid_o(fetch_valid),
				.rdata_o(fetch_rdata),
				.addr_o(fetch_addr),
				.is_hwlp_o(fetch_is_hwlp),
				.instr_req_o(instr_req_o),
				.instr_addr_o(instr_addr_o),
				.instr_gnt_i(instr_gnt_i),
				.instr_rvalid_i(instr_rvalid_i),
				.instr_rdata_i(instr_rdata_i),
				.busy_o(prefetch_busy)
			);
			assign hwlp_branch = 1'b0;
			assign fetch_failed = 1'b0;
		end
	endgenerate
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			offset_fsm_cs <= 1'd1;
		else
			offset_fsm_cs <= offset_fsm_ns;
	always @(*) begin
		if (_sv2v_0)
			;
		offset_fsm_ns = offset_fsm_cs;
		fetch_ready = 1'b0;
		branch_req = 1'b0;
		valid = 1'b0;
		case (offset_fsm_cs)
			1'd1:
				if (req_i) begin
					branch_req = 1'b1;
					offset_fsm_ns = 1'd0;
				end
			1'd0:
				if (fetch_valid) begin
					valid = 1'b1;
					if (req_i && if_valid) begin
						fetch_ready = 1'b1;
						offset_fsm_ns = 1'd0;
					end
				end
			default: offset_fsm_ns = 1'd1;
		endcase
		if (pc_set_i) begin
			valid = 1'b0;
			branch_req = 1'b1;
			offset_fsm_ns = 1'd0;
		end
		else if (hwlp_branch)
			valid = 1'b0;
	end
	riscv_hwloop_controller #(.N_REGS(N_HWLP)) hwloop_controller_i(
		.current_pc_i(fetch_addr),
		.hwlp_jump_o(hwlp_jump),
		.hwlp_targ_addr_o(hwlp_target),
		.hwlp_start_addr_i(hwlp_start_i),
		.hwlp_end_addr_i(hwlp_end_i),
		.hwlp_counter_i(hwlp_cnt_i),
		.hwlp_dec_cnt_o(hwlp_dec_cnt),
		.hwlp_dec_cnt_id_i(hwlp_dec_cnt_id_o & {N_HWLP {is_hwlp_id_o}})
	);
	assign pc_if_o = fetch_addr;
	assign if_busy_o = prefetch_busy;
	assign perf_imiss_o = ~fetch_valid | branch_req;
	wire [31:0] instr_decompressed;
	wire illegal_c_insn;
	wire instr_compressed_int;
	riscv_compressed_decoder #(.FPU(FPU)) compressed_decoder_i(
		.instr_i(fetch_rdata),
		.instr_o(instr_decompressed),
		.is_compressed_o(instr_compressed_int),
		.illegal_instr_o(illegal_c_insn)
	);
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			hwlp_dec_cnt_if <= 1'sb0;
		else if (hwlp_jump)
			hwlp_dec_cnt_if <= hwlp_dec_cnt;
	always @(posedge clk or negedge rst_n) begin : IF_ID_PIPE_REGISTERS
		if (rst_n == 1'b0) begin
			instr_valid_id_o <= 1'b0;
			instr_rdata_id_o <= 1'sb0;
			illegal_c_insn_id_o <= 1'b0;
			is_compressed_id_o <= 1'b0;
			pc_id_o <= 1'sb0;
			is_hwlp_id_q <= 1'b0;
			hwlp_dec_cnt_id_o <= 1'sb0;
			is_fetch_failed_o <= 1'b0;
		end
		else if (if_valid) begin
			instr_valid_id_o <= 1'b1;
			instr_rdata_id_o <= instr_decompressed;
			illegal_c_insn_id_o <= illegal_c_insn;
			is_compressed_id_o <= instr_compressed_int;
			pc_id_o <= pc_if_o;
			is_hwlp_id_q <= fetch_is_hwlp;
			is_fetch_failed_o <= 1'b0;
			if (fetch_is_hwlp)
				hwlp_dec_cnt_id_o <= hwlp_dec_cnt_if;
		end
		else if (clear_instr_valid_i) begin
			instr_valid_id_o <= 1'b0;
			is_fetch_failed_o <= fetch_failed;
		end
	end
	assign is_hwlp_id_o = is_hwlp_id_q & instr_valid_id_o;
	assign if_ready = valid & id_ready_i;
	assign if_valid = ~halt_if_i & if_ready;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_clock_gate (
	clk_i,
	en_i,
	test_en_i,
	clk_o
);
	input wire clk_i;
	input wire en_i;
	input wire test_en_i;
	output wire clk_o;
	assign clk_o = clk_i;
endmodule
module riscv_cs_registers (
	clk,
	rst_n,
	core_id_i,
	cluster_id_i,
	mtvec_o,
	mtvecx_o,
	utvec_o,
	boot_addr_i,
	csr_access_i,
	csr_addr_i,
	csr_wdata_i,
	csr_op_i,
	csr_rdata_o,
	frm_o,
	fprec_o,
	fflags_i,
	fflags_we_i,
	irq_software_i,
	irq_timer_i,
	irq_external_i,
	irq_fast_i,
	irq_nmi_i,
	irq_fastx_i,
	m_irq_enable_o,
	u_irq_enable_o,
	irq_pending_o,
	irq_id_o,
	csr_irq_sec_i,
	sec_lvl_o,
	mepc_o,
	uepc_o,
	debug_mode_i,
	debug_cause_i,
	debug_csr_save_i,
	depc_o,
	debug_single_step_o,
	debug_ebreakm_o,
	debug_ebreaku_o,
	pmp_addr_o,
	pmp_cfg_o,
	priv_lvl_o,
	pc_if_i,
	pc_id_i,
	pc_ex_i,
	csr_save_if_i,
	csr_save_id_i,
	csr_save_ex_i,
	csr_restore_mret_i,
	csr_restore_uret_i,
	csr_restore_dret_i,
	csr_cause_i,
	csr_save_cause_i,
	hwlp_start_i,
	hwlp_end_i,
	hwlp_cnt_i,
	hwlp_data_o,
	hwlp_regid_o,
	hwlp_we_o,
	id_valid_i,
	is_compressed_i,
	is_decoding_i,
	imiss_i,
	pc_set_i,
	jump_i,
	branch_i,
	branch_taken_i,
	ld_stall_i,
	jr_stall_i,
	pipeline_stall_i,
	apu_typeconflict_i,
	apu_contention_i,
	apu_dep_i,
	apu_wb_i,
	mem_load_i,
	mem_store_i,
	ext_counters_i
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter N_HWLP_BITS = $clog2(N_HWLP);
	parameter N_EXT_CNT = 0;
	parameter APU = 0;
	parameter A_EXTENSION = 0;
	parameter FPU = 0;
	parameter PULP_SECURE = 0;
	parameter USE_PMP = 0;
	parameter N_PMP_ENTRIES = 16;
	input wire clk;
	input wire rst_n;
	input wire [3:0] core_id_i;
	input wire [5:0] cluster_id_i;
	output wire [23:0] mtvec_o;
	output wire [23:0] mtvecx_o;
	output wire [23:0] utvec_o;
	input wire [30:0] boot_addr_i;
	input wire csr_access_i;
	input wire [11:0] csr_addr_i;
	input wire [31:0] csr_wdata_i;
	input wire [1:0] csr_op_i;
	output reg [31:0] csr_rdata_o;
	output wire [2:0] frm_o;
	localparam riscv_defines_C_PC = 5;
	output wire [4:0] fprec_o;
	localparam riscv_defines_C_FFLAG = 5;
	input wire [4:0] fflags_i;
	input wire fflags_we_i;
	input wire irq_software_i;
	input wire irq_timer_i;
	input wire irq_external_i;
	input wire [14:0] irq_fast_i;
	input wire irq_nmi_i;
	input wire [31:0] irq_fastx_i;
	output wire m_irq_enable_o;
	output wire u_irq_enable_o;
	output wire irq_pending_o;
	output reg [5:0] irq_id_o;
	input wire csr_irq_sec_i;
	output wire sec_lvl_o;
	output wire [31:0] mepc_o;
	output wire [31:0] uepc_o;
	input wire debug_mode_i;
	input wire [2:0] debug_cause_i;
	input wire debug_csr_save_i;
	output wire [31:0] depc_o;
	output wire debug_single_step_o;
	output wire debug_ebreakm_o;
	output wire debug_ebreaku_o;
	output wire [(N_PMP_ENTRIES * 32) - 1:0] pmp_addr_o;
	output wire [(N_PMP_ENTRIES * 8) - 1:0] pmp_cfg_o;
	output wire [1:0] priv_lvl_o;
	input wire [31:0] pc_if_i;
	input wire [31:0] pc_id_i;
	input wire [31:0] pc_ex_i;
	input wire csr_save_if_i;
	input wire csr_save_id_i;
	input wire csr_save_ex_i;
	input wire csr_restore_mret_i;
	input wire csr_restore_uret_i;
	input wire csr_restore_dret_i;
	input wire [6:0] csr_cause_i;
	input wire csr_save_cause_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_start_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_end_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_cnt_i;
	output wire [31:0] hwlp_data_o;
	output reg [N_HWLP_BITS - 1:0] hwlp_regid_o;
	output reg [2:0] hwlp_we_o;
	input wire id_valid_i;
	input wire is_compressed_i;
	input wire is_decoding_i;
	input wire imiss_i;
	input wire pc_set_i;
	input wire jump_i;
	input wire branch_i;
	input wire branch_taken_i;
	input wire ld_stall_i;
	input wire jr_stall_i;
	input wire pipeline_stall_i;
	input wire apu_typeconflict_i;
	input wire apu_contention_i;
	input wire apu_dep_i;
	input wire apu_wb_i;
	input wire mem_load_i;
	input wire mem_store_i;
	input wire [N_EXT_CNT - 1:0] ext_counters_i;
	localparam N_APU_CNT = (APU == 1 ? 4 : 0);
	localparam N_PERF_COUNTERS = (12 + N_EXT_CNT) + N_APU_CNT;
	localparam PERF_EXT_ID = 12;
	localparam PERF_APU_ID = PERF_EXT_ID + N_EXT_CNT;
	localparam MTVEC_MODE = 2'b01;
	localparam MTVECX_MODE = 2'b01;
	localparam MAX_N_PMP_ENTRIES = 16;
	localparam MAX_N_PMP_CFG = 4;
	localparam N_PMP_CFG = ((N_PMP_ENTRIES % 4) == 0 ? N_PMP_ENTRIES / 4 : (N_PMP_ENTRIES / 4) + 1);
	localparam N_PERF_REGS = N_PERF_COUNTERS;
	localparam [1:0] MXL = 2'd1;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	localparam [31:0] MISA_VALUE = (((((((((((A_EXTENSION << 0) | 4) | 0) | 0) | (FPU << 5)) | 256) | 4096) | 0) | 0) | (PULP_SECURE << 20)) | 8388608) | (sv2v_cast_32(MXL) << 30);
	reg [31:0] csr_wdata_int;
	reg [31:0] csr_rdata_int;
	reg csr_we_int;
	localparam riscv_defines_C_RM = 3;
	reg [2:0] frm_q;
	reg [2:0] frm_n;
	reg [4:0] fflags_q;
	reg [4:0] fflags_n;
	reg [4:0] fprec_q;
	reg [4:0] fprec_n;
	reg [31:0] mepc_q;
	reg [31:0] mepc_n;
	reg [31:0] uepc_q;
	reg [31:0] uepc_n;
	reg [31:0] dcsr_q;
	reg [31:0] dcsr_n;
	reg [31:0] depc_q;
	reg [31:0] depc_n;
	reg [31:0] dscratch0_q;
	reg [31:0] dscratch0_n;
	reg [31:0] dscratch1_q;
	reg [31:0] dscratch1_n;
	reg [31:0] mscratch_q;
	reg [31:0] mscratch_n;
	reg [31:0] exception_pc;
	reg [6:0] mstatus_q;
	reg [6:0] mstatus_n;
	reg [6:0] mcause_q;
	reg [6:0] mcause_n;
	reg [6:0] ucause_q;
	reg [6:0] ucause_n;
	reg [23:0] mtvec_n;
	reg [23:0] mtvec_q;
	reg [23:0] mtvecx_n;
	reg [23:0] mtvecx_q;
	reg [23:0] utvec_n;
	reg [23:0] utvec_q;
	wire [18:0] mip;
	wire [31:0] mipx;
	reg [18:0] mie_q;
	reg [18:0] mie_n;
	reg [31:0] miex_q;
	reg [31:0] miex_n;
	wire [18:0] menip;
	wire [31:0] menipx;
	wire is_irq;
	reg [1:0] priv_lvl_n;
	reg [1:0] priv_lvl_q;
	wire [1:0] priv_lvl_reg_q;
	reg [767:0] pmp_reg_q;
	reg [767:0] pmp_reg_n;
	reg [15:0] pmpaddr_we;
	reg [15:0] pmpcfg_we;
	reg id_valid_q;
	wire [N_PERF_COUNTERS - 1:0] PCCR_in;
	reg [N_PERF_COUNTERS - 1:0] PCCR_inc;
	reg [N_PERF_COUNTERS - 1:0] PCCR_inc_q;
	reg [(N_PERF_REGS * 32) - 1:0] PCCR_q;
	reg [(N_PERF_REGS * 32) - 1:0] PCCR_n;
	reg [1:0] PCMR_n;
	reg [1:0] PCMR_q;
	reg [N_PERF_COUNTERS - 1:0] PCER_n;
	reg [N_PERF_COUNTERS - 1:0] PCER_q;
	reg [31:0] perf_rdata;
	reg [4:0] pccr_index;
	reg pccr_all_sel;
	reg is_pccr;
	reg is_pcer;
	reg is_pcmr;
	wire [18:0] irq_req;
	wire [31:0] irq_reqx;
	assign is_irq = csr_cause_i[6];
	assign irq_req[18] = irq_software_i;
	assign irq_req[17] = irq_timer_i;
	assign irq_req[16] = irq_external_i;
	assign irq_req[15-:15] = irq_fast_i;
	assign irq_req[0] = irq_nmi_i;
	assign irq_reqx = irq_fastx_i;
	assign mip[18] = irq_req[18];
	assign mip[17] = irq_req[17];
	assign mip[16] = irq_req[16];
	assign mip[15-:15] = irq_req[15-:15];
	assign mip[0] = irq_req[0];
	assign mipx = irq_reqx;
	assign menip[18] = irq_req[18] & mie_q[18];
	assign menip[17] = irq_req[17] & mie_q[17];
	assign menip[16] = irq_req[16] & mie_q[16];
	assign menip[15-:15] = irq_req[15-:15] & mie_q[15-:15];
	assign menip[0] = irq_req[0];
	assign menipx = irq_reqx & miex_q;
	genvar _gv_j_1;
	localparam [31:0] riscv_defines_CSR_MEIX_BIT = 11;
	localparam [31:0] riscv_defines_CSR_MFIX_BIT_HIGH = 30;
	localparam [31:0] riscv_defines_CSR_MFIX_BIT_LOW = 16;
	localparam [31:0] riscv_defines_CSR_MSIX_BIT = 3;
	localparam [31:0] riscv_defines_CSR_MTIX_BIT = 7;
	localparam [31:0] riscv_defines_CSR_NMIX_BIT = 31;
	localparam riscv_defines_HWLoop0_COUNTER = 12'h7c2;
	localparam riscv_defines_HWLoop0_END = 12'h7c1;
	localparam riscv_defines_HWLoop0_START = 12'h7c0;
	localparam riscv_defines_HWLoop1_COUNTER = 12'h7c6;
	localparam riscv_defines_HWLoop1_END = 12'h7c5;
	localparam riscv_defines_HWLoop1_START = 12'h7c4;
	generate
		if (PULP_SECURE == 1) begin : genblk1
			always @(*) begin
				if (_sv2v_0)
					;
				case (csr_addr_i)
					12'h001: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fflags_q} : {32 {1'sb0}});
					12'h002: csr_rdata_int = (FPU == 1 ? {29'b00000000000000000000000000000, frm_q} : {32 {1'sb0}});
					12'h003: csr_rdata_int = (FPU == 1 ? {24'b000000000000000000000000, frm_q, fflags_q} : {32 {1'sb0}});
					12'h006: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fprec_q} : {32 {1'sb0}});
					12'h300: csr_rdata_int = {14'b00000000000000, mstatus_q[0], 4'b0000, mstatus_q[2-:2], 3'b000, mstatus_q[3], 2'h0, mstatus_q[4], mstatus_q[5], 2'h0, mstatus_q[6]};
					12'h301: csr_rdata_int = MISA_VALUE;
					12'h304: begin
						csr_rdata_int = 1'sb0;
						csr_rdata_int[riscv_defines_CSR_MSIX_BIT] = mie_q[18];
						csr_rdata_int[riscv_defines_CSR_MTIX_BIT] = mie_q[17];
						csr_rdata_int[riscv_defines_CSR_MEIX_BIT] = mie_q[16];
						csr_rdata_int[riscv_defines_CSR_MFIX_BIT_HIGH:riscv_defines_CSR_MFIX_BIT_LOW] = mie_q[15-:15];
					end
					12'h7d0: csr_rdata_int = miex_q;
					12'h305: csr_rdata_int = {mtvec_q, 6'h00, MTVEC_MODE};
					12'h7d1: csr_rdata_int = {mtvecx_q, 6'h00, MTVECX_MODE};
					12'h340: csr_rdata_int = mscratch_q;
					12'h341: csr_rdata_int = mepc_q;
					12'h342: csr_rdata_int = {mcause_q[6], 25'b0000000000000000000000000, mcause_q[5:0]};
					12'h344: begin
						csr_rdata_int = 1'sb0;
						csr_rdata_int[riscv_defines_CSR_MSIX_BIT] = mip[18];
						csr_rdata_int[riscv_defines_CSR_MTIX_BIT] = mip[17];
						csr_rdata_int[riscv_defines_CSR_MEIX_BIT] = mip[16];
						csr_rdata_int[riscv_defines_CSR_MFIX_BIT_HIGH:riscv_defines_CSR_MFIX_BIT_LOW] = mip[15-:15];
						csr_rdata_int[riscv_defines_CSR_NMIX_BIT] = mip[0];
					end
					12'h7d2: csr_rdata_int = mipx;
					12'hf14: csr_rdata_int = {21'b000000000000000000000, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
					12'h7b0: csr_rdata_int = dcsr_q;
					12'h7b1: csr_rdata_int = depc_q;
					12'h7b2: csr_rdata_int = dscratch0_q;
					12'h7b3: csr_rdata_int = dscratch1_q;
					riscv_defines_HWLoop0_START: csr_rdata_int = hwlp_start_i[0+:32];
					riscv_defines_HWLoop0_END: csr_rdata_int = hwlp_end_i[0+:32];
					riscv_defines_HWLoop0_COUNTER: csr_rdata_int = hwlp_cnt_i[0+:32];
					riscv_defines_HWLoop1_START: csr_rdata_int = hwlp_start_i[32+:32];
					riscv_defines_HWLoop1_END: csr_rdata_int = hwlp_end_i[32+:32];
					riscv_defines_HWLoop1_COUNTER: csr_rdata_int = hwlp_cnt_i[32+:32];
					12'h3a0: csr_rdata_int = (USE_PMP ? pmp_reg_q[128+:32] : {32 {1'sb0}});
					12'h3a1: csr_rdata_int = (USE_PMP ? pmp_reg_q[160+:32] : {32 {1'sb0}});
					12'h3a2: csr_rdata_int = (USE_PMP ? pmp_reg_q[192+:32] : {32 {1'sb0}});
					12'h3a3: csr_rdata_int = (USE_PMP ? pmp_reg_q[224+:32] : {32 {1'sb0}});
					12'h3bx: csr_rdata_int = (USE_PMP ? pmp_reg_q[256 + (csr_addr_i[3:0] * 32)+:32] : {32 {1'sb0}});
					12'h000: csr_rdata_int = {27'b000000000000000000000000000, mstatus_q[4], 3'h0, mstatus_q[6]};
					12'h005: csr_rdata_int = {utvec_q, 6'h00, MTVEC_MODE};
					12'h014: csr_rdata_int = {21'b000000000000000000000, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
					12'h041: csr_rdata_int = uepc_q;
					12'h042: csr_rdata_int = {ucause_q[6], 25'h0000000, ucause_q[5:0]};
					12'hc10: csr_rdata_int = {30'h00000000, priv_lvl_q};
					default: csr_rdata_int = 1'sb0;
				endcase
			end
		end
		else begin : genblk1
			always @(*) begin
				if (_sv2v_0)
					;
				case (csr_addr_i)
					12'h001: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fflags_q} : {32 {1'sb0}});
					12'h002: csr_rdata_int = (FPU == 1 ? {29'b00000000000000000000000000000, frm_q} : {32 {1'sb0}});
					12'h003: csr_rdata_int = (FPU == 1 ? {24'b000000000000000000000000, frm_q, fflags_q} : {32 {1'sb0}});
					12'h006: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fprec_q} : {32 {1'sb0}});
					12'h300: csr_rdata_int = {14'b00000000000000, mstatus_q[0], 4'b0000, mstatus_q[2-:2], 3'b000, mstatus_q[3], 2'h0, mstatus_q[4], mstatus_q[5], 2'h0, mstatus_q[6]};
					12'h301: csr_rdata_int = MISA_VALUE;
					12'h304: begin
						csr_rdata_int = 1'sb0;
						csr_rdata_int[riscv_defines_CSR_MSIX_BIT] = mie_q[18];
						csr_rdata_int[riscv_defines_CSR_MTIX_BIT] = mie_q[17];
						csr_rdata_int[riscv_defines_CSR_MEIX_BIT] = mie_q[16];
						csr_rdata_int[riscv_defines_CSR_MFIX_BIT_HIGH:riscv_defines_CSR_MFIX_BIT_LOW] = mie_q[15-:15];
					end
					12'h7d0: csr_rdata_int = miex_q;
					12'h305: csr_rdata_int = {mtvec_q, 6'h00, MTVEC_MODE};
					12'h7d1: csr_rdata_int = {mtvecx_q, 6'h00, MTVECX_MODE};
					12'h340: csr_rdata_int = mscratch_q;
					12'h341: csr_rdata_int = mepc_q;
					12'h342: csr_rdata_int = {mcause_q[6], 25'b0000000000000000000000000, mcause_q[5:0]};
					12'h344: begin
						csr_rdata_int = 1'sb0;
						csr_rdata_int[riscv_defines_CSR_MSIX_BIT] = mip[18];
						csr_rdata_int[riscv_defines_CSR_MTIX_BIT] = mip[17];
						csr_rdata_int[riscv_defines_CSR_MEIX_BIT] = mip[16];
						csr_rdata_int[riscv_defines_CSR_MFIX_BIT_HIGH:riscv_defines_CSR_MFIX_BIT_LOW] = mip[15-:15];
						csr_rdata_int[riscv_defines_CSR_NMIX_BIT] = mip[0];
					end
					12'h7d2: csr_rdata_int = mipx;
					12'hf14: csr_rdata_int = {21'b000000000000000000000, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
					12'h7b0: csr_rdata_int = dcsr_q;
					12'h7b1: csr_rdata_int = depc_q;
					12'h7b2: csr_rdata_int = dscratch0_q;
					12'h7b3: csr_rdata_int = dscratch1_q;
					riscv_defines_HWLoop0_START: csr_rdata_int = hwlp_start_i[0+:32];
					riscv_defines_HWLoop0_END: csr_rdata_int = hwlp_end_i[0+:32];
					riscv_defines_HWLoop0_COUNTER: csr_rdata_int = hwlp_cnt_i[0+:32];
					riscv_defines_HWLoop1_START: csr_rdata_int = hwlp_start_i[32+:32];
					riscv_defines_HWLoop1_END: csr_rdata_int = hwlp_end_i[32+:32];
					riscv_defines_HWLoop1_COUNTER: csr_rdata_int = hwlp_cnt_i[32+:32];
					12'h014: csr_rdata_int = {21'b000000000000000000000, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
					12'hc10: csr_rdata_int = {30'h00000000, priv_lvl_q};
					default: csr_rdata_int = 1'sb0;
				endcase
			end
		end
	endgenerate
	function automatic [1:0] sv2v_cast_2;
		input reg [1:0] inp;
		sv2v_cast_2 = inp;
	endfunction
	generate
		if (PULP_SECURE == 1) begin : genblk2
			always @(*) begin
				if (_sv2v_0)
					;
				fflags_n = fflags_q;
				frm_n = frm_q;
				fprec_n = fprec_q;
				mscratch_n = mscratch_q;
				mepc_n = mepc_q;
				uepc_n = uepc_q;
				depc_n = depc_q;
				dcsr_n = dcsr_q;
				dscratch0_n = dscratch0_q;
				dscratch1_n = dscratch1_q;
				mstatus_n = mstatus_q;
				mcause_n = mcause_q;
				ucause_n = ucause_q;
				hwlp_we_o = 1'sb0;
				hwlp_regid_o = 1'sb0;
				exception_pc = pc_id_i;
				priv_lvl_n = priv_lvl_q;
				mtvec_n = mtvec_q;
				utvec_n = utvec_q;
				pmp_reg_n[767-:512] = pmp_reg_q[767-:512];
				pmp_reg_n[255-:128] = pmp_reg_q[255-:128];
				pmpaddr_we = 1'sb0;
				pmpcfg_we = 1'sb0;
				mie_n = mie_q;
				miex_n = miex_q;
				mtvecx_n = mtvecx_q;
				if (FPU == 1) begin
					if (fflags_we_i)
						fflags_n = fflags_i | fflags_q;
				end
				casex (csr_addr_i)
					12'h001:
						if (csr_we_int)
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h002:
						if (csr_we_int)
							frm_n = (FPU == 1 ? csr_wdata_int[2:0] : {3 {1'sb0}});
					12'h003:
						if (csr_we_int) begin
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
							frm_n = (FPU == 1 ? csr_wdata_int[7:riscv_defines_C_FFLAG] : {3 {1'sb0}});
						end
					12'h006:
						if (csr_we_int)
							fprec_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h300:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[0], csr_wdata_int[3], csr_wdata_int[4], csr_wdata_int[7], csr_wdata_int[12:11], csr_wdata_int[17]};
					12'h304:
						if (csr_we_int) begin
							mie_n[18] = csr_wdata_int[riscv_defines_CSR_MSIX_BIT];
							mie_n[17] = csr_wdata_int[riscv_defines_CSR_MTIX_BIT];
							mie_n[16] = csr_wdata_int[riscv_defines_CSR_MEIX_BIT];
							mie_n[15-:15] = csr_wdata_int[riscv_defines_CSR_MFIX_BIT_HIGH:riscv_defines_CSR_MFIX_BIT_LOW];
						end
					12'h7d0:
						if (csr_we_int)
							miex_n = csr_wdata_int;
					12'h305:
						if (csr_we_int)
							mtvec_n = csr_wdata_int[31:8];
					12'h7d1:
						if (csr_we_int)
							mtvecx_n = csr_wdata_int[31:8];
					12'h340:
						if (csr_we_int)
							mscratch_n = csr_wdata_int;
					12'h341:
						if (csr_we_int)
							mepc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h342:
						if (csr_we_int)
							mcause_n = {csr_wdata_int[31], csr_wdata_int[5:0]};
					12'h7b0:
						if (csr_we_int) begin
							dcsr_n = csr_wdata_int;
							dcsr_n[31-:4] = 4'h4;
							dcsr_n[1-:2] = priv_lvl_q;
							dcsr_n[3] = 1'b0;
							dcsr_n[4] = 1'b0;
							dcsr_n[10] = 1'b0;
							dcsr_n[9] = 1'b0;
						end
					12'h7b1:
						if (csr_we_int)
							depc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h7b2:
						if (csr_we_int)
							dscratch0_n = csr_wdata_int;
					12'h7b3:
						if (csr_we_int)
							dscratch1_n = csr_wdata_int;
					riscv_defines_HWLoop0_START:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b0;
						end
					riscv_defines_HWLoop0_END:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b0;
						end
					riscv_defines_HWLoop0_COUNTER:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b0;
						end
					riscv_defines_HWLoop1_START:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b1;
						end
					riscv_defines_HWLoop1_END:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b1;
						end
					riscv_defines_HWLoop1_COUNTER:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b1;
						end
					12'h3a0:
						if (csr_we_int) begin
							pmp_reg_n[128+:32] = csr_wdata_int;
							pmpcfg_we[3:0] = 4'b1111;
						end
					12'h3a1:
						if (csr_we_int) begin
							pmp_reg_n[160+:32] = csr_wdata_int;
							pmpcfg_we[7:4] = 4'b1111;
						end
					12'h3a2:
						if (csr_we_int) begin
							pmp_reg_n[192+:32] = csr_wdata_int;
							pmpcfg_we[11:8] = 4'b1111;
						end
					12'h3a3:
						if (csr_we_int) begin
							pmp_reg_n[224+:32] = csr_wdata_int;
							pmpcfg_we[15:12] = 4'b1111;
						end
					12'h3bx:
						if (csr_we_int) begin
							pmp_reg_n[256 + (csr_addr_i[3:0] * 32)+:32] = csr_wdata_int;
							pmpaddr_we[csr_addr_i[3:0]] = 1'b1;
						end
					12'h000:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[0], mstatus_q[5], csr_wdata_int[4], mstatus_q[3], sv2v_cast_2(mstatus_q[2-:2]), mstatus_q[0]};
					12'h005:
						if (csr_we_int)
							utvec_n = csr_wdata_int[31:8];
					12'h041:
						if (csr_we_int)
							uepc_n = csr_wdata_int;
					12'h042:
						if (csr_we_int)
							ucause_n = {csr_wdata_int[31], csr_wdata_int[5:0]};
				endcase
				case (1'b1)
					csr_save_cause_i: begin
						case (1'b1)
							csr_save_if_i: exception_pc = pc_if_i;
							csr_save_id_i: exception_pc = pc_id_i;
							csr_save_ex_i: exception_pc = pc_ex_i;
							default:
								;
						endcase
						case (priv_lvl_q)
							2'b00:
								if (~is_irq) begin
									priv_lvl_n = 2'b11;
									mstatus_n[3] = mstatus_q[6];
									mstatus_n[5] = 1'b0;
									mstatus_n[2-:2] = 2'b00;
									if (debug_csr_save_i)
										depc_n = exception_pc;
									else
										mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
								else if (~csr_irq_sec_i) begin
									priv_lvl_n = 2'b00;
									mstatus_n[4] = mstatus_q[6];
									mstatus_n[6] = 1'b0;
									if (debug_csr_save_i)
										depc_n = exception_pc;
									else
										uepc_n = exception_pc;
									ucause_n = csr_cause_i;
								end
								else begin
									priv_lvl_n = 2'b11;
									mstatus_n[3] = mstatus_q[6];
									mstatus_n[5] = 1'b0;
									mstatus_n[2-:2] = 2'b00;
									if (debug_csr_save_i)
										depc_n = exception_pc;
									else
										mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
							2'b11:
								if (debug_csr_save_i) begin
									dcsr_n[1-:2] = 2'b11;
									dcsr_n[8-:3] = debug_cause_i;
									depc_n = exception_pc;
								end
								else begin
									priv_lvl_n = 2'b11;
									mstatus_n[3] = mstatus_q[5];
									mstatus_n[5] = 1'b0;
									mstatus_n[2-:2] = 2'b11;
									mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
							default:
								;
						endcase
					end
					csr_restore_uret_i: begin
						mstatus_n[6] = mstatus_q[4];
						priv_lvl_n = 2'b00;
						mstatus_n[4] = 1'b1;
					end
					csr_restore_mret_i:
						case (mstatus_q[2-:2])
							2'b00: begin
								mstatus_n[6] = mstatus_q[3];
								priv_lvl_n = 2'b00;
								mstatus_n[3] = 1'b1;
								mstatus_n[2-:2] = 2'b00;
							end
							2'b11: begin
								mstatus_n[5] = mstatus_q[3];
								priv_lvl_n = 2'b11;
								mstatus_n[3] = 1'b1;
								mstatus_n[2-:2] = 2'b00;
							end
							default:
								;
						endcase
					csr_restore_dret_i: priv_lvl_n = dcsr_q[1-:2];
					default:
						;
				endcase
			end
		end
		else begin : genblk2
			always @(*) begin
				if (_sv2v_0)
					;
				fflags_n = fflags_q;
				frm_n = frm_q;
				fprec_n = fprec_q;
				mscratch_n = mscratch_q;
				mepc_n = mepc_q;
				depc_n = depc_q;
				dcsr_n = dcsr_q;
				dscratch0_n = dscratch0_q;
				dscratch1_n = dscratch1_q;
				mstatus_n = mstatus_q;
				mcause_n = mcause_q;
				hwlp_we_o = 1'sb0;
				hwlp_regid_o = 1'sb0;
				exception_pc = pc_id_i;
				priv_lvl_n = priv_lvl_q;
				mtvec_n = mtvec_q;
				pmp_reg_n[767-:512] = pmp_reg_q[767-:512];
				pmp_reg_n[255-:128] = pmp_reg_q[255-:128];
				pmpaddr_we = 1'sb0;
				pmpcfg_we = 1'sb0;
				mie_n = mie_q;
				miex_n = miex_q;
				mtvecx_n = mtvecx_q;
				if (FPU == 1) begin
					if (fflags_we_i)
						fflags_n = fflags_i | fflags_q;
				end
				case (csr_addr_i)
					12'h001:
						if (csr_we_int)
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h002:
						if (csr_we_int)
							frm_n = (FPU == 1 ? csr_wdata_int[2:0] : {3 {1'sb0}});
					12'h003:
						if (csr_we_int) begin
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
							frm_n = (FPU == 1 ? csr_wdata_int[7:riscv_defines_C_FFLAG] : {3 {1'sb0}});
						end
					12'h006:
						if (csr_we_int)
							fprec_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h300:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[0], csr_wdata_int[3], csr_wdata_int[4], csr_wdata_int[7], csr_wdata_int[12:11], csr_wdata_int[17]};
					12'h304:
						if (csr_we_int) begin
							mie_n[18] = csr_wdata_int[riscv_defines_CSR_MSIX_BIT];
							mie_n[17] = csr_wdata_int[riscv_defines_CSR_MTIX_BIT];
							mie_n[16] = csr_wdata_int[riscv_defines_CSR_MEIX_BIT];
							mie_n[15-:15] = csr_wdata_int[riscv_defines_CSR_MFIX_BIT_HIGH:riscv_defines_CSR_MFIX_BIT_LOW];
						end
					12'h7d0:
						if (csr_we_int)
							miex_n = csr_wdata_int;
					12'h305:
						if (csr_we_int)
							mtvec_n = csr_wdata_int[31:8];
					12'h7d1:
						if (csr_we_int)
							mtvecx_n = csr_wdata_int[31:8];
					12'h340:
						if (csr_we_int)
							mscratch_n = csr_wdata_int;
					12'h341:
						if (csr_we_int)
							mepc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h342:
						if (csr_we_int)
							mcause_n = {csr_wdata_int[31], csr_wdata_int[5:0]};
					12'h7b0:
						if (csr_we_int) begin
							dcsr_n = csr_wdata_int;
							dcsr_n[31-:4] = 4'h4;
							dcsr_n[1-:2] = priv_lvl_q;
							dcsr_n[3] = 1'b0;
							dcsr_n[4] = 1'b0;
							dcsr_n[10] = 1'b0;
							dcsr_n[9] = 1'b0;
						end
					12'h7b1:
						if (csr_we_int)
							depc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h7b2:
						if (csr_we_int)
							dscratch0_n = csr_wdata_int;
					12'h7b3:
						if (csr_we_int)
							dscratch1_n = csr_wdata_int;
					riscv_defines_HWLoop0_START:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b0;
						end
					riscv_defines_HWLoop0_END:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b0;
						end
					riscv_defines_HWLoop0_COUNTER:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b0;
						end
					riscv_defines_HWLoop1_START:
						if (csr_we_int) begin
							hwlp_we_o = 3'b001;
							hwlp_regid_o = 1'b1;
						end
					riscv_defines_HWLoop1_END:
						if (csr_we_int) begin
							hwlp_we_o = 3'b010;
							hwlp_regid_o = 1'b1;
						end
					riscv_defines_HWLoop1_COUNTER:
						if (csr_we_int) begin
							hwlp_we_o = 3'b100;
							hwlp_regid_o = 1'b1;
						end
				endcase
				case (1'b1)
					csr_save_cause_i: begin
						case (1'b1)
							csr_save_if_i: exception_pc = pc_if_i;
							csr_save_id_i: exception_pc = pc_id_i;
							default:
								;
						endcase
						if (debug_csr_save_i) begin
							dcsr_n[1-:2] = 2'b11;
							dcsr_n[8-:3] = debug_cause_i;
							depc_n = exception_pc;
						end
						else begin
							priv_lvl_n = 2'b11;
							mstatus_n[3] = mstatus_q[5];
							mstatus_n[5] = 1'b0;
							mstatus_n[2-:2] = 2'b11;
							mepc_n = exception_pc;
							mcause_n = csr_cause_i;
						end
					end
					csr_restore_mret_i: begin
						mstatus_n[5] = mstatus_q[3];
						priv_lvl_n = 2'b11;
						mstatus_n[3] = 1'b1;
						mstatus_n[2-:2] = 2'b11;
					end
					csr_restore_dret_i: priv_lvl_n = dcsr_q[1-:2];
					default:
						;
				endcase
			end
		end
	endgenerate
	assign hwlp_data_o = csr_wdata_int;
	localparam riscv_defines_CSR_OP_CLEAR = 2'b11;
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_SET = 2'b10;
	localparam riscv_defines_CSR_OP_WRITE = 2'b01;
	always @(*) begin
		if (_sv2v_0)
			;
		csr_wdata_int = csr_wdata_i;
		csr_we_int = 1'b1;
		case (csr_op_i)
			riscv_defines_CSR_OP_WRITE: csr_wdata_int = csr_wdata_i;
			riscv_defines_CSR_OP_SET: csr_wdata_int = csr_wdata_i | csr_rdata_o;
			riscv_defines_CSR_OP_CLEAR: csr_wdata_int = ~csr_wdata_i & csr_rdata_o;
			riscv_defines_CSR_OP_NONE: begin
				csr_wdata_int = csr_wdata_i;
				csr_we_int = 1'b0;
			end
			default:
				;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		csr_rdata_o = csr_rdata_int;
		if ((is_pccr || is_pcer) || is_pcmr)
			csr_rdata_o = perf_rdata;
	end
	always @(*) begin
		if (_sv2v_0)
			;
		if (menip[0])
			irq_id_o = 6'd31;
		else if (menipx[31])
			irq_id_o = 6'd63;
		else if (menipx[30])
			irq_id_o = 6'd62;
		else if (menipx[29])
			irq_id_o = 6'd61;
		else if (menipx[28])
			irq_id_o = 6'd60;
		else if (menipx[27])
			irq_id_o = 6'd59;
		else if (menipx[26])
			irq_id_o = 6'd58;
		else if (menipx[25])
			irq_id_o = 6'd57;
		else if (menipx[24])
			irq_id_o = 6'd56;
		else if (menipx[23])
			irq_id_o = 6'd55;
		else if (menipx[22])
			irq_id_o = 6'd54;
		else if (menipx[21])
			irq_id_o = 6'd53;
		else if (menipx[20])
			irq_id_o = 6'd52;
		else if (menipx[19])
			irq_id_o = 6'd51;
		else if (menipx[18])
			irq_id_o = 6'd50;
		else if (menipx[17])
			irq_id_o = 6'd49;
		else if (menipx[16])
			irq_id_o = 6'd48;
		else if (menipx[15])
			irq_id_o = 6'd47;
		else if (menipx[14])
			irq_id_o = 6'd46;
		else if (menipx[13])
			irq_id_o = 6'd45;
		else if (menipx[12])
			irq_id_o = 6'd44;
		else if (menipx[11])
			irq_id_o = 6'd43;
		else if (menipx[10])
			irq_id_o = 6'd42;
		else if (menipx[9])
			irq_id_o = 6'd41;
		else if (menipx[8])
			irq_id_o = 6'd40;
		else if (menipx[7])
			irq_id_o = 6'd39;
		else if (menipx[6])
			irq_id_o = 6'd38;
		else if (menipx[5])
			irq_id_o = 6'd37;
		else if (menipx[4])
			irq_id_o = 6'd36;
		else if (menipx[3])
			irq_id_o = 6'd35;
		else if (menipx[2])
			irq_id_o = 6'd34;
		else if (menipx[1])
			irq_id_o = 6'd33;
		else if (menipx[0])
			irq_id_o = 6'd32;
		else if (menip[15])
			irq_id_o = 6'd30;
		else if (menip[14])
			irq_id_o = 6'd29;
		else if (menip[13])
			irq_id_o = 6'd28;
		else if (menip[12])
			irq_id_o = 6'd27;
		else if (menip[11])
			irq_id_o = 6'd26;
		else if (menip[10])
			irq_id_o = 6'd25;
		else if (menip[9])
			irq_id_o = 6'd24;
		else if (menip[8])
			irq_id_o = 6'd23;
		else if (menip[7])
			irq_id_o = 6'd22;
		else if (menip[6])
			irq_id_o = 6'd21;
		else if (menip[5])
			irq_id_o = 6'd20;
		else if (menip[4])
			irq_id_o = 6'd19;
		else if (menip[3])
			irq_id_o = 6'd18;
		else if (menip[2])
			irq_id_o = 6'd17;
		else if (menip[1])
			irq_id_o = 6'd16;
		else if (menip[16])
			irq_id_o = riscv_defines_CSR_MEIX_BIT;
		else if (menip[18])
			irq_id_o = riscv_defines_CSR_MSIX_BIT;
		else if (menip[17])
			irq_id_o = riscv_defines_CSR_MTIX_BIT;
		else
			irq_id_o = riscv_defines_CSR_MTIX_BIT;
	end
	assign m_irq_enable_o = mstatus_q[5] & (priv_lvl_q == 2'b11);
	assign u_irq_enable_o = mstatus_q[6] & (priv_lvl_q == 2'b00);
	assign priv_lvl_o = priv_lvl_q;
	assign sec_lvl_o = priv_lvl_q[0];
	assign frm_o = (FPU == 1 ? frm_q : {3 {1'sb0}});
	assign fprec_o = (FPU == 1 ? fprec_q : {5 {1'sb0}});
	assign mtvec_o = mtvec_q;
	assign mtvecx_o = mtvecx_q;
	assign utvec_o = utvec_q;
	assign mepc_o = mepc_q;
	assign uepc_o = uepc_q;
	assign depc_o = depc_q;
	assign pmp_addr_o = pmp_reg_q[767-:512];
	assign pmp_cfg_o = pmp_reg_q[127-:128];
	assign debug_single_step_o = dcsr_q[2];
	assign debug_ebreakm_o = dcsr_q[15];
	assign debug_ebreaku_o = dcsr_q[12];
	assign irq_pending_o = ((((menip[18] | menip[17]) | menip[16]) | (|menip[15-:15])) | menip[0]) | (|menipx);
	generate
		if (PULP_SECURE == 1) begin : genblk3
			for (_gv_j_1 = 0; _gv_j_1 < N_PMP_ENTRIES; _gv_j_1 = _gv_j_1 + 1) begin : CS_PMP_CFG
				localparam j = _gv_j_1;
				wire [8:1] sv2v_tmp_C98C8;
				assign sv2v_tmp_C98C8 = pmp_reg_n[128 + (((j / 4) * 32) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (8 * ((j % 4) + 1)) - 1 : (((8 * ((j % 4) + 1)) - 1) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)) - 1))-:(((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)];
				always @(*) pmp_reg_n[0 + (j * 8)+:8] = sv2v_tmp_C98C8;
				wire [(((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1) * 1:1] sv2v_tmp_864C8;
				assign sv2v_tmp_864C8 = pmp_reg_q[0 + (j * 8)+:8];
				always @(*) pmp_reg_q[128 + (((j / 4) * 32) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (8 * ((j % 4) + 1)) - 1 : (((8 * ((j % 4) + 1)) - 1) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)) - 1))-:(((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)] = sv2v_tmp_864C8;
			end
			for (_gv_j_1 = 0; _gv_j_1 < N_PMP_ENTRIES; _gv_j_1 = _gv_j_1 + 1) begin : CS_PMP_REGS_FF
				localparam j = _gv_j_1;
				always @(posedge clk or negedge rst_n)
					if (rst_n == 1'b0) begin
						pmp_reg_q[0 + (j * 8)+:8] <= 1'sb0;
						pmp_reg_q[256 + (j * 32)+:32] <= 1'sb0;
					end
					else begin
						if (pmpcfg_we[j])
							pmp_reg_q[0 + (j * 8)+:8] <= (USE_PMP ? pmp_reg_n[0 + (j * 8)+:8] : {8 {1'sb0}});
						if (pmpaddr_we[j])
							pmp_reg_q[256 + (j * 32)+:32] <= (USE_PMP ? pmp_reg_n[256 + (j * 32)+:32] : {32 {1'sb0}});
					end
			end
			always @(posedge clk or negedge rst_n)
				if (rst_n == 1'b0) begin
					uepc_q <= 1'sb0;
					ucause_q <= 1'sb0;
					utvec_q <= 1'sb0;
					priv_lvl_q <= 2'b11;
				end
				else begin
					uepc_q <= uepc_n;
					ucause_q <= ucause_n;
					utvec_q <= utvec_n;
					priv_lvl_q <= priv_lvl_n;
				end
		end
		else begin : genblk3
			wire [32:1] sv2v_tmp_6C649;
			assign sv2v_tmp_6C649 = 1'sb0;
			always @(*) uepc_q = sv2v_tmp_6C649;
			wire [7:1] sv2v_tmp_1A9D8;
			assign sv2v_tmp_1A9D8 = 1'sb0;
			always @(*) ucause_q = sv2v_tmp_1A9D8;
			wire [24:1] sv2v_tmp_CC076;
			assign sv2v_tmp_CC076 = 1'sb0;
			always @(*) utvec_q = sv2v_tmp_CC076;
			wire [2:1] sv2v_tmp_16422;
			assign sv2v_tmp_16422 = 2'b11;
			always @(*) priv_lvl_q = sv2v_tmp_16422;
		end
	endgenerate
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			if (FPU == 1) begin
				frm_q <= 1'sb0;
				fflags_q <= 1'sb0;
				fprec_q <= 1'sb0;
			end
			mstatus_q <= 7'b0000110;
			mepc_q <= 1'sb0;
			mcause_q <= 1'sb0;
			depc_q <= 1'sb0;
			dcsr_q <= 1'sb0;
			dcsr_q[1-:2] <= 2'b11;
			dscratch0_q <= 1'sb0;
			dscratch1_q <= 1'sb0;
			mscratch_q <= 1'sb0;
			mie_q <= 1'sb0;
			miex_q <= 1'sb0;
			mtvec_q <= 1'sb0;
			mtvecx_q <= 1'sb0;
		end
		else begin
			if (FPU == 1) begin
				frm_q <= frm_n;
				fflags_q <= fflags_n;
				fprec_q <= fprec_n;
			end
			if (PULP_SECURE == 1)
				mstatus_q <= mstatus_n;
			else
				mstatus_q <= {1'b0, mstatus_n[5], 1'b0, mstatus_n[3], 3'b110};
			mepc_q <= mepc_n;
			mcause_q <= mcause_n;
			depc_q <= depc_n;
			dcsr_q <= dcsr_n;
			dscratch0_q <= dscratch0_n;
			dscratch1_q <= dscratch1_n;
			mscratch_q <= mscratch_n;
			mie_q <= mie_n;
			miex_q <= miex_n;
			mtvec_q <= mtvec_n;
			mtvecx_q <= mtvecx_n;
		end
	assign PCCR_in[0] = 1'b1;
	assign PCCR_in[1] = id_valid_i & is_decoding_i;
	assign PCCR_in[2] = ld_stall_i & id_valid_q;
	assign PCCR_in[3] = jr_stall_i & id_valid_q;
	assign PCCR_in[4] = imiss_i & ~pc_set_i;
	assign PCCR_in[5] = mem_load_i;
	assign PCCR_in[6] = mem_store_i;
	assign PCCR_in[7] = jump_i & id_valid_q;
	assign PCCR_in[8] = branch_i & id_valid_q;
	assign PCCR_in[9] = (branch_i & branch_taken_i) & id_valid_q;
	assign PCCR_in[10] = (id_valid_i & is_decoding_i) & is_compressed_i;
	assign PCCR_in[11] = pipeline_stall_i;
	generate
		if (APU == 1) begin : genblk4
			assign PCCR_in[PERF_APU_ID] = apu_typeconflict_i & ~apu_dep_i;
			assign PCCR_in[PERF_APU_ID + 1] = apu_contention_i;
			assign PCCR_in[PERF_APU_ID + 2] = apu_dep_i & ~apu_contention_i;
			assign PCCR_in[PERF_APU_ID + 3] = apu_wb_i;
		end
	endgenerate
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < N_EXT_CNT; _gv_i_1 = _gv_i_1 + 1) begin : genblk5
			localparam i = _gv_i_1;
			assign PCCR_in[PERF_EXT_ID + i] = ext_counters_i[i];
		end
	endgenerate
	localparam riscv_defines_PCER_MACHINE = 12'h7e0;
	localparam riscv_defines_PCER_USER = 12'hcc0;
	localparam riscv_defines_PCMR_MACHINE = 12'h7e1;
	localparam riscv_defines_PCMR_USER = 12'hcc1;
	always @(*) begin
		if (_sv2v_0)
			;
		is_pccr = 1'b0;
		is_pcmr = 1'b0;
		is_pcer = 1'b0;
		pccr_all_sel = 1'b0;
		pccr_index = 1'sb0;
		perf_rdata = 1'sb0;
		if (csr_access_i) begin
			case (csr_addr_i)
				riscv_defines_PCER_USER, riscv_defines_PCER_MACHINE: begin
					is_pcer = 1'b1;
					perf_rdata[N_PERF_COUNTERS - 1:0] = PCER_q;
				end
				riscv_defines_PCMR_USER, riscv_defines_PCMR_MACHINE: begin
					is_pcmr = 1'b1;
					perf_rdata[1:0] = PCMR_q;
				end
				12'h79f: begin
					is_pccr = 1'b1;
					pccr_all_sel = 1'b1;
				end
				default:
					;
			endcase
			if (csr_addr_i[11:5] == 7'b0111100) begin
				is_pccr = 1'b1;
				pccr_index = csr_addr_i[4:0];
				perf_rdata = (csr_addr_i[4:0] < N_PERF_COUNTERS ? PCCR_q[csr_addr_i[4:0] * 32+:32] : {32 {1'sb0}});
			end
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < N_PERF_COUNTERS; i = i + 1)
				begin : PERF_CNT_INC
					PCCR_inc[i] = (PCCR_in[i] & PCER_q[i]) & PCMR_q[0];
					PCCR_n[i * 32+:32] = PCCR_q[i * 32+:32];
					if ((PCCR_inc_q[i] == 1'b1) && ((PCCR_q[i * 32+:32] != 32'hffffffff) || (PCMR_q[1] == 1'b0)))
						PCCR_n[i * 32+:32] = PCCR_q[i * 32+:32] + 1;
					if ((is_pccr == 1'b1) && ((pccr_all_sel == 1'b1) || (pccr_index == i)))
						case (csr_op_i)
							riscv_defines_CSR_OP_NONE:
								;
							riscv_defines_CSR_OP_WRITE: PCCR_n[i * 32+:32] = csr_wdata_i;
							riscv_defines_CSR_OP_SET: PCCR_n[i * 32+:32] = csr_wdata_i | PCCR_q[i * 32+:32];
							riscv_defines_CSR_OP_CLEAR: PCCR_n[i * 32+:32] = csr_wdata_i & ~PCCR_q[i * 32+:32];
						endcase
				end
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		PCMR_n = PCMR_q;
		PCER_n = PCER_q;
		if (is_pcmr)
			case (csr_op_i)
				riscv_defines_CSR_OP_NONE:
					;
				riscv_defines_CSR_OP_WRITE: PCMR_n = csr_wdata_i[1:0];
				riscv_defines_CSR_OP_SET: PCMR_n = csr_wdata_i[1:0] | PCMR_q;
				riscv_defines_CSR_OP_CLEAR: PCMR_n = csr_wdata_i[1:0] & ~PCMR_q;
			endcase
		if (is_pcer)
			case (csr_op_i)
				riscv_defines_CSR_OP_NONE:
					;
				riscv_defines_CSR_OP_WRITE: PCER_n = csr_wdata_i[N_PERF_COUNTERS - 1:0];
				riscv_defines_CSR_OP_SET: PCER_n = csr_wdata_i[N_PERF_COUNTERS - 1:0] | PCER_q;
				riscv_defines_CSR_OP_CLEAR: PCER_n = csr_wdata_i[N_PERF_COUNTERS - 1:0] & ~PCER_q;
			endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			id_valid_q <= 1'b0;
			PCER_q <= 1'sb0;
			PCMR_q <= 2'h3;
			begin : sv2v_autoblock_2
				reg signed [31:0] i;
				for (i = 0; i < N_PERF_REGS; i = i + 1)
					begin
						PCCR_q[i * 32+:32] <= 1'sb0;
						PCCR_inc_q[i] <= 1'sb0;
					end
			end
		end
		else begin
			id_valid_q <= id_valid_i;
			PCER_q <= PCER_n;
			PCMR_q <= PCMR_n;
			begin : sv2v_autoblock_3
				reg signed [31:0] i;
				for (i = 0; i < N_PERF_REGS; i = i + 1)
					begin
						PCCR_q[i * 32+:32] <= PCCR_n[i * 32+:32];
						PCCR_inc_q[i] <= PCCR_inc[i];
					end
			end
		end
	initial _sv2v_0 = 0;
endmodule
module riscv_register_file (
	clk,
	rst_n,
	test_en_i,
	raddr_a_i,
	rdata_a_o,
	raddr_b_i,
	rdata_b_o,
	raddr_c_i,
	rdata_c_o,
	waddr_a_i,
	wdata_a_i,
	we_a_i,
	waddr_b_i,
	wdata_b_i,
	we_b_i
);
	reg _sv2v_0;
	parameter ADDR_WIDTH = 5;
	parameter DATA_WIDTH = 32;
	parameter FPU = 0;
	parameter Zfinx = 0;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire [ADDR_WIDTH - 1:0] raddr_a_i;
	output wire [DATA_WIDTH - 1:0] rdata_a_o;
	input wire [ADDR_WIDTH - 1:0] raddr_b_i;
	output wire [DATA_WIDTH - 1:0] rdata_b_o;
	input wire [ADDR_WIDTH - 1:0] raddr_c_i;
	output wire [DATA_WIDTH - 1:0] rdata_c_o;
	input wire [ADDR_WIDTH - 1:0] waddr_a_i;
	input wire [DATA_WIDTH - 1:0] wdata_a_i;
	input wire we_a_i;
	input wire [ADDR_WIDTH - 1:0] waddr_b_i;
	input wire [DATA_WIDTH - 1:0] wdata_b_i;
	input wire we_b_i;
	localparam NUM_WORDS = 2 ** (ADDR_WIDTH - 1);
	localparam NUM_FP_WORDS = 2 ** (ADDR_WIDTH - 1);
	localparam NUM_TOT_WORDS = (FPU ? (Zfinx ? NUM_WORDS : NUM_WORDS + NUM_FP_WORDS) : NUM_WORDS);
	reg [(NUM_WORDS * DATA_WIDTH) - 1:0] mem;
	reg [(NUM_FP_WORDS * DATA_WIDTH) - 1:0] mem_fp;
	wire [ADDR_WIDTH - 1:0] waddr_a;
	wire [ADDR_WIDTH - 1:0] waddr_b;
	reg [NUM_TOT_WORDS - 1:0] we_a_dec;
	reg [NUM_TOT_WORDS - 1:0] we_b_dec;
	generate
		if ((FPU == 1) && (Zfinx == 0)) begin : genblk1
			assign rdata_a_o = (raddr_a_i[5] ? mem_fp[raddr_a_i[4:0] * DATA_WIDTH+:DATA_WIDTH] : mem[raddr_a_i[4:0] * DATA_WIDTH+:DATA_WIDTH]);
			assign rdata_b_o = (raddr_b_i[5] ? mem_fp[raddr_b_i[4:0] * DATA_WIDTH+:DATA_WIDTH] : mem[raddr_b_i[4:0] * DATA_WIDTH+:DATA_WIDTH]);
			assign rdata_c_o = (raddr_c_i[5] ? mem_fp[raddr_c_i[4:0] * DATA_WIDTH+:DATA_WIDTH] : mem[raddr_c_i[4:0] * DATA_WIDTH+:DATA_WIDTH]);
		end
		else begin : genblk1
			assign rdata_a_o = mem[raddr_a_i[4:0] * DATA_WIDTH+:DATA_WIDTH];
			assign rdata_b_o = mem[raddr_b_i[4:0] * DATA_WIDTH+:DATA_WIDTH];
			assign rdata_c_o = mem[raddr_c_i[4:0] * DATA_WIDTH+:DATA_WIDTH];
		end
	endgenerate
	assign waddr_a = waddr_a_i;
	assign waddr_b = waddr_b_i;
	always @(*) begin : we_a_decoder
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < NUM_TOT_WORDS; i = i + 1)
				if (waddr_a == i)
					we_a_dec[i] = we_a_i;
				else
					we_a_dec[i] = 1'b0;
		end
	end
	always @(*) begin : we_b_decoder
		if (_sv2v_0)
			;
		begin : sv2v_autoblock_2
			reg signed [31:0] i;
			for (i = 0; i < NUM_TOT_WORDS; i = i + 1)
				if (waddr_b == i)
					we_b_dec[i] = we_b_i;
				else
					we_b_dec[i] = 1'b0;
		end
	end
	genvar _gv_i_2;
	genvar _gv_l_1;
	always @(posedge clk or negedge rst_n)
		if (~rst_n)
			mem[0+:DATA_WIDTH] <= 32'b00000000000000000000000000000000;
		else
			mem[0+:DATA_WIDTH] <= 32'b00000000000000000000000000000000;
	generate
		for (_gv_i_2 = 1; _gv_i_2 < NUM_WORDS; _gv_i_2 = _gv_i_2 + 1) begin : rf_gen
			localparam i = _gv_i_2;
			always @(posedge clk or negedge rst_n) begin : register_write_behavioral
				if (rst_n == 1'b0)
					mem[i * DATA_WIDTH+:DATA_WIDTH] <= 32'b00000000000000000000000000000000;
				else if (we_b_dec[i] == 1'b1)
					mem[i * DATA_WIDTH+:DATA_WIDTH] <= wdata_b_i;
				else if (we_a_dec[i] == 1'b1)
					mem[i * DATA_WIDTH+:DATA_WIDTH] <= wdata_a_i;
			end
		end
		if ((FPU == 1) && (Zfinx == 0)) begin : genblk3
			for (_gv_l_1 = 0; _gv_l_1 < NUM_FP_WORDS; _gv_l_1 = _gv_l_1 + 1) begin : genblk1
				localparam l = _gv_l_1;
				always @(posedge clk or negedge rst_n) begin : fp_regs
					if (rst_n == 1'b0)
						mem_fp[l * DATA_WIDTH+:DATA_WIDTH] <= 1'sb0;
					else if (we_b_dec[l + NUM_WORDS] == 1'b1)
						mem_fp[l * DATA_WIDTH+:DATA_WIDTH] <= wdata_b_i;
					else if (we_a_dec[l + NUM_WORDS] == 1'b1)
						mem_fp[l * DATA_WIDTH+:DATA_WIDTH] <= wdata_a_i;
				end
			end
		end
	endgenerate
	initial _sv2v_0 = 0;
endmodule
module riscv_load_store_unit (
	clk,
	rst_n,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_err_i,
	data_addr_o,
	data_we_o,
	data_be_o,
	data_wdata_o,
	data_rdata_i,
	data_we_ex_i,
	data_type_ex_i,
	data_wdata_ex_i,
	data_reg_offset_ex_i,
	data_sign_ext_ex_i,
	data_rdata_ex_o,
	data_req_ex_i,
	operand_a_ex_i,
	operand_b_ex_i,
	addr_useincr_ex_i,
	data_misaligned_ex_i,
	data_misaligned_o,
	data_atop_ex_i,
	data_atop_o,
	lsu_ready_ex_o,
	lsu_ready_wb_o,
	ex_valid_i,
	busy_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	output reg data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	input wire data_err_i;
	output wire [31:0] data_addr_o;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	input wire data_we_ex_i;
	input wire [1:0] data_type_ex_i;
	input wire [31:0] data_wdata_ex_i;
	input wire [1:0] data_reg_offset_ex_i;
	input wire [1:0] data_sign_ext_ex_i;
	output wire [31:0] data_rdata_ex_o;
	input wire data_req_ex_i;
	input wire [31:0] operand_a_ex_i;
	input wire [31:0] operand_b_ex_i;
	input wire addr_useincr_ex_i;
	input wire data_misaligned_ex_i;
	output reg data_misaligned_o;
	input wire [5:0] data_atop_ex_i;
	output wire [5:0] data_atop_o;
	output reg lsu_ready_ex_o;
	output reg lsu_ready_wb_o;
	input wire ex_valid_i;
	output wire busy_o;
	wire [31:0] data_addr_int;
	reg [1:0] data_type_q;
	reg [1:0] rdata_offset_q;
	reg [1:0] data_sign_ext_q;
	reg data_we_q;
	wire [1:0] wdata_offset;
	reg [3:0] data_be;
	reg [31:0] data_wdata;
	wire misaligned_st;
	reg [1:0] CS;
	reg [1:0] NS;
	reg [31:0] rdata_q;
	always @(*) begin
		if (_sv2v_0)
			;
		case (data_type_ex_i)
			2'b00:
				if (misaligned_st == 1'b0)
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b1111;
						2'b01: data_be = 4'b1110;
						2'b10: data_be = 4'b1100;
						2'b11: data_be = 4'b1000;
					endcase
				else
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b0000;
						2'b01: data_be = 4'b0001;
						2'b10: data_be = 4'b0011;
						2'b11: data_be = 4'b0111;
					endcase
			2'b01:
				if (misaligned_st == 1'b0)
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b0011;
						2'b01: data_be = 4'b0110;
						2'b10: data_be = 4'b1100;
						2'b11: data_be = 4'b1000;
					endcase
				else
					data_be = 4'b0001;
			2'b10, 2'b11:
				case (data_addr_int[1:0])
					2'b00: data_be = 4'b0001;
					2'b01: data_be = 4'b0010;
					2'b10: data_be = 4'b0100;
					2'b11: data_be = 4'b1000;
				endcase
		endcase
	end
	assign wdata_offset = data_addr_int[1:0] - data_reg_offset_ex_i[1:0];
	always @(*) begin
		if (_sv2v_0)
			;
		case (wdata_offset)
			2'b00: data_wdata = data_wdata_ex_i[31:0];
			2'b01: data_wdata = {data_wdata_ex_i[23:0], data_wdata_ex_i[31:24]};
			2'b10: data_wdata = {data_wdata_ex_i[15:0], data_wdata_ex_i[31:16]};
			2'b11: data_wdata = {data_wdata_ex_i[7:0], data_wdata_ex_i[31:8]};
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			data_type_q <= 1'sb0;
			rdata_offset_q <= 1'sb0;
			data_sign_ext_q <= 1'sb0;
			data_we_q <= 1'b0;
		end
		else if (data_gnt_i == 1'b1) begin
			data_type_q <= data_type_ex_i;
			rdata_offset_q <= data_addr_int[1:0];
			data_sign_ext_q <= data_sign_ext_ex_i;
			data_we_q <= data_we_ex_i;
		end
	reg [31:0] data_rdata_ext;
	reg [31:0] rdata_w_ext;
	reg [31:0] rdata_h_ext;
	reg [31:0] rdata_b_ext;
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00: rdata_w_ext = data_rdata_i[31:0];
			2'b01: rdata_w_ext = {data_rdata_i[7:0], rdata_q[31:8]};
			2'b10: rdata_w_ext = {data_rdata_i[15:0], rdata_q[31:16]};
			2'b11: rdata_w_ext = {data_rdata_i[23:0], rdata_q[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, data_rdata_i[15:0]};
				else
					rdata_h_ext = {{16 {data_rdata_i[15]}}, data_rdata_i[15:0]};
			2'b01:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, data_rdata_i[23:8]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, data_rdata_i[23:8]};
				else
					rdata_h_ext = {{16 {data_rdata_i[23]}}, data_rdata_i[23:8]};
			2'b10:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, data_rdata_i[31:16]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, data_rdata_i[31:16]};
				else
					rdata_h_ext = {{16 {data_rdata_i[31]}}, data_rdata_i[31:16]};
			2'b11:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, data_rdata_i[7:0], rdata_q[31:24]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, data_rdata_i[7:0], rdata_q[31:24]};
				else
					rdata_h_ext = {{16 {data_rdata_i[7]}}, data_rdata_i[7:0], rdata_q[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, data_rdata_i[7:0]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, data_rdata_i[7:0]};
				else
					rdata_b_ext = {{24 {data_rdata_i[7]}}, data_rdata_i[7:0]};
			2'b01:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, data_rdata_i[15:8]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, data_rdata_i[15:8]};
				else
					rdata_b_ext = {{24 {data_rdata_i[15]}}, data_rdata_i[15:8]};
			2'b10:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, data_rdata_i[23:16]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, data_rdata_i[23:16]};
				else
					rdata_b_ext = {{24 {data_rdata_i[23]}}, data_rdata_i[23:16]};
			2'b11:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, data_rdata_i[31:24]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, data_rdata_i[31:24]};
				else
					rdata_b_ext = {{24 {data_rdata_i[31]}}, data_rdata_i[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (data_type_q)
			2'b00: data_rdata_ext = rdata_w_ext;
			2'b01: data_rdata_ext = rdata_h_ext;
			2'b10, 2'b11: data_rdata_ext = rdata_b_ext;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			CS <= 2'd0;
			rdata_q <= 1'sb0;
		end
		else begin
			CS <= NS;
			if (data_rvalid_i && ~data_we_q) begin
				if ((data_misaligned_ex_i == 1'b1) || (data_misaligned_o == 1'b1))
					rdata_q <= data_rdata_i;
				else
					rdata_q <= data_rdata_ext;
			end
		end
	assign data_rdata_ex_o = (data_rvalid_i == 1'b1 ? data_rdata_ext : rdata_q);
	assign data_addr_o = data_addr_int;
	assign data_wdata_o = data_wdata;
	assign data_we_o = data_we_ex_i;
	assign data_atop_o = data_atop_ex_i;
	assign data_be_o = data_be;
	assign misaligned_st = data_misaligned_ex_i;
	wire load_err_o;
	assign load_err_o = (data_gnt_i && data_err_i) && ~data_we_o;
	wire store_err_o;
	assign store_err_o = (data_gnt_i && data_err_i) && data_we_o;
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		data_req_o = 1'b0;
		lsu_ready_ex_o = 1'b1;
		lsu_ready_wb_o = 1'b1;
		case (CS)
			2'd0: begin
				data_req_o = data_req_ex_i;
				if (data_req_ex_i) begin
					lsu_ready_ex_o = 1'b0;
					if (data_gnt_i) begin
						lsu_ready_ex_o = 1'b1;
						if (ex_valid_i)
							NS = 2'd1;
						else
							NS = 2'd2;
					end
					if (data_err_i)
						lsu_ready_ex_o = 1'b1;
				end
			end
			2'd1: begin
				lsu_ready_wb_o = 1'b0;
				if (data_rvalid_i) begin
					lsu_ready_wb_o = 1'b1;
					data_req_o = data_req_ex_i;
					if (data_req_ex_i) begin
						lsu_ready_ex_o = 1'b0;
						if (data_gnt_i) begin
							lsu_ready_ex_o = 1'b1;
							if (ex_valid_i)
								NS = 2'd1;
							else
								NS = 2'd2;
						end
						else begin
							if (data_err_i)
								lsu_ready_ex_o = 1'b1;
							NS = 2'd0;
						end
					end
					else if (data_rvalid_i)
						NS = 2'd0;
				end
			end
			2'd2: begin
				data_req_o = 1'b0;
				if (data_rvalid_i) begin
					if (ex_valid_i)
						NS = 2'd0;
					else
						NS = 2'd3;
				end
				else if (ex_valid_i)
					NS = 2'd1;
			end
			2'd3:
				if (ex_valid_i)
					NS = 2'd0;
			default: NS = 2'd0;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		data_misaligned_o = 1'b0;
		if ((data_req_ex_i == 1'b1) && (data_misaligned_ex_i == 1'b0))
			case (data_type_ex_i)
				2'b00:
					if (data_addr_int[1:0] != 2'b00)
						data_misaligned_o = 1'b1;
				2'b01:
					if (data_addr_int[1:0] == 2'b11)
						data_misaligned_o = 1'b1;
			endcase
	end
	assign data_addr_int = (addr_useincr_ex_i ? operand_a_ex_i + operand_b_ex_i : operand_a_ex_i);
	assign busy_o = (((CS == 2'd1) || (CS == 2'd2)) || (CS == 2'd3)) || (data_req_o == 1'b1);
	initial _sv2v_0 = 0;
endmodule
module riscv_id_stage (
	clk,
	rst_n,
	test_en_i,
	fregfile_disable_i,
	fetch_enable_i,
	ctrl_busy_o,
	core_ctrl_firstfetch_o,
	is_decoding_o,
	hwlp_dec_cnt_i,
	is_hwlp_i,
	instr_valid_i,
	instr_rdata_i,
	instr_req_o,
	branch_in_ex_o,
	branch_decision_i,
	jump_target_o,
	clear_instr_valid_o,
	pc_set_o,
	pc_mux_o,
	exc_pc_mux_o,
	trap_addr_mux_o,
	illegal_c_insn_i,
	is_compressed_i,
	is_fetch_failed_i,
	pc_if_i,
	pc_id_i,
	halt_if_o,
	id_ready_o,
	ex_ready_i,
	wb_ready_i,
	id_valid_o,
	ex_valid_i,
	pc_ex_o,
	alu_operand_a_ex_o,
	alu_operand_b_ex_o,
	alu_operand_c_ex_o,
	bmask_a_ex_o,
	bmask_b_ex_o,
	imm_vec_ext_ex_o,
	alu_vec_mode_ex_o,
	regfile_waddr_ex_o,
	regfile_we_ex_o,
	regfile_alu_waddr_ex_o,
	regfile_alu_we_ex_o,
	alu_en_ex_o,
	alu_operator_ex_o,
	alu_is_clpx_ex_o,
	alu_is_subrot_ex_o,
	alu_clpx_shift_ex_o,
	mult_operator_ex_o,
	mult_operand_a_ex_o,
	mult_operand_b_ex_o,
	mult_operand_c_ex_o,
	mult_en_ex_o,
	mult_sel_subword_ex_o,
	mult_signed_mode_ex_o,
	mult_imm_ex_o,
	mult_dot_op_a_ex_o,
	mult_dot_op_b_ex_o,
	mult_dot_op_c_ex_o,
	mult_dot_signed_ex_o,
	mult_is_clpx_ex_o,
	mult_clpx_shift_ex_o,
	mult_clpx_img_ex_o,
	apu_en_ex_o,
	apu_type_ex_o,
	apu_op_ex_o,
	apu_lat_ex_o,
	apu_operands_ex_o,
	apu_flags_ex_o,
	apu_waddr_ex_o,
	apu_read_regs_o,
	apu_read_regs_valid_o,
	apu_read_dep_i,
	apu_write_regs_o,
	apu_write_regs_valid_o,
	apu_write_dep_i,
	apu_perf_dep_o,
	apu_busy_i,
	frm_i,
	csr_access_ex_o,
	csr_op_ex_o,
	current_priv_lvl_i,
	csr_irq_sec_o,
	csr_cause_o,
	csr_save_if_o,
	csr_save_id_o,
	csr_save_ex_o,
	csr_restore_mret_id_o,
	csr_restore_uret_id_o,
	csr_restore_dret_id_o,
	csr_save_cause_o,
	hwlp_start_o,
	hwlp_end_o,
	hwlp_cnt_o,
	csr_hwlp_regid_i,
	csr_hwlp_we_i,
	csr_hwlp_data_i,
	data_req_ex_o,
	data_we_ex_o,
	data_type_ex_o,
	data_sign_ext_ex_o,
	data_reg_offset_ex_o,
	data_load_event_ex_o,
	data_misaligned_ex_o,
	prepost_useincr_ex_o,
	data_misaligned_i,
	data_err_i,
	data_err_ack_o,
	atop_ex_o,
	irq_pending_i,
	irq_sec_i,
	irq_id_i,
	m_irq_enable_i,
	u_irq_enable_i,
	irq_ack_o,
	irq_id_o,
	exc_cause_o,
	debug_mode_o,
	debug_cause_o,
	debug_csr_save_o,
	debug_req_i,
	debug_single_step_i,
	debug_ebreakm_i,
	debug_ebreaku_i,
	regfile_waddr_wb_i,
	regfile_we_wb_i,
	regfile_wdata_wb_i,
	regfile_alu_waddr_fw_i,
	regfile_alu_we_fw_i,
	regfile_alu_wdata_fw_i,
	mult_multicycle_i,
	perf_jump_o,
	perf_jr_stall_o,
	perf_ld_stall_o,
	perf_pipeline_stall_o
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter N_HWLP_BITS = $clog2(N_HWLP);
	parameter PULP_SECURE = 0;
	parameter A_EXTENSION = 0;
	parameter APU = 0;
	parameter FPU = 0;
	parameter Zfinx = 0;
	parameter FP_DIVSQRT = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter SHARED_FP_DIVSQRT = 0;
	parameter WAPUTYPE = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire fregfile_disable_i;
	input wire fetch_enable_i;
	output wire ctrl_busy_o;
	output wire core_ctrl_firstfetch_o;
	output wire is_decoding_o;
	input wire [N_HWLP - 1:0] hwlp_dec_cnt_i;
	input wire is_hwlp_i;
	input wire instr_valid_i;
	input wire [31:0] instr_rdata_i;
	output wire instr_req_o;
	output reg branch_in_ex_o;
	input wire branch_decision_i;
	output wire [31:0] jump_target_o;
	output wire clear_instr_valid_o;
	output wire pc_set_o;
	output wire [2:0] pc_mux_o;
	output wire [2:0] exc_pc_mux_o;
	output wire [1:0] trap_addr_mux_o;
	input wire illegal_c_insn_i;
	input wire is_compressed_i;
	input wire is_fetch_failed_i;
	input wire [31:0] pc_if_i;
	input wire [31:0] pc_id_i;
	output wire halt_if_o;
	output wire id_ready_o;
	input wire ex_ready_i;
	input wire wb_ready_i;
	output wire id_valid_o;
	input wire ex_valid_i;
	output reg [31:0] pc_ex_o;
	output reg [31:0] alu_operand_a_ex_o;
	output reg [31:0] alu_operand_b_ex_o;
	output reg [31:0] alu_operand_c_ex_o;
	output reg [4:0] bmask_a_ex_o;
	output reg [4:0] bmask_b_ex_o;
	output reg [1:0] imm_vec_ext_ex_o;
	output reg [1:0] alu_vec_mode_ex_o;
	output reg [5:0] regfile_waddr_ex_o;
	output reg regfile_we_ex_o;
	output reg [5:0] regfile_alu_waddr_ex_o;
	output reg regfile_alu_we_ex_o;
	output reg alu_en_ex_o;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	output reg [6:0] alu_operator_ex_o;
	output reg alu_is_clpx_ex_o;
	output reg alu_is_subrot_ex_o;
	output reg [1:0] alu_clpx_shift_ex_o;
	output reg [2:0] mult_operator_ex_o;
	output reg [31:0] mult_operand_a_ex_o;
	output reg [31:0] mult_operand_b_ex_o;
	output reg [31:0] mult_operand_c_ex_o;
	output reg mult_en_ex_o;
	output reg mult_sel_subword_ex_o;
	output reg [1:0] mult_signed_mode_ex_o;
	output reg [4:0] mult_imm_ex_o;
	output reg [31:0] mult_dot_op_a_ex_o;
	output reg [31:0] mult_dot_op_b_ex_o;
	output reg [31:0] mult_dot_op_c_ex_o;
	output reg [1:0] mult_dot_signed_ex_o;
	output reg mult_is_clpx_ex_o;
	output reg [1:0] mult_clpx_shift_ex_o;
	output reg mult_clpx_img_ex_o;
	output reg apu_en_ex_o;
	output reg [WAPUTYPE - 1:0] apu_type_ex_o;
	output reg [APU_WOP_CPU - 1:0] apu_op_ex_o;
	output reg [1:0] apu_lat_ex_o;
	output reg [(APU_NARGS_CPU * 32) - 1:0] apu_operands_ex_o;
	output reg [APU_NDSFLAGS_CPU - 1:0] apu_flags_ex_o;
	output reg [5:0] apu_waddr_ex_o;
	output wire [17:0] apu_read_regs_o;
	output wire [2:0] apu_read_regs_valid_o;
	input wire apu_read_dep_i;
	output wire [11:0] apu_write_regs_o;
	output wire [1:0] apu_write_regs_valid_o;
	input wire apu_write_dep_i;
	output wire apu_perf_dep_o;
	input wire apu_busy_i;
	localparam riscv_defines_C_RM = 3;
	input wire [2:0] frm_i;
	output reg csr_access_ex_o;
	output reg [1:0] csr_op_ex_o;
	input wire [1:0] current_priv_lvl_i;
	output wire csr_irq_sec_o;
	output wire [6:0] csr_cause_o;
	output wire csr_save_if_o;
	output wire csr_save_id_o;
	output wire csr_save_ex_o;
	output wire csr_restore_mret_id_o;
	output wire csr_restore_uret_id_o;
	output wire csr_restore_dret_id_o;
	output wire csr_save_cause_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_start_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_end_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_cnt_o;
	input wire [N_HWLP_BITS - 1:0] csr_hwlp_regid_i;
	input wire [2:0] csr_hwlp_we_i;
	input wire [31:0] csr_hwlp_data_i;
	output reg data_req_ex_o;
	output reg data_we_ex_o;
	output reg [1:0] data_type_ex_o;
	output reg [1:0] data_sign_ext_ex_o;
	output reg [1:0] data_reg_offset_ex_o;
	output reg data_load_event_ex_o;
	output reg data_misaligned_ex_o;
	output reg prepost_useincr_ex_o;
	input wire data_misaligned_i;
	input wire data_err_i;
	output wire data_err_ack_o;
	output reg [5:0] atop_ex_o;
	input wire irq_pending_i;
	input wire irq_sec_i;
	input wire [5:0] irq_id_i;
	input wire m_irq_enable_i;
	input wire u_irq_enable_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	output wire [5:0] exc_cause_o;
	output wire debug_mode_o;
	output wire [2:0] debug_cause_o;
	output wire debug_csr_save_o;
	input wire debug_req_i;
	input wire debug_single_step_i;
	input wire debug_ebreakm_i;
	input wire debug_ebreaku_i;
	input wire [5:0] regfile_waddr_wb_i;
	input wire regfile_we_wb_i;
	input wire [31:0] regfile_wdata_wb_i;
	input wire [5:0] regfile_alu_waddr_fw_i;
	input wire regfile_alu_we_fw_i;
	input wire [31:0] regfile_alu_wdata_fw_i;
	input wire mult_multicycle_i;
	output wire perf_jump_o;
	output wire perf_jr_stall_o;
	output wire perf_ld_stall_o;
	output wire perf_pipeline_stall_o;
	wire [31:0] instr;
	wire deassert_we;
	wire illegal_insn_dec;
	wire ebrk_insn;
	wire mret_insn_dec;
	wire uret_insn_dec;
	wire dret_insn_dec;
	wire ecall_insn_dec;
	wire pipe_flush_dec;
	wire fencei_insn_dec;
	wire rega_used_dec;
	wire regb_used_dec;
	wire regc_used_dec;
	wire branch_taken_ex;
	wire [1:0] jump_in_id;
	wire [1:0] jump_in_dec;
	wire misaligned_stall;
	wire jr_stall;
	wire load_stall;
	wire csr_apu_stall;
	wire instr_multicycle;
	wire hwloop_mask;
	wire halt_id;
	wire [31:0] imm_i_type;
	wire [31:0] imm_iz_type;
	wire [31:0] imm_s_type;
	wire [31:0] imm_sb_type;
	wire [31:0] imm_u_type;
	wire [31:0] imm_uj_type;
	wire [31:0] imm_z_type;
	wire [31:0] imm_s2_type;
	wire [31:0] imm_bi_type;
	wire [31:0] imm_s3_type;
	wire [31:0] imm_vs_type;
	wire [31:0] imm_vu_type;
	wire [31:0] imm_shuffleb_type;
	wire [31:0] imm_shuffleh_type;
	reg [31:0] imm_shuffle_type;
	wire [31:0] imm_clip_type;
	reg [31:0] imm_a;
	reg [31:0] imm_b;
	reg [31:0] jump_target;
	wire irq_req_ctrl;
	wire irq_sec_ctrl;
	wire [5:0] irq_id_ctrl;
	wire exc_ack;
	wire exc_kill;
	wire [5:0] regfile_addr_ra_id;
	wire [5:0] regfile_addr_rb_id;
	reg [5:0] regfile_addr_rc_id;
	wire regfile_fp_a;
	wire regfile_fp_b;
	wire regfile_fp_c;
	wire regfile_fp_d;
	wire fregfile_ena;
	wire [5:0] regfile_waddr_id;
	wire [5:0] regfile_alu_waddr_id;
	wire regfile_alu_we_id;
	wire regfile_alu_we_dec_id;
	wire [31:0] regfile_data_ra_id;
	wire [31:0] regfile_data_rb_id;
	wire [31:0] regfile_data_rc_id;
	wire alu_en;
	wire [6:0] alu_operator;
	wire [2:0] alu_op_a_mux_sel;
	wire [2:0] alu_op_b_mux_sel;
	wire [1:0] alu_op_c_mux_sel;
	wire [1:0] regc_mux;
	wire [0:0] imm_a_mux_sel;
	wire [3:0] imm_b_mux_sel;
	wire [1:0] jump_target_mux_sel;
	wire [2:0] mult_operator;
	wire mult_en;
	wire mult_int_en;
	wire mult_sel_subword;
	wire [1:0] mult_signed_mode;
	wire mult_dot_en;
	wire [1:0] mult_dot_signed;
	localparam [31:0] fpnew_pkg_NUM_FP_FORMATS = 5;
	localparam [31:0] fpnew_pkg_FP_FORMAT_BITS = 3;
	localparam riscv_defines_C_FPNEW_FMTBITS = fpnew_pkg_FP_FORMAT_BITS;
	wire [2:0] fpu_src_fmt;
	wire [2:0] fpu_dst_fmt;
	localparam [31:0] fpnew_pkg_NUM_INT_FORMATS = 4;
	localparam [31:0] fpnew_pkg_INT_FORMAT_BITS = 2;
	localparam riscv_defines_C_FPNEW_IFMTBITS = fpnew_pkg_INT_FORMAT_BITS;
	wire [1:0] fpu_int_fmt;
	wire apu_en;
	wire [WAPUTYPE - 1:0] apu_type;
	wire [APU_WOP_CPU - 1:0] apu_op;
	wire [1:0] apu_lat;
	wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands;
	reg [APU_NDSFLAGS_CPU - 1:0] apu_flags;
	wire [5:0] apu_waddr;
	reg [17:0] apu_read_regs;
	reg [2:0] apu_read_regs_valid;
	wire [11:0] apu_write_regs;
	wire [1:0] apu_write_regs_valid;
	wire [WAPUTYPE - 1:0] apu_flags_src;
	wire apu_stall;
	wire [2:0] fp_rnd_mode;
	wire regfile_we_id;
	wire regfile_alu_waddr_mux_sel;
	wire data_we_id;
	wire [1:0] data_type_id;
	wire [1:0] data_sign_ext_id;
	wire [1:0] data_reg_offset_id;
	wire data_req_id;
	wire data_load_event_id;
	wire [5:0] atop_id;
	wire [N_HWLP_BITS - 1:0] hwloop_regid;
	wire [N_HWLP_BITS - 1:0] hwloop_regid_int;
	wire [2:0] hwloop_we;
	wire [2:0] hwloop_we_int;
	wire [2:0] hwloop_we_masked;
	wire hwloop_target_mux_sel;
	wire hwloop_start_mux_sel;
	wire hwloop_cnt_mux_sel;
	reg [31:0] hwloop_target;
	wire [31:0] hwloop_start;
	reg [31:0] hwloop_start_int;
	wire [31:0] hwloop_end;
	wire [31:0] hwloop_cnt;
	reg [31:0] hwloop_cnt_int;
	wire hwloop_valid;
	wire csr_access;
	wire [1:0] csr_op;
	wire csr_status;
	wire prepost_useincr;
	wire [1:0] operand_a_fw_mux_sel;
	wire [1:0] operand_b_fw_mux_sel;
	wire [1:0] operand_c_fw_mux_sel;
	reg [31:0] operand_a_fw_id;
	reg [31:0] operand_b_fw_id;
	reg [31:0] operand_c_fw_id;
	reg [31:0] operand_b;
	reg [31:0] operand_b_vec;
	reg [31:0] operand_c;
	reg [31:0] operand_c_vec;
	reg [31:0] alu_operand_a;
	wire [31:0] alu_operand_b;
	wire [31:0] alu_operand_c;
	wire [0:0] bmask_a_mux;
	wire [1:0] bmask_b_mux;
	wire alu_bmask_a_mux_sel;
	wire alu_bmask_b_mux_sel;
	wire [0:0] mult_imm_mux;
	reg [4:0] bmask_a_id_imm;
	reg [4:0] bmask_b_id_imm;
	reg [4:0] bmask_a_id;
	reg [4:0] bmask_b_id;
	wire [1:0] imm_vec_ext_id;
	reg [4:0] mult_imm_id;
	wire [1:0] alu_vec_mode;
	wire scalar_replication;
	wire scalar_replication_c;
	wire reg_d_ex_is_reg_a_id;
	wire reg_d_ex_is_reg_b_id;
	wire reg_d_ex_is_reg_c_id;
	wire reg_d_wb_is_reg_a_id;
	wire reg_d_wb_is_reg_b_id;
	wire reg_d_wb_is_reg_c_id;
	wire reg_d_alu_is_reg_a_id;
	wire reg_d_alu_is_reg_b_id;
	wire reg_d_alu_is_reg_c_id;
	wire is_clpx;
	wire is_subrot;
	wire mret_dec;
	wire uret_dec;
	wire dret_dec;
	assign instr = instr_rdata_i;
	assign imm_i_type = {{20 {instr[31]}}, instr[31:20]};
	assign imm_iz_type = {20'b00000000000000000000, instr[31:20]};
	assign imm_s_type = {{20 {instr[31]}}, instr[31:25], instr[11:7]};
	assign imm_sb_type = {{19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
	assign imm_u_type = {instr[31:12], 12'b000000000000};
	assign imm_uj_type = {{12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
	assign imm_z_type = {27'b000000000000000000000000000, instr[19:15]};
	assign imm_s2_type = {27'b000000000000000000000000000, instr[24:20]};
	assign imm_bi_type = {{27 {instr[24]}}, instr[24:20]};
	assign imm_s3_type = {27'b000000000000000000000000000, instr[29:25]};
	assign imm_vs_type = {{26 {instr[24]}}, instr[24:20], instr[25]};
	assign imm_vu_type = {26'b00000000000000000000000000, instr[24:20], instr[25]};
	assign imm_shuffleb_type = {6'b000000, instr[28:27], 6'b000000, instr[24:23], 6'b000000, instr[22:21], 6'b000000, instr[20], instr[25]};
	assign imm_shuffleh_type = {15'h0000, instr[20], 15'h0000, instr[25]};
	assign imm_clip_type = (32'h00000001 << instr[24:20]) - 1;
	assign fregfile_ena = (FPU && !Zfinx ? ~fregfile_disable_i : 1'b0);
	assign regfile_addr_ra_id = {fregfile_ena & regfile_fp_a, instr[19:15]};
	assign regfile_addr_rb_id = {fregfile_ena & regfile_fp_b, instr[24:20]};
	localparam riscv_defines_REGC_RD = 2'b01;
	localparam riscv_defines_REGC_S1 = 2'b10;
	localparam riscv_defines_REGC_S4 = 2'b00;
	localparam riscv_defines_REGC_ZERO = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		case (regc_mux)
			riscv_defines_REGC_ZERO: regfile_addr_rc_id = 1'sb0;
			riscv_defines_REGC_RD: regfile_addr_rc_id = {fregfile_ena & regfile_fp_c, instr[11:7]};
			riscv_defines_REGC_S1: regfile_addr_rc_id = {fregfile_ena & regfile_fp_c, instr[19:15]};
			riscv_defines_REGC_S4: regfile_addr_rc_id = {fregfile_ena & regfile_fp_c, instr[31:27]};
			default: regfile_addr_rc_id = 1'sb0;
		endcase
	end
	assign regfile_waddr_id = {fregfile_ena & regfile_fp_d, instr[11:7]};
	assign regfile_alu_waddr_id = (regfile_alu_waddr_mux_sel ? regfile_waddr_id : regfile_addr_ra_id);
	assign reg_d_ex_is_reg_a_id = ((regfile_waddr_ex_o == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_ex_is_reg_b_id = ((regfile_waddr_ex_o == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_ex_is_reg_c_id = ((regfile_waddr_ex_o == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_a_id = ((regfile_waddr_wb_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_b_id = ((regfile_waddr_wb_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_c_id = ((regfile_waddr_wb_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_a_id = ((regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_b_id = ((regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_c_id = ((regfile_alu_waddr_fw_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign clear_instr_valid_o = (id_ready_o | halt_id) | branch_taken_ex;
	assign branch_taken_ex = branch_in_ex_o & branch_decision_i;
	assign mult_en = mult_int_en | mult_dot_en;
	assign hwloop_regid_int = instr[7];
	always @(*) begin
		if (_sv2v_0)
			;
		case (hwloop_target_mux_sel)
			1'b0: hwloop_target = pc_id_i + {imm_iz_type[30:0], 1'b0};
			1'b1: hwloop_target = pc_id_i + {imm_z_type[30:0], 1'b0};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (hwloop_start_mux_sel)
			1'b0: hwloop_start_int = hwloop_target;
			1'b1: hwloop_start_int = pc_if_i;
		endcase
	end
	always @(*) begin : hwloop_cnt_mux
		if (_sv2v_0)
			;
		case (hwloop_cnt_mux_sel)
			1'b0: hwloop_cnt_int = imm_iz_type;
			1'b1: hwloop_cnt_int = operand_a_fw_id;
		endcase
	end
	assign hwloop_we_masked = (hwloop_we_int & ~{3 {hwloop_mask}}) & {3 {id_ready_o}};
	assign hwloop_start = (hwloop_we_masked[0] ? hwloop_start_int : csr_hwlp_data_i);
	assign hwloop_end = (hwloop_we_masked[1] ? hwloop_target : csr_hwlp_data_i);
	assign hwloop_cnt = (hwloop_we_masked[2] ? hwloop_cnt_int : csr_hwlp_data_i);
	assign hwloop_regid = (|hwloop_we_masked ? hwloop_regid_int : csr_hwlp_regid_i);
	assign hwloop_we = (|hwloop_we_masked ? hwloop_we_masked : csr_hwlp_we_i);
	localparam riscv_defines_JT_COND = 2'b11;
	localparam riscv_defines_JT_JAL = 2'b01;
	localparam riscv_defines_JT_JALR = 2'b10;
	always @(*) begin : jump_target_mux
		if (_sv2v_0)
			;
		case (jump_target_mux_sel)
			riscv_defines_JT_JAL: jump_target = pc_id_i + imm_uj_type;
			riscv_defines_JT_COND: jump_target = pc_id_i + imm_sb_type;
			riscv_defines_JT_JALR: jump_target = regfile_data_ra_id + imm_i_type;
			default: jump_target = regfile_data_ra_id + imm_i_type;
		endcase
	end
	assign jump_target_o = jump_target;
	localparam riscv_defines_OP_A_CURRPC = 3'b001;
	localparam riscv_defines_OP_A_IMM = 3'b010;
	localparam riscv_defines_OP_A_REGA_OR_FWD = 3'b000;
	localparam riscv_defines_OP_A_REGB_OR_FWD = 3'b011;
	localparam riscv_defines_OP_A_REGC_OR_FWD = 3'b100;
	always @(*) begin : alu_operand_a_mux
		if (_sv2v_0)
			;
		case (alu_op_a_mux_sel)
			riscv_defines_OP_A_REGA_OR_FWD: alu_operand_a = operand_a_fw_id;
			riscv_defines_OP_A_REGB_OR_FWD: alu_operand_a = operand_b_fw_id;
			riscv_defines_OP_A_REGC_OR_FWD: alu_operand_a = operand_c_fw_id;
			riscv_defines_OP_A_CURRPC: alu_operand_a = pc_id_i;
			riscv_defines_OP_A_IMM: alu_operand_a = imm_a;
			default: alu_operand_a = operand_a_fw_id;
		endcase
	end
	localparam riscv_defines_IMMA_Z = 1'b0;
	localparam riscv_defines_IMMA_ZERO = 1'b1;
	always @(*) begin : immediate_a_mux
		if (_sv2v_0)
			;
		case (imm_a_mux_sel)
			riscv_defines_IMMA_Z: imm_a = imm_z_type;
			riscv_defines_IMMA_ZERO: imm_a = 1'sb0;
			default: imm_a = 1'sb0;
		endcase
	end
	localparam riscv_defines_SEL_FW_EX = 2'b01;
	localparam riscv_defines_SEL_FW_WB = 2'b10;
	localparam riscv_defines_SEL_REGFILE = 2'b00;
	always @(*) begin : operand_a_fw_mux
		if (_sv2v_0)
			;
		case (operand_a_fw_mux_sel)
			riscv_defines_SEL_FW_EX: operand_a_fw_id = regfile_alu_wdata_fw_i;
			riscv_defines_SEL_FW_WB: operand_a_fw_id = regfile_wdata_wb_i;
			riscv_defines_SEL_REGFILE: operand_a_fw_id = regfile_data_ra_id;
			default: operand_a_fw_id = regfile_data_ra_id;
		endcase
	end
	localparam riscv_defines_IMMB_BI = 4'b1011;
	localparam riscv_defines_IMMB_CLIP = 4'b1001;
	localparam riscv_defines_IMMB_I = 4'b0000;
	localparam riscv_defines_IMMB_PCINCR = 4'b0011;
	localparam riscv_defines_IMMB_S = 4'b0001;
	localparam riscv_defines_IMMB_S2 = 4'b0100;
	localparam riscv_defines_IMMB_S3 = 4'b0101;
	localparam riscv_defines_IMMB_SHUF = 4'b1000;
	localparam riscv_defines_IMMB_U = 4'b0010;
	localparam riscv_defines_IMMB_VS = 4'b0110;
	localparam riscv_defines_IMMB_VU = 4'b0111;
	always @(*) begin : immediate_b_mux
		if (_sv2v_0)
			;
		case (imm_b_mux_sel)
			riscv_defines_IMMB_I: imm_b = imm_i_type;
			riscv_defines_IMMB_S: imm_b = imm_s_type;
			riscv_defines_IMMB_U: imm_b = imm_u_type;
			riscv_defines_IMMB_PCINCR: imm_b = (is_compressed_i && ~data_misaligned_i ? 32'h00000002 : 32'h00000004);
			riscv_defines_IMMB_S2: imm_b = imm_s2_type;
			riscv_defines_IMMB_BI: imm_b = imm_bi_type;
			riscv_defines_IMMB_S3: imm_b = imm_s3_type;
			riscv_defines_IMMB_VS: imm_b = imm_vs_type;
			riscv_defines_IMMB_VU: imm_b = imm_vu_type;
			riscv_defines_IMMB_SHUF: imm_b = imm_shuffle_type;
			riscv_defines_IMMB_CLIP: imm_b = {1'b0, imm_clip_type[31:1]};
			default: imm_b = imm_i_type;
		endcase
	end
	localparam riscv_defines_OP_B_BMASK = 3'b100;
	localparam riscv_defines_OP_B_IMM = 3'b010;
	localparam riscv_defines_OP_B_REGA_OR_FWD = 3'b011;
	localparam riscv_defines_OP_B_REGB_OR_FWD = 3'b000;
	localparam riscv_defines_OP_B_REGC_OR_FWD = 3'b001;
	always @(*) begin : alu_operand_b_mux
		if (_sv2v_0)
			;
		case (alu_op_b_mux_sel)
			riscv_defines_OP_B_REGA_OR_FWD: operand_b = operand_a_fw_id;
			riscv_defines_OP_B_REGB_OR_FWD: operand_b = operand_b_fw_id;
			riscv_defines_OP_B_REGC_OR_FWD: operand_b = operand_c_fw_id;
			riscv_defines_OP_B_IMM: operand_b = imm_b;
			riscv_defines_OP_B_BMASK: operand_b = $unsigned(operand_b_fw_id[4:0]);
			default: operand_b = operand_b_fw_id;
		endcase
	end
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		if (alu_vec_mode == riscv_defines_VEC_MODE8) begin
			operand_b_vec = {4 {operand_b[7:0]}};
			imm_shuffle_type = imm_shuffleb_type;
		end
		else begin
			operand_b_vec = {2 {operand_b[15:0]}};
			imm_shuffle_type = imm_shuffleh_type;
		end
	end
	assign alu_operand_b = (scalar_replication == 1'b1 ? operand_b_vec : operand_b);
	always @(*) begin : operand_b_fw_mux
		if (_sv2v_0)
			;
		case (operand_b_fw_mux_sel)
			riscv_defines_SEL_FW_EX: operand_b_fw_id = regfile_alu_wdata_fw_i;
			riscv_defines_SEL_FW_WB: operand_b_fw_id = regfile_wdata_wb_i;
			riscv_defines_SEL_REGFILE: operand_b_fw_id = regfile_data_rb_id;
			default: operand_b_fw_id = regfile_data_rb_id;
		endcase
	end
	localparam riscv_defines_OP_C_JT = 2'b10;
	localparam riscv_defines_OP_C_REGB_OR_FWD = 2'b01;
	localparam riscv_defines_OP_C_REGC_OR_FWD = 2'b00;
	always @(*) begin : alu_operand_c_mux
		if (_sv2v_0)
			;
		case (alu_op_c_mux_sel)
			riscv_defines_OP_C_REGC_OR_FWD: operand_c = operand_c_fw_id;
			riscv_defines_OP_C_REGB_OR_FWD: operand_c = operand_b_fw_id;
			riscv_defines_OP_C_JT: operand_c = jump_target;
			default: operand_c = operand_c_fw_id;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		if (alu_vec_mode == riscv_defines_VEC_MODE8)
			operand_c_vec = {4 {operand_c[7:0]}};
		else
			operand_c_vec = {2 {operand_c[15:0]}};
	end
	assign alu_operand_c = (scalar_replication_c == 1'b1 ? operand_c_vec : operand_c);
	always @(*) begin : operand_c_fw_mux
		if (_sv2v_0)
			;
		case (operand_c_fw_mux_sel)
			riscv_defines_SEL_FW_EX: operand_c_fw_id = regfile_alu_wdata_fw_i;
			riscv_defines_SEL_FW_WB: operand_c_fw_id = regfile_wdata_wb_i;
			riscv_defines_SEL_REGFILE: operand_c_fw_id = regfile_data_rc_id;
			default: operand_c_fw_id = regfile_data_rc_id;
		endcase
	end
	localparam riscv_defines_BMASK_A_S3 = 1'b1;
	localparam riscv_defines_BMASK_A_ZERO = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (bmask_a_mux)
			riscv_defines_BMASK_A_ZERO: bmask_a_id_imm = 1'sb0;
			riscv_defines_BMASK_A_S3: bmask_a_id_imm = imm_s3_type[4:0];
			default: bmask_a_id_imm = 1'sb0;
		endcase
	end
	localparam riscv_defines_BMASK_B_ONE = 2'b11;
	localparam riscv_defines_BMASK_B_S2 = 2'b00;
	localparam riscv_defines_BMASK_B_S3 = 2'b01;
	localparam riscv_defines_BMASK_B_ZERO = 2'b10;
	always @(*) begin
		if (_sv2v_0)
			;
		case (bmask_b_mux)
			riscv_defines_BMASK_B_ZERO: bmask_b_id_imm = 1'sb0;
			riscv_defines_BMASK_B_ONE: bmask_b_id_imm = 5'd1;
			riscv_defines_BMASK_B_S2: bmask_b_id_imm = imm_s2_type[4:0];
			riscv_defines_BMASK_B_S3: bmask_b_id_imm = imm_s3_type[4:0];
			default: bmask_b_id_imm = 1'sb0;
		endcase
	end
	localparam riscv_defines_BMASK_A_IMM = 1'b1;
	localparam riscv_defines_BMASK_A_REG = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (alu_bmask_a_mux_sel)
			riscv_defines_BMASK_A_IMM: bmask_a_id = bmask_a_id_imm;
			riscv_defines_BMASK_A_REG: bmask_a_id = operand_b_fw_id[9:5];
			default: bmask_a_id = bmask_a_id_imm;
		endcase
	end
	localparam riscv_defines_BMASK_B_IMM = 1'b1;
	localparam riscv_defines_BMASK_B_REG = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (alu_bmask_b_mux_sel)
			riscv_defines_BMASK_B_IMM: bmask_b_id = bmask_b_id_imm;
			riscv_defines_BMASK_B_REG: bmask_b_id = operand_b_fw_id[4:0];
			default: bmask_b_id = bmask_b_id_imm;
		endcase
	end
	assign imm_vec_ext_id = imm_vu_type[1:0];
	localparam riscv_defines_MIMM_S3 = 1'b1;
	localparam riscv_defines_MIMM_ZERO = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (mult_imm_mux)
			riscv_defines_MIMM_ZERO: mult_imm_id = 1'sb0;
			riscv_defines_MIMM_S3: mult_imm_id = imm_s3_type[4:0];
			default: mult_imm_id = 1'sb0;
		endcase
	end
	localparam apu_core_package_APU_FLAGS_DSP_MULT = 0;
	localparam apu_core_package_APU_FLAGS_FP = 2;
	localparam apu_core_package_APU_FLAGS_FPNEW = 3;
	localparam apu_core_package_APU_FLAGS_INT_MULT = 1;
	generate
		if (APU == 1) begin : apu_op_preparation
			if (APU_NARGS_CPU >= 1) begin : genblk1
				assign apu_operands[0+:32] = alu_operand_a;
			end
			if (APU_NARGS_CPU >= 2) begin : genblk2
				assign apu_operands[32+:32] = alu_operand_b;
			end
			if (APU_NARGS_CPU >= 3) begin : genblk3
				assign apu_operands[64+:32] = alu_operand_c;
			end
			assign apu_waddr = regfile_alu_waddr_id;
			always @(*) begin
				if (_sv2v_0)
					;
				case (apu_flags_src)
					apu_core_package_APU_FLAGS_INT_MULT: apu_flags = {7'h00, mult_imm_id, mult_signed_mode, mult_sel_subword};
					apu_core_package_APU_FLAGS_DSP_MULT: apu_flags = {13'h0000, mult_dot_signed};
					apu_core_package_APU_FLAGS_FP:
						if (FPU == 1)
							apu_flags = fp_rnd_mode;
						else
							apu_flags = 1'sb0;
					apu_core_package_APU_FLAGS_FPNEW:
						if (FPU == 1)
							apu_flags = {fpu_int_fmt, fpu_src_fmt, fpu_dst_fmt, fp_rnd_mode};
						else
							apu_flags = 1'sb0;
					default: apu_flags = 1'sb0;
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				case (alu_op_a_mux_sel)
					riscv_defines_OP_A_REGA_OR_FWD: begin
						apu_read_regs[0+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[0] = 1'b1;
					end
					riscv_defines_OP_A_REGB_OR_FWD: begin
						apu_read_regs[0+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[0] = 1'b1;
					end
					default: begin
						apu_read_regs[0+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[0] = 1'b0;
					end
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				case (alu_op_b_mux_sel)
					riscv_defines_OP_B_REGA_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					riscv_defines_OP_B_REGB_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					riscv_defines_OP_B_REGC_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					default: begin
						apu_read_regs[6+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[1] = 1'b0;
					end
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				case (alu_op_c_mux_sel)
					riscv_defines_OP_C_REGB_OR_FWD: begin
						apu_read_regs[12+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[2] = 1'b1;
					end
					riscv_defines_OP_C_REGC_OR_FWD: begin
						apu_read_regs[12+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[2] = 1'b1;
					end
					default: begin
						apu_read_regs[12+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[2] = 1'b0;
					end
				endcase
			end
			assign apu_write_regs[0+:6] = regfile_alu_waddr_id;
			assign apu_write_regs_valid[0] = regfile_alu_we_id;
			assign apu_write_regs[6+:6] = regfile_waddr_id;
			assign apu_write_regs_valid[1] = regfile_we_id;
			assign apu_read_regs_o = apu_read_regs;
			assign apu_read_regs_valid_o = apu_read_regs_valid;
			assign apu_write_regs_o = apu_write_regs;
			assign apu_write_regs_valid_o = apu_write_regs_valid;
		end
		else begin : genblk1
			genvar _gv_i_3;
			for (_gv_i_3 = 0; _gv_i_3 < APU_NARGS_CPU; _gv_i_3 = _gv_i_3 + 1) begin : apu_tie_off
				localparam i = _gv_i_3;
				assign apu_operands[i * 32+:32] = 1'sb0;
			end
			assign apu_waddr = 1'sb0;
			wire [APU_NDSFLAGS_CPU:1] sv2v_tmp_718B3;
			assign sv2v_tmp_718B3 = 1'sb0;
			always @(*) apu_flags = sv2v_tmp_718B3;
			assign apu_write_regs_o = 1'sb0;
			assign apu_read_regs_o = 1'sb0;
			assign apu_write_regs_valid_o = 1'sb0;
			assign apu_read_regs_valid_o = 1'sb0;
		end
	endgenerate
	assign apu_perf_dep_o = apu_stall;
	assign csr_apu_stall = csr_access & ((apu_en_ex_o & (apu_lat_ex_o[1] == 1'b1)) | apu_busy_i);
	register_file_test_wrap #(
		.ADDR_WIDTH(6),
		.FPU(FPU),
		.Zfinx(Zfinx)
	) registers_i(
		.clk(clk),
		.rst_n(rst_n),
		.test_en_i(test_en_i),
		.raddr_a_i(regfile_addr_ra_id),
		.rdata_a_o(regfile_data_ra_id),
		.raddr_b_i(regfile_addr_rb_id),
		.rdata_b_o(regfile_data_rb_id),
		.raddr_c_i(regfile_addr_rc_id),
		.rdata_c_o(regfile_data_rc_id),
		.waddr_a_i(regfile_waddr_wb_i),
		.wdata_a_i(regfile_wdata_wb_i),
		.we_a_i(regfile_we_wb_i),
		.waddr_b_i(regfile_alu_waddr_fw_i),
		.wdata_b_i(regfile_alu_wdata_fw_i),
		.we_b_i(regfile_alu_we_fw_i),
		.BIST(1'b0),
		.CSN_T(),
		.WEN_T(),
		.A_T(),
		.D_T(),
		.Q_T()
	);
	riscv_decoder #(
		.A_EXTENSION(A_EXTENSION),
		.FPU(FPU),
		.FP_DIVSQRT(FP_DIVSQRT),
		.PULP_SECURE(PULP_SECURE),
		.SHARED_FP(SHARED_FP),
		.SHARED_DSP_MULT(SHARED_DSP_MULT),
		.SHARED_INT_MULT(SHARED_INT_MULT),
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.SHARED_FP_DIVSQRT(SHARED_FP_DIVSQRT),
		.WAPUTYPE(WAPUTYPE),
		.APU_WOP_CPU(APU_WOP_CPU)
	) decoder_i(
		.deassert_we_i(deassert_we),
		.data_misaligned_i(data_misaligned_i),
		.mult_multicycle_i(mult_multicycle_i),
		.instr_multicycle_o(instr_multicycle),
		.illegal_insn_o(illegal_insn_dec),
		.ebrk_insn_o(ebrk_insn),
		.mret_insn_o(mret_insn_dec),
		.uret_insn_o(uret_insn_dec),
		.dret_insn_o(dret_insn_dec),
		.mret_dec_o(mret_dec),
		.uret_dec_o(uret_dec),
		.dret_dec_o(dret_dec),
		.ecall_insn_o(ecall_insn_dec),
		.pipe_flush_o(pipe_flush_dec),
		.fencei_insn_o(fencei_insn_dec),
		.rega_used_o(rega_used_dec),
		.regb_used_o(regb_used_dec),
		.regc_used_o(regc_used_dec),
		.reg_fp_a_o(regfile_fp_a),
		.reg_fp_b_o(regfile_fp_b),
		.reg_fp_c_o(regfile_fp_c),
		.reg_fp_d_o(regfile_fp_d),
		.bmask_a_mux_o(bmask_a_mux),
		.bmask_b_mux_o(bmask_b_mux),
		.alu_bmask_a_mux_sel_o(alu_bmask_a_mux_sel),
		.alu_bmask_b_mux_sel_o(alu_bmask_b_mux_sel),
		.instr_rdata_i(instr),
		.illegal_c_insn_i(illegal_c_insn_i),
		.alu_en_o(alu_en),
		.alu_operator_o(alu_operator),
		.alu_op_a_mux_sel_o(alu_op_a_mux_sel),
		.alu_op_b_mux_sel_o(alu_op_b_mux_sel),
		.alu_op_c_mux_sel_o(alu_op_c_mux_sel),
		.alu_vec_mode_o(alu_vec_mode),
		.scalar_replication_o(scalar_replication),
		.scalar_replication_c_o(scalar_replication_c),
		.imm_a_mux_sel_o(imm_a_mux_sel),
		.imm_b_mux_sel_o(imm_b_mux_sel),
		.regc_mux_o(regc_mux),
		.is_clpx_o(is_clpx),
		.is_subrot_o(is_subrot),
		.mult_operator_o(mult_operator),
		.mult_int_en_o(mult_int_en),
		.mult_sel_subword_o(mult_sel_subword),
		.mult_signed_mode_o(mult_signed_mode),
		.mult_imm_mux_o(mult_imm_mux),
		.mult_dot_en_o(mult_dot_en),
		.mult_dot_signed_o(mult_dot_signed),
		.frm_i(frm_i),
		.fpu_src_fmt_o(fpu_src_fmt),
		.fpu_dst_fmt_o(fpu_dst_fmt),
		.fpu_int_fmt_o(fpu_int_fmt),
		.apu_en_o(apu_en),
		.apu_type_o(apu_type),
		.apu_op_o(apu_op),
		.apu_lat_o(apu_lat),
		.apu_flags_src_o(apu_flags_src),
		.fp_rnd_mode_o(fp_rnd_mode),
		.regfile_mem_we_o(regfile_we_id),
		.regfile_alu_we_o(regfile_alu_we_id),
		.regfile_alu_we_dec_o(regfile_alu_we_dec_id),
		.regfile_alu_waddr_sel_o(regfile_alu_waddr_mux_sel),
		.csr_access_o(csr_access),
		.csr_status_o(csr_status),
		.csr_op_o(csr_op),
		.current_priv_lvl_i(current_priv_lvl_i),
		.data_req_o(data_req_id),
		.data_we_o(data_we_id),
		.prepost_useincr_o(prepost_useincr),
		.data_type_o(data_type_id),
		.data_sign_extension_o(data_sign_ext_id),
		.data_reg_offset_o(data_reg_offset_id),
		.data_load_event_o(data_load_event_id),
		.atop_o(atop_id),
		.hwloop_we_o(hwloop_we_int),
		.hwloop_target_mux_sel_o(hwloop_target_mux_sel),
		.hwloop_start_mux_sel_o(hwloop_start_mux_sel),
		.hwloop_cnt_mux_sel_o(hwloop_cnt_mux_sel),
		.jump_in_dec_o(jump_in_dec),
		.jump_in_id_o(jump_in_id),
		.jump_target_mux_sel_o(jump_target_mux_sel)
	);
	riscv_controller #(.FPU(FPU)) controller_i(
		.clk(clk),
		.rst_n(rst_n),
		.fetch_enable_i(fetch_enable_i),
		.ctrl_busy_o(ctrl_busy_o),
		.first_fetch_o(core_ctrl_firstfetch_o),
		.is_decoding_o(is_decoding_o),
		.is_fetch_failed_i(is_fetch_failed_i),
		.deassert_we_o(deassert_we),
		.illegal_insn_i(illegal_insn_dec),
		.ecall_insn_i(ecall_insn_dec),
		.mret_insn_i(mret_insn_dec),
		.uret_insn_i(uret_insn_dec),
		.dret_insn_i(dret_insn_dec),
		.mret_dec_i(mret_dec),
		.uret_dec_i(uret_dec),
		.dret_dec_i(dret_dec),
		.pipe_flush_i(pipe_flush_dec),
		.ebrk_insn_i(ebrk_insn),
		.fencei_insn_i(fencei_insn_dec),
		.csr_status_i(csr_status),
		.instr_multicycle_i(instr_multicycle),
		.hwloop_mask_o(hwloop_mask),
		.instr_valid_i(instr_valid_i),
		.instr_req_o(instr_req_o),
		.pc_set_o(pc_set_o),
		.pc_mux_o(pc_mux_o),
		.exc_pc_mux_o(exc_pc_mux_o),
		.exc_cause_o(exc_cause_o),
		.trap_addr_mux_o(trap_addr_mux_o),
		.data_req_ex_i(data_req_ex_o),
		.data_we_ex_i(data_we_ex_o),
		.data_misaligned_i(data_misaligned_i),
		.data_load_event_i(data_load_event_id),
		.data_err_i(data_err_i),
		.data_err_ack_o(data_err_ack_o),
		.mult_multicycle_i(mult_multicycle_i),
		.apu_en_i(apu_en),
		.apu_read_dep_i(apu_read_dep_i),
		.apu_write_dep_i(apu_write_dep_i),
		.apu_stall_o(apu_stall),
		.branch_taken_ex_i(branch_taken_ex),
		.jump_in_id_i(jump_in_id),
		.jump_in_dec_i(jump_in_dec),
		.irq_pending_i(irq_pending_i),
		.irq_req_ctrl_i(irq_req_ctrl),
		.irq_sec_ctrl_i(irq_sec_ctrl),
		.irq_id_ctrl_i(irq_id_ctrl),
		.m_IE_i(m_irq_enable_i),
		.u_IE_i(u_irq_enable_i),
		.current_priv_lvl_i(current_priv_lvl_i),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.exc_ack_o(exc_ack),
		.exc_kill_o(exc_kill),
		.debug_mode_o(debug_mode_o),
		.debug_cause_o(debug_cause_o),
		.debug_csr_save_o(debug_csr_save_o),
		.debug_req_i(debug_req_i),
		.debug_single_step_i(debug_single_step_i),
		.debug_ebreakm_i(debug_ebreakm_i),
		.debug_ebreaku_i(debug_ebreaku_i),
		.csr_save_cause_o(csr_save_cause_o),
		.csr_cause_o(csr_cause_o),
		.csr_save_if_o(csr_save_if_o),
		.csr_save_id_o(csr_save_id_o),
		.csr_save_ex_o(csr_save_ex_o),
		.csr_restore_mret_id_o(csr_restore_mret_id_o),
		.csr_restore_uret_id_o(csr_restore_uret_id_o),
		.csr_restore_dret_id_o(csr_restore_dret_id_o),
		.csr_irq_sec_o(csr_irq_sec_o),
		.regfile_we_id_i(regfile_alu_we_dec_id),
		.regfile_alu_waddr_id_i(regfile_alu_waddr_id),
		.regfile_we_ex_i(regfile_we_ex_o),
		.regfile_waddr_ex_i(regfile_waddr_ex_o),
		.regfile_we_wb_i(regfile_we_wb_i),
		.regfile_alu_we_fw_i(regfile_alu_we_fw_i),
		.reg_d_ex_is_reg_a_i(reg_d_ex_is_reg_a_id),
		.reg_d_ex_is_reg_b_i(reg_d_ex_is_reg_b_id),
		.reg_d_ex_is_reg_c_i(reg_d_ex_is_reg_c_id),
		.reg_d_wb_is_reg_a_i(reg_d_wb_is_reg_a_id),
		.reg_d_wb_is_reg_b_i(reg_d_wb_is_reg_b_id),
		.reg_d_wb_is_reg_c_i(reg_d_wb_is_reg_c_id),
		.reg_d_alu_is_reg_a_i(reg_d_alu_is_reg_a_id),
		.reg_d_alu_is_reg_b_i(reg_d_alu_is_reg_b_id),
		.reg_d_alu_is_reg_c_i(reg_d_alu_is_reg_c_id),
		.operand_a_fw_mux_sel_o(operand_a_fw_mux_sel),
		.operand_b_fw_mux_sel_o(operand_b_fw_mux_sel),
		.operand_c_fw_mux_sel_o(operand_c_fw_mux_sel),
		.halt_if_o(halt_if_o),
		.halt_id_o(halt_id),
		.misaligned_stall_o(misaligned_stall),
		.jr_stall_o(jr_stall),
		.load_stall_o(load_stall),
		.id_ready_i(id_ready_o),
		.ex_valid_i(ex_valid_i),
		.wb_ready_i(wb_ready_i),
		.perf_jump_o(perf_jump_o),
		.perf_jr_stall_o(perf_jr_stall_o),
		.perf_ld_stall_o(perf_ld_stall_o),
		.perf_pipeline_stall_o(perf_pipeline_stall_o)
	);
	riscv_int_controller #(.PULP_SECURE(PULP_SECURE)) int_controller_i(
		.clk(clk),
		.rst_n(rst_n),
		.irq_req_ctrl_o(irq_req_ctrl),
		.irq_sec_ctrl_o(irq_sec_ctrl),
		.irq_id_ctrl_o(irq_id_ctrl),
		.ctrl_ack_i(exc_ack),
		.ctrl_kill_i(exc_kill),
		.irq_pending_i(irq_pending_i),
		.irq_sec_i(irq_sec_i),
		.irq_id_i(irq_id_i),
		.m_IE_i(m_irq_enable_i),
		.u_IE_i(u_irq_enable_i),
		.current_priv_lvl_i(current_priv_lvl_i)
	);
	riscv_hwloop_regs #(.N_REGS(N_HWLP)) hwloop_regs_i(
		.clk(clk),
		.rst_n(rst_n),
		.hwlp_start_data_i(hwloop_start),
		.hwlp_end_data_i(hwloop_end),
		.hwlp_cnt_data_i(hwloop_cnt),
		.hwlp_we_i(hwloop_we),
		.hwlp_regid_i(hwloop_regid),
		.valid_i(hwloop_valid),
		.hwlp_start_addr_o(hwlp_start_o),
		.hwlp_end_addr_o(hwlp_end_o),
		.hwlp_counter_o(hwlp_cnt_o),
		.hwlp_dec_cnt_i(hwlp_dec_cnt_i)
	);
	assign hwloop_valid = (instr_valid_i & clear_instr_valid_o) & is_hwlp_i;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	localparam riscv_defines_BRANCH_COND = 2'b11;
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	always @(posedge clk or negedge rst_n) begin : ID_EX_PIPE_REGISTERS
		if (rst_n == 1'b0) begin
			alu_en_ex_o <= 1'sb0;
			alu_operator_ex_o <= riscv_defines_ALU_SLTU;
			alu_operand_a_ex_o <= 1'sb0;
			alu_operand_b_ex_o <= 1'sb0;
			alu_operand_c_ex_o <= 1'sb0;
			bmask_a_ex_o <= 1'sb0;
			bmask_b_ex_o <= 1'sb0;
			imm_vec_ext_ex_o <= 1'sb0;
			alu_vec_mode_ex_o <= 1'sb0;
			alu_clpx_shift_ex_o <= 2'b00;
			alu_is_clpx_ex_o <= 1'b0;
			alu_is_subrot_ex_o <= 1'b0;
			mult_operator_ex_o <= 1'sb0;
			mult_operand_a_ex_o <= 1'sb0;
			mult_operand_b_ex_o <= 1'sb0;
			mult_operand_c_ex_o <= 1'sb0;
			mult_en_ex_o <= 1'b0;
			mult_sel_subword_ex_o <= 1'b0;
			mult_signed_mode_ex_o <= 2'b00;
			mult_imm_ex_o <= 1'sb0;
			mult_dot_op_a_ex_o <= 1'sb0;
			mult_dot_op_b_ex_o <= 1'sb0;
			mult_dot_op_c_ex_o <= 1'sb0;
			mult_dot_signed_ex_o <= 1'sb0;
			mult_is_clpx_ex_o <= 1'b0;
			mult_clpx_shift_ex_o <= 2'b00;
			mult_clpx_img_ex_o <= 1'b0;
			apu_en_ex_o <= 1'sb0;
			apu_type_ex_o <= 1'sb0;
			apu_op_ex_o <= 1'sb0;
			apu_lat_ex_o <= 1'sb0;
			apu_operands_ex_o[0+:32] <= 1'sb0;
			apu_operands_ex_o[32+:32] <= 1'sb0;
			apu_operands_ex_o[64+:32] <= 1'sb0;
			apu_flags_ex_o <= 1'sb0;
			apu_waddr_ex_o <= 1'sb0;
			regfile_waddr_ex_o <= 6'b000000;
			regfile_we_ex_o <= 1'b0;
			regfile_alu_waddr_ex_o <= 6'b000000;
			regfile_alu_we_ex_o <= 1'b0;
			prepost_useincr_ex_o <= 1'b0;
			csr_access_ex_o <= 1'b0;
			csr_op_ex_o <= riscv_defines_CSR_OP_NONE;
			data_we_ex_o <= 1'b0;
			data_type_ex_o <= 2'b00;
			data_sign_ext_ex_o <= 2'b00;
			data_reg_offset_ex_o <= 2'b00;
			data_req_ex_o <= 1'b0;
			data_load_event_ex_o <= 1'b0;
			atop_ex_o <= 5'b00000;
			data_misaligned_ex_o <= 1'b0;
			pc_ex_o <= 1'sb0;
			branch_in_ex_o <= 1'b0;
		end
		else if (data_misaligned_i) begin
			if (ex_ready_i) begin
				if (prepost_useincr_ex_o == 1'b1)
					alu_operand_a_ex_o <= alu_operand_a;
				alu_operand_b_ex_o <= alu_operand_b;
				regfile_alu_we_ex_o <= regfile_alu_we_id;
				prepost_useincr_ex_o <= prepost_useincr;
				data_misaligned_ex_o <= 1'b1;
			end
		end
		else if (mult_multicycle_i)
			mult_operand_c_ex_o <= alu_operand_c;
		else if (id_valid_o) begin
			alu_en_ex_o <= alu_en | branch_taken_ex;
			if (alu_en | branch_taken_ex) begin
				alu_operator_ex_o <= (branch_taken_ex ? riscv_defines_ALU_SLTU : alu_operator);
				if (~branch_taken_ex) begin
					alu_operand_a_ex_o <= alu_operand_a;
					alu_operand_b_ex_o <= alu_operand_b;
					alu_operand_c_ex_o <= alu_operand_c;
					bmask_a_ex_o <= bmask_a_id;
					bmask_b_ex_o <= bmask_b_id;
					imm_vec_ext_ex_o <= imm_vec_ext_id;
					alu_vec_mode_ex_o <= alu_vec_mode;
					alu_is_clpx_ex_o <= is_clpx;
					alu_clpx_shift_ex_o <= instr[14:13];
					alu_is_subrot_ex_o <= is_subrot;
				end
			end
			mult_en_ex_o <= mult_en;
			if (mult_int_en) begin
				mult_operator_ex_o <= mult_operator;
				mult_sel_subword_ex_o <= mult_sel_subword;
				mult_signed_mode_ex_o <= mult_signed_mode;
				mult_operand_a_ex_o <= alu_operand_a;
				mult_operand_b_ex_o <= alu_operand_b;
				mult_operand_c_ex_o <= alu_operand_c;
				mult_imm_ex_o <= mult_imm_id;
			end
			if (mult_dot_en) begin
				mult_operator_ex_o <= mult_operator;
				mult_dot_signed_ex_o <= mult_dot_signed;
				mult_dot_op_a_ex_o <= alu_operand_a;
				mult_dot_op_b_ex_o <= alu_operand_b;
				mult_dot_op_c_ex_o <= alu_operand_c;
				mult_is_clpx_ex_o <= is_clpx;
				mult_clpx_shift_ex_o <= instr[14:13];
				mult_clpx_img_ex_o <= instr[25];
			end
			apu_en_ex_o <= apu_en;
			if (apu_en) begin
				apu_type_ex_o <= apu_type;
				apu_op_ex_o <= apu_op;
				apu_lat_ex_o <= apu_lat;
				apu_operands_ex_o <= apu_operands;
				apu_flags_ex_o <= apu_flags;
				apu_waddr_ex_o <= apu_waddr;
			end
			regfile_we_ex_o <= regfile_we_id;
			if (regfile_we_id)
				regfile_waddr_ex_o <= regfile_waddr_id;
			regfile_alu_we_ex_o <= regfile_alu_we_id;
			if (regfile_alu_we_id)
				regfile_alu_waddr_ex_o <= regfile_alu_waddr_id;
			prepost_useincr_ex_o <= prepost_useincr;
			csr_access_ex_o <= csr_access;
			csr_op_ex_o <= csr_op;
			data_req_ex_o <= data_req_id;
			if (data_req_id) begin
				data_we_ex_o <= data_we_id;
				data_type_ex_o <= data_type_id;
				data_sign_ext_ex_o <= data_sign_ext_id;
				data_reg_offset_ex_o <= data_reg_offset_id;
				data_load_event_ex_o <= data_load_event_id;
				atop_ex_o <= atop_id;
			end
			else
				data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			if ((jump_in_id == riscv_defines_BRANCH_COND) || data_req_id)
				pc_ex_o <= pc_id_i;
			branch_in_ex_o <= jump_in_id == riscv_defines_BRANCH_COND;
		end
		else if (ex_ready_i) begin
			regfile_we_ex_o <= 1'b0;
			regfile_alu_we_ex_o <= 1'b0;
			csr_op_ex_o <= riscv_defines_CSR_OP_NONE;
			data_req_ex_o <= 1'b0;
			data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			branch_in_ex_o <= 1'b0;
			apu_en_ex_o <= 1'b0;
			alu_operator_ex_o <= riscv_defines_ALU_SLTU;
			mult_en_ex_o <= 1'b0;
			alu_en_ex_o <= 1'b1;
		end
		else if (csr_access_ex_o)
			regfile_alu_we_ex_o <= 1'b0;
	end
	assign id_ready_o = ((((~misaligned_stall & ~jr_stall) & ~load_stall) & ~apu_stall) & ~csr_apu_stall) & ex_ready_i;
	assign id_valid_o = ~halt_id & id_ready_o;
	initial _sv2v_0 = 0;
endmodule
module riscv_decoder (
	deassert_we_i,
	data_misaligned_i,
	mult_multicycle_i,
	instr_multicycle_o,
	illegal_insn_o,
	ebrk_insn_o,
	mret_insn_o,
	uret_insn_o,
	dret_insn_o,
	mret_dec_o,
	uret_dec_o,
	dret_dec_o,
	ecall_insn_o,
	pipe_flush_o,
	fencei_insn_o,
	rega_used_o,
	regb_used_o,
	regc_used_o,
	reg_fp_a_o,
	reg_fp_b_o,
	reg_fp_c_o,
	reg_fp_d_o,
	bmask_a_mux_o,
	bmask_b_mux_o,
	alu_bmask_a_mux_sel_o,
	alu_bmask_b_mux_sel_o,
	instr_rdata_i,
	illegal_c_insn_i,
	alu_en_o,
	alu_operator_o,
	alu_op_a_mux_sel_o,
	alu_op_b_mux_sel_o,
	alu_op_c_mux_sel_o,
	alu_vec_mode_o,
	scalar_replication_o,
	scalar_replication_c_o,
	imm_a_mux_sel_o,
	imm_b_mux_sel_o,
	regc_mux_o,
	is_clpx_o,
	is_subrot_o,
	mult_operator_o,
	mult_int_en_o,
	mult_dot_en_o,
	mult_imm_mux_o,
	mult_sel_subword_o,
	mult_signed_mode_o,
	mult_dot_signed_o,
	frm_i,
	fpu_dst_fmt_o,
	fpu_src_fmt_o,
	fpu_int_fmt_o,
	apu_en_o,
	apu_type_o,
	apu_op_o,
	apu_lat_o,
	apu_flags_src_o,
	fp_rnd_mode_o,
	regfile_mem_we_o,
	regfile_alu_we_o,
	regfile_alu_we_dec_o,
	regfile_alu_waddr_sel_o,
	csr_access_o,
	csr_status_o,
	csr_op_o,
	current_priv_lvl_i,
	data_req_o,
	data_we_o,
	prepost_useincr_o,
	data_type_o,
	data_sign_extension_o,
	data_reg_offset_o,
	data_load_event_o,
	atop_o,
	hwloop_we_o,
	hwloop_target_mux_sel_o,
	hwloop_start_mux_sel_o,
	hwloop_cnt_mux_sel_o,
	jump_in_dec_o,
	jump_in_id_o,
	jump_target_mux_sel_o
);
	reg _sv2v_0;
	parameter A_EXTENSION = 0;
	parameter FPU = 0;
	parameter FP_DIVSQRT = 0;
	parameter PULP_SECURE = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter SHARED_FP_DIVSQRT = 0;
	parameter WAPUTYPE = 0;
	parameter APU_WOP_CPU = 6;
	input wire deassert_we_i;
	input wire data_misaligned_i;
	input wire mult_multicycle_i;
	output reg instr_multicycle_o;
	output reg illegal_insn_o;
	output reg ebrk_insn_o;
	output reg mret_insn_o;
	output reg uret_insn_o;
	output reg dret_insn_o;
	output reg mret_dec_o;
	output reg uret_dec_o;
	output reg dret_dec_o;
	output reg ecall_insn_o;
	output reg pipe_flush_o;
	output reg fencei_insn_o;
	output reg rega_used_o;
	output reg regb_used_o;
	output reg regc_used_o;
	output reg reg_fp_a_o;
	output reg reg_fp_b_o;
	output reg reg_fp_c_o;
	output reg reg_fp_d_o;
	output reg [0:0] bmask_a_mux_o;
	output reg [1:0] bmask_b_mux_o;
	output reg alu_bmask_a_mux_sel_o;
	output reg alu_bmask_b_mux_sel_o;
	input wire [31:0] instr_rdata_i;
	input wire illegal_c_insn_i;
	output reg alu_en_o;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	output reg [6:0] alu_operator_o;
	output reg [2:0] alu_op_a_mux_sel_o;
	output reg [2:0] alu_op_b_mux_sel_o;
	output reg [1:0] alu_op_c_mux_sel_o;
	output reg [1:0] alu_vec_mode_o;
	output reg scalar_replication_o;
	output reg scalar_replication_c_o;
	output reg [0:0] imm_a_mux_sel_o;
	output reg [3:0] imm_b_mux_sel_o;
	output reg [1:0] regc_mux_o;
	output reg is_clpx_o;
	output reg is_subrot_o;
	output reg [2:0] mult_operator_o;
	output wire mult_int_en_o;
	output wire mult_dot_en_o;
	output reg [0:0] mult_imm_mux_o;
	output reg mult_sel_subword_o;
	output reg [1:0] mult_signed_mode_o;
	output reg [1:0] mult_dot_signed_o;
	localparam riscv_defines_C_RM = 3;
	input wire [2:0] frm_i;
	localparam [31:0] fpnew_pkg_NUM_FP_FORMATS = 5;
	localparam [31:0] fpnew_pkg_FP_FORMAT_BITS = 3;
	localparam riscv_defines_C_FPNEW_FMTBITS = fpnew_pkg_FP_FORMAT_BITS;
	output reg [2:0] fpu_dst_fmt_o;
	output reg [2:0] fpu_src_fmt_o;
	localparam [31:0] fpnew_pkg_NUM_INT_FORMATS = 4;
	localparam [31:0] fpnew_pkg_INT_FORMAT_BITS = 2;
	localparam riscv_defines_C_FPNEW_IFMTBITS = fpnew_pkg_INT_FORMAT_BITS;
	output reg [1:0] fpu_int_fmt_o;
	output wire apu_en_o;
	output reg [WAPUTYPE - 1:0] apu_type_o;
	output reg [APU_WOP_CPU - 1:0] apu_op_o;
	output reg [1:0] apu_lat_o;
	output reg [WAPUTYPE - 1:0] apu_flags_src_o;
	output reg [2:0] fp_rnd_mode_o;
	output wire regfile_mem_we_o;
	output wire regfile_alu_we_o;
	output wire regfile_alu_we_dec_o;
	output reg regfile_alu_waddr_sel_o;
	output reg csr_access_o;
	output reg csr_status_o;
	output wire [1:0] csr_op_o;
	input wire [1:0] current_priv_lvl_i;
	output wire data_req_o;
	output reg data_we_o;
	output reg prepost_useincr_o;
	output reg [1:0] data_type_o;
	output reg [1:0] data_sign_extension_o;
	output reg [1:0] data_reg_offset_o;
	output reg data_load_event_o;
	output reg [5:0] atop_o;
	output wire [2:0] hwloop_we_o;
	output reg hwloop_target_mux_sel_o;
	output reg hwloop_start_mux_sel_o;
	output reg hwloop_cnt_mux_sel_o;
	output wire [1:0] jump_in_dec_o;
	output wire [1:0] jump_in_id_o;
	output reg [1:0] jump_target_mux_sel_o;
	localparam APUTYPE_DSP_MULT = (SHARED_DSP_MULT ? 0 : 0);
	localparam APUTYPE_INT_MULT = (SHARED_INT_MULT ? SHARED_DSP_MULT : 0);
	localparam APUTYPE_INT_DIV = (SHARED_INT_DIV ? SHARED_DSP_MULT + SHARED_INT_MULT : 0);
	localparam APUTYPE_FP = (SHARED_FP ? (SHARED_DSP_MULT + SHARED_INT_MULT) + SHARED_INT_DIV : 0);
	localparam APUTYPE_ADDSUB = (SHARED_FP ? (SHARED_FP == 1 ? APUTYPE_FP : APUTYPE_FP) : 0);
	localparam APUTYPE_MULT = (SHARED_FP ? (SHARED_FP == 1 ? APUTYPE_FP + 1 : APUTYPE_FP) : 0);
	localparam APUTYPE_CAST = (SHARED_FP ? (SHARED_FP == 1 ? APUTYPE_FP + 2 : APUTYPE_FP) : 0);
	localparam APUTYPE_MAC = (SHARED_FP ? (SHARED_FP == 1 ? APUTYPE_FP + 3 : APUTYPE_FP) : 0);
	localparam APUTYPE_DIV = (SHARED_FP_DIVSQRT == 1 ? (SHARED_FP == 1 ? APUTYPE_FP + 4 : APUTYPE_FP) : (SHARED_FP_DIVSQRT == 2 ? (SHARED_FP == 1 ? APUTYPE_FP + 4 : APUTYPE_FP + 1) : 0));
	localparam APUTYPE_SQRT = (SHARED_FP_DIVSQRT == 1 ? (SHARED_FP == 1 ? APUTYPE_FP + 5 : APUTYPE_FP) : (SHARED_FP_DIVSQRT == 2 ? (SHARED_FP == 1 ? APUTYPE_FP + 4 : APUTYPE_FP + 1) : 0));
	reg regfile_mem_we;
	reg regfile_alu_we;
	reg data_req;
	reg [2:0] hwloop_we;
	reg csr_illegal;
	reg [1:0] jump_in_id;
	reg [1:0] csr_op;
	reg mult_int_en;
	reg mult_dot_en;
	reg apu_en;
	reg check_fprm;
	localparam [31:0] fpnew_pkg_OP_BITS = 4;
	localparam riscv_defines_C_FPNEW_OPBITS = fpnew_pkg_OP_BITS;
	reg [3:0] fpu_op;
	reg fpu_op_mod;
	reg fpu_vec_op;
	reg [1:0] fp_op_group;
	localparam apu_core_package_APU_FLAGS_DSP_MULT = 0;
	localparam apu_core_package_APU_FLAGS_FP = 2;
	localparam apu_core_package_APU_FLAGS_FPNEW = 3;
	localparam apu_core_package_APU_FLAGS_INT_MULT = 1;
	localparam apu_core_package_PIPE_REG_ADDSUB = 1;
	localparam apu_core_package_PIPE_REG_CAST = 1;
	localparam apu_core_package_PIPE_REG_DSP_MULT = 1;
	localparam apu_core_package_PIPE_REG_MAC = 2;
	localparam apu_core_package_PIPE_REG_MULT = 1;
	localparam apu_core_package_SHARED_INT_MULT = 0;
	localparam riscv_defines_ALU_ABS = 7'b0010100;
	localparam riscv_defines_ALU_ADD = 7'b0011000;
	localparam riscv_defines_ALU_ADDR = 7'b0011100;
	localparam riscv_defines_ALU_ADDU = 7'b0011010;
	localparam riscv_defines_ALU_ADDUR = 7'b0011110;
	localparam riscv_defines_ALU_AND = 7'b0010101;
	localparam riscv_defines_ALU_BCLR = 7'b0101011;
	localparam riscv_defines_ALU_BEXT = 7'b0101000;
	localparam riscv_defines_ALU_BEXTU = 7'b0101001;
	localparam riscv_defines_ALU_BINS = 7'b0101010;
	localparam riscv_defines_ALU_BREV = 7'b1001001;
	localparam riscv_defines_ALU_BSET = 7'b0101100;
	localparam riscv_defines_ALU_CLB = 7'b0110101;
	localparam riscv_defines_ALU_CLIP = 7'b0010110;
	localparam riscv_defines_ALU_CLIPU = 7'b0010111;
	localparam riscv_defines_ALU_CNT = 7'b0110100;
	localparam riscv_defines_ALU_DIV = 7'b0110001;
	localparam riscv_defines_ALU_DIVU = 7'b0110000;
	localparam riscv_defines_ALU_EQ = 7'b0001100;
	localparam riscv_defines_ALU_EXT = 7'b0111111;
	localparam riscv_defines_ALU_EXTS = 7'b0111110;
	localparam riscv_defines_ALU_FCLASS = 7'b1001000;
	localparam riscv_defines_ALU_FEQ = 7'b1000011;
	localparam riscv_defines_ALU_FF1 = 7'b0110110;
	localparam riscv_defines_ALU_FKEEP = 7'b1111111;
	localparam riscv_defines_ALU_FL1 = 7'b0110111;
	localparam riscv_defines_ALU_FLE = 7'b1000101;
	localparam riscv_defines_ALU_FLT = 7'b1000100;
	localparam riscv_defines_ALU_FMAX = 7'b1000110;
	localparam riscv_defines_ALU_FMIN = 7'b1000111;
	localparam riscv_defines_ALU_FSGNJ = 7'b1000000;
	localparam riscv_defines_ALU_FSGNJN = 7'b1000001;
	localparam riscv_defines_ALU_FSGNJX = 7'b1000010;
	localparam riscv_defines_ALU_GES = 7'b0001010;
	localparam riscv_defines_ALU_GEU = 7'b0001011;
	localparam riscv_defines_ALU_GTS = 7'b0001000;
	localparam riscv_defines_ALU_GTU = 7'b0001001;
	localparam riscv_defines_ALU_INS = 7'b0101101;
	localparam riscv_defines_ALU_LES = 7'b0000100;
	localparam riscv_defines_ALU_LEU = 7'b0000101;
	localparam riscv_defines_ALU_LTS = 7'b0000000;
	localparam riscv_defines_ALU_LTU = 7'b0000001;
	localparam riscv_defines_ALU_MAX = 7'b0010010;
	localparam riscv_defines_ALU_MAXU = 7'b0010011;
	localparam riscv_defines_ALU_MIN = 7'b0010000;
	localparam riscv_defines_ALU_MINU = 7'b0010001;
	localparam riscv_defines_ALU_NE = 7'b0001101;
	localparam riscv_defines_ALU_OR = 7'b0101110;
	localparam riscv_defines_ALU_PCKHI = 7'b0111001;
	localparam riscv_defines_ALU_PCKLO = 7'b0111000;
	localparam riscv_defines_ALU_REM = 7'b0110011;
	localparam riscv_defines_ALU_REMU = 7'b0110010;
	localparam riscv_defines_ALU_ROR = 7'b0100110;
	localparam riscv_defines_ALU_SHUF = 7'b0111010;
	localparam riscv_defines_ALU_SHUF2 = 7'b0111011;
	localparam riscv_defines_ALU_SLETS = 7'b0000110;
	localparam riscv_defines_ALU_SLETU = 7'b0000111;
	localparam riscv_defines_ALU_SLL = 7'b0100111;
	localparam riscv_defines_ALU_SLTS = 7'b0000010;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	localparam riscv_defines_ALU_SRA = 7'b0100100;
	localparam riscv_defines_ALU_SRL = 7'b0100101;
	localparam riscv_defines_ALU_SUB = 7'b0011001;
	localparam riscv_defines_ALU_SUBR = 7'b0011101;
	localparam riscv_defines_ALU_SUBU = 7'b0011011;
	localparam riscv_defines_ALU_SUBUR = 7'b0011111;
	localparam riscv_defines_ALU_XOR = 7'b0101111;
	localparam riscv_defines_AMO_ADD = 5'b00000;
	localparam riscv_defines_AMO_AND = 5'b01100;
	localparam riscv_defines_AMO_LR = 5'b00010;
	localparam riscv_defines_AMO_MAX = 5'b10100;
	localparam riscv_defines_AMO_MAXU = 5'b11100;
	localparam riscv_defines_AMO_MIN = 5'b10000;
	localparam riscv_defines_AMO_MINU = 5'b11000;
	localparam riscv_defines_AMO_OR = 5'b01000;
	localparam riscv_defines_AMO_SC = 5'b00011;
	localparam riscv_defines_AMO_SWAP = 5'b00001;
	localparam riscv_defines_AMO_XOR = 5'b00100;
	localparam riscv_defines_BMASK_A_IMM = 1'b1;
	localparam riscv_defines_BMASK_A_REG = 1'b0;
	localparam riscv_defines_BMASK_A_S3 = 1'b1;
	localparam riscv_defines_BMASK_A_ZERO = 1'b0;
	localparam riscv_defines_BMASK_B_IMM = 1'b1;
	localparam riscv_defines_BMASK_B_ONE = 2'b11;
	localparam riscv_defines_BMASK_B_REG = 1'b0;
	localparam riscv_defines_BMASK_B_S2 = 2'b00;
	localparam riscv_defines_BMASK_B_S3 = 2'b01;
	localparam riscv_defines_BMASK_B_ZERO = 2'b10;
	localparam riscv_defines_BRANCH_COND = 2'b11;
	localparam riscv_defines_BRANCH_JAL = 2'b01;
	localparam riscv_defines_BRANCH_JALR = 2'b10;
	localparam riscv_defines_BRANCH_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_CLEAR = 2'b11;
	localparam riscv_defines_CSR_OP_NONE = 2'b00;
	localparam riscv_defines_CSR_OP_SET = 2'b10;
	localparam riscv_defines_CSR_OP_WRITE = 2'b01;
	localparam [31:0] riscv_defines_C_LAT_CONV = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP16 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP16ALT = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP32 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP64 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP8 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_NONCOMP = 'd5;
	localparam [0:0] riscv_defines_C_RVD = 1'b0;
	localparam [0:0] riscv_defines_C_RVF = 1'b1;
	localparam [0:0] riscv_defines_C_XF16 = 1'b0;
	localparam [0:0] riscv_defines_C_XF16ALT = 1'b0;
	localparam [0:0] riscv_defines_C_XF8 = 1'b0;
	localparam [0:0] riscv_defines_C_XFVEC = 1'b0;
	localparam riscv_defines_IMMA_Z = 1'b0;
	localparam riscv_defines_IMMA_ZERO = 1'b1;
	localparam riscv_defines_IMMB_BI = 4'b1011;
	localparam riscv_defines_IMMB_CLIP = 4'b1001;
	localparam riscv_defines_IMMB_I = 4'b0000;
	localparam riscv_defines_IMMB_PCINCR = 4'b0011;
	localparam riscv_defines_IMMB_S = 4'b0001;
	localparam riscv_defines_IMMB_S2 = 4'b0100;
	localparam riscv_defines_IMMB_SHUF = 4'b1000;
	localparam riscv_defines_IMMB_U = 4'b0010;
	localparam riscv_defines_IMMB_VS = 4'b0110;
	localparam riscv_defines_IMMB_VU = 4'b0111;
	localparam riscv_defines_JT_COND = 2'b11;
	localparam riscv_defines_JT_JAL = 2'b01;
	localparam riscv_defines_JT_JALR = 2'b10;
	localparam riscv_defines_MIMM_S3 = 1'b1;
	localparam riscv_defines_MIMM_ZERO = 1'b0;
	localparam riscv_defines_MUL_DOT16 = 3'b101;
	localparam riscv_defines_MUL_DOT8 = 3'b100;
	localparam riscv_defines_MUL_H = 3'b110;
	localparam riscv_defines_MUL_I = 3'b010;
	localparam riscv_defines_MUL_IR = 3'b011;
	localparam riscv_defines_MUL_MAC32 = 3'b000;
	localparam riscv_defines_MUL_MSU32 = 3'b001;
	localparam riscv_defines_OPCODE_AMO = 7'h2f;
	localparam riscv_defines_OPCODE_AUIPC = 7'h17;
	localparam riscv_defines_OPCODE_BRANCH = 7'h63;
	localparam riscv_defines_OPCODE_FENCE = 7'h0f;
	localparam riscv_defines_OPCODE_HWLOOP = 7'h7b;
	localparam riscv_defines_OPCODE_JAL = 7'h6f;
	localparam riscv_defines_OPCODE_JALR = 7'h67;
	localparam riscv_defines_OPCODE_LOAD = 7'h03;
	localparam riscv_defines_OPCODE_LOAD_FP = 7'h07;
	localparam riscv_defines_OPCODE_LOAD_POST = 7'h0b;
	localparam riscv_defines_OPCODE_LUI = 7'h37;
	localparam riscv_defines_OPCODE_OP = 7'h33;
	localparam riscv_defines_OPCODE_OPIMM = 7'h13;
	localparam riscv_defines_OPCODE_OP_FMADD = 7'h43;
	localparam riscv_defines_OPCODE_OP_FMSUB = 7'h47;
	localparam riscv_defines_OPCODE_OP_FNMADD = 7'h4f;
	localparam riscv_defines_OPCODE_OP_FNMSUB = 7'h4b;
	localparam riscv_defines_OPCODE_OP_FP = 7'h53;
	localparam riscv_defines_OPCODE_PULP_OP = 7'h5b;
	localparam riscv_defines_OPCODE_STORE = 7'h23;
	localparam riscv_defines_OPCODE_STORE_FP = 7'h27;
	localparam riscv_defines_OPCODE_STORE_POST = 7'h2b;
	localparam riscv_defines_OPCODE_SYSTEM = 7'h73;
	localparam riscv_defines_OPCODE_VECOP = 7'h57;
	localparam riscv_defines_OP_A_CURRPC = 3'b001;
	localparam riscv_defines_OP_A_IMM = 3'b010;
	localparam riscv_defines_OP_A_REGA_OR_FWD = 3'b000;
	localparam riscv_defines_OP_A_REGB_OR_FWD = 3'b011;
	localparam riscv_defines_OP_A_REGC_OR_FWD = 3'b100;
	localparam riscv_defines_OP_B_BMASK = 3'b100;
	localparam riscv_defines_OP_B_IMM = 3'b010;
	localparam riscv_defines_OP_B_REGA_OR_FWD = 3'b011;
	localparam riscv_defines_OP_B_REGB_OR_FWD = 3'b000;
	localparam riscv_defines_OP_B_REGC_OR_FWD = 3'b001;
	localparam riscv_defines_OP_C_JT = 2'b10;
	localparam riscv_defines_OP_C_REGB_OR_FWD = 2'b01;
	localparam riscv_defines_OP_C_REGC_OR_FWD = 2'b00;
	localparam riscv_defines_REGC_RD = 2'b01;
	localparam riscv_defines_REGC_S1 = 2'b10;
	localparam riscv_defines_REGC_S4 = 2'b00;
	localparam riscv_defines_REGC_ZERO = 2'b11;
	localparam riscv_defines_VEC_MODE16 = 2'b10;
	localparam riscv_defines_VEC_MODE32 = 2'b00;
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	function automatic [3:0] sv2v_cast_A53F3;
		input reg [3:0] inp;
		sv2v_cast_A53F3 = inp;
	endfunction
	function automatic [2:0] sv2v_cast_0BC43;
		input reg [2:0] inp;
		sv2v_cast_0BC43 = inp;
	endfunction
	function automatic [1:0] sv2v_cast_87CC5;
		input reg [1:0] inp;
		sv2v_cast_87CC5 = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		jump_in_id = riscv_defines_BRANCH_NONE;
		jump_target_mux_sel_o = riscv_defines_JT_JAL;
		alu_en_o = 1'b1;
		alu_operator_o = riscv_defines_ALU_SLTU;
		alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
		alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
		alu_op_c_mux_sel_o = riscv_defines_OP_C_REGC_OR_FWD;
		alu_vec_mode_o = riscv_defines_VEC_MODE32;
		scalar_replication_o = 1'b0;
		scalar_replication_c_o = 1'b0;
		regc_mux_o = riscv_defines_REGC_ZERO;
		imm_a_mux_sel_o = riscv_defines_IMMA_ZERO;
		imm_b_mux_sel_o = riscv_defines_IMMB_I;
		mult_operator_o = riscv_defines_MUL_I;
		mult_int_en = 1'b0;
		mult_dot_en = 1'b0;
		mult_imm_mux_o = riscv_defines_MIMM_ZERO;
		mult_signed_mode_o = 2'b00;
		mult_sel_subword_o = 1'b0;
		mult_dot_signed_o = 2'b00;
		apu_en = 1'b0;
		apu_type_o = 1'sb0;
		apu_op_o = 1'sb0;
		apu_lat_o = 1'sb0;
		apu_flags_src_o = 1'sb0;
		fp_rnd_mode_o = 1'sb0;
		fpu_op = sv2v_cast_A53F3(6);
		fpu_op_mod = 1'b0;
		fpu_vec_op = 1'b0;
		fpu_dst_fmt_o = sv2v_cast_0BC43('d0);
		fpu_src_fmt_o = sv2v_cast_0BC43('d0);
		fpu_int_fmt_o = sv2v_cast_87CC5(2);
		check_fprm = 1'b0;
		fp_op_group = 2'd0;
		regfile_mem_we = 1'b0;
		regfile_alu_we = 1'b0;
		regfile_alu_waddr_sel_o = 1'b1;
		prepost_useincr_o = 1'b1;
		hwloop_we = 3'b000;
		hwloop_target_mux_sel_o = 1'b0;
		hwloop_start_mux_sel_o = 1'b0;
		hwloop_cnt_mux_sel_o = 1'b0;
		csr_access_o = 1'b0;
		csr_status_o = 1'b0;
		csr_illegal = 1'b0;
		csr_op = riscv_defines_CSR_OP_NONE;
		mret_insn_o = 1'b0;
		uret_insn_o = 1'b0;
		dret_insn_o = 1'b0;
		data_we_o = 1'b0;
		data_type_o = 2'b00;
		data_sign_extension_o = 2'b00;
		data_reg_offset_o = 2'b00;
		data_req = 1'b0;
		data_load_event_o = 1'b0;
		atop_o = 6'b000000;
		illegal_insn_o = 1'b0;
		ebrk_insn_o = 1'b0;
		ecall_insn_o = 1'b0;
		pipe_flush_o = 1'b0;
		fencei_insn_o = 1'b0;
		rega_used_o = 1'b0;
		regb_used_o = 1'b0;
		regc_used_o = 1'b0;
		reg_fp_a_o = 1'b0;
		reg_fp_b_o = 1'b0;
		reg_fp_c_o = 1'b0;
		reg_fp_d_o = 1'b0;
		bmask_a_mux_o = riscv_defines_BMASK_A_ZERO;
		bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
		alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_IMM;
		alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_IMM;
		instr_multicycle_o = 1'b0;
		is_clpx_o = 1'b0;
		is_subrot_o = 1'b0;
		mret_dec_o = 1'b0;
		uret_dec_o = 1'b0;
		dret_dec_o = 1'b0;
		case (instr_rdata_i[6:0])
			riscv_defines_OPCODE_JAL: begin
				jump_target_mux_sel_o = riscv_defines_JT_JAL;
				jump_in_id = riscv_defines_BRANCH_JAL;
				alu_op_a_mux_sel_o = riscv_defines_OP_A_CURRPC;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_PCINCR;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
			end
			riscv_defines_OPCODE_JALR: begin
				jump_target_mux_sel_o = riscv_defines_JT_JALR;
				jump_in_id = riscv_defines_BRANCH_JALR;
				alu_op_a_mux_sel_o = riscv_defines_OP_A_CURRPC;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_PCINCR;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				if (instr_rdata_i[14:12] != 3'b000) begin
					jump_in_id = riscv_defines_BRANCH_NONE;
					regfile_alu_we = 1'b0;
					illegal_insn_o = 1'b1;
				end
			end
			riscv_defines_OPCODE_BRANCH: begin
				jump_target_mux_sel_o = riscv_defines_JT_COND;
				jump_in_id = riscv_defines_BRANCH_COND;
				alu_op_c_mux_sel_o = riscv_defines_OP_C_JT;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				case (instr_rdata_i[14:12])
					3'b000: alu_operator_o = riscv_defines_ALU_EQ;
					3'b001: alu_operator_o = riscv_defines_ALU_NE;
					3'b100: alu_operator_o = riscv_defines_ALU_LTS;
					3'b101: alu_operator_o = riscv_defines_ALU_GES;
					3'b110: alu_operator_o = riscv_defines_ALU_LTU;
					3'b111: alu_operator_o = riscv_defines_ALU_GEU;
					3'b010: begin
						alu_operator_o = riscv_defines_ALU_EQ;
						regb_used_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
						imm_b_mux_sel_o = riscv_defines_IMMB_BI;
					end
					3'b011: begin
						alu_operator_o = riscv_defines_ALU_NE;
						regb_used_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
						imm_b_mux_sel_o = riscv_defines_IMMB_BI;
					end
				endcase
			end
			riscv_defines_OPCODE_STORE, riscv_defines_OPCODE_STORE_POST: begin
				data_req = 1'b1;
				data_we_o = 1'b1;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				alu_operator_o = riscv_defines_ALU_ADD;
				instr_multicycle_o = 1'b1;
				alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
				if (instr_rdata_i[6:0] == riscv_defines_OPCODE_STORE_POST) begin
					prepost_useincr_o = 1'b0;
					regfile_alu_waddr_sel_o = 1'b0;
					regfile_alu_we = 1'b1;
				end
				if (instr_rdata_i[14] == 1'b0) begin
					imm_b_mux_sel_o = riscv_defines_IMMB_S;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				end
				else begin
					regc_used_o = 1'b1;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
					regc_mux_o = riscv_defines_REGC_RD;
				end
				case (instr_rdata_i[13:12])
					2'b00: data_type_o = 2'b10;
					2'b01: data_type_o = 2'b01;
					2'b10: data_type_o = 2'b00;
					default: begin
						data_req = 1'b0;
						data_we_o = 1'b0;
						illegal_insn_o = 1'b1;
					end
				endcase
			end
			riscv_defines_OPCODE_LOAD, riscv_defines_OPCODE_LOAD_POST: begin
				data_req = 1'b1;
				regfile_mem_we = 1'b1;
				rega_used_o = 1'b1;
				data_type_o = 2'b00;
				instr_multicycle_o = 1'b1;
				alu_operator_o = riscv_defines_ALU_ADD;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_I;
				if (instr_rdata_i[6:0] == riscv_defines_OPCODE_LOAD_POST) begin
					prepost_useincr_o = 1'b0;
					regfile_alu_waddr_sel_o = 1'b0;
					regfile_alu_we = 1'b1;
				end
				data_sign_extension_o = {1'b0, ~instr_rdata_i[14]};
				case (instr_rdata_i[13:12])
					2'b00: data_type_o = 2'b10;
					2'b01: data_type_o = 2'b01;
					2'b10: data_type_o = 2'b00;
					default: data_type_o = 2'b00;
				endcase
				if (instr_rdata_i[14:12] == 3'b111) begin
					regb_used_o = 1'b1;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
					data_sign_extension_o = {1'b0, ~instr_rdata_i[30]};
					case (instr_rdata_i[31:25])
						7'b0000000, 7'b0100000: data_type_o = 2'b10;
						7'b0001000, 7'b0101000: data_type_o = 2'b01;
						7'b0010000: data_type_o = 2'b00;
						default: illegal_insn_o = 1'b1;
					endcase
				end
				if (instr_rdata_i[14:12] == 3'b110)
					data_load_event_o = 1'b1;
				if (instr_rdata_i[14:12] == 3'b011)
					illegal_insn_o = 1'b1;
			end
			riscv_defines_OPCODE_AMO:
				if (A_EXTENSION) begin : decode_amo
					if (instr_rdata_i[14:12] == 3'b010) begin
						data_req = 1'b1;
						data_type_o = 2'b00;
						rega_used_o = 1'b1;
						regb_used_o = 1'b1;
						regfile_mem_we = 1'b1;
						prepost_useincr_o = 1'b0;
						alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
						data_sign_extension_o = 1'b1;
						atop_o = {1'b1, instr_rdata_i[31:27]};
						case (instr_rdata_i[31:27])
							riscv_defines_AMO_LR: data_we_o = 1'b0;
							riscv_defines_AMO_SC, riscv_defines_AMO_SWAP, riscv_defines_AMO_ADD, riscv_defines_AMO_XOR, riscv_defines_AMO_AND, riscv_defines_AMO_OR, riscv_defines_AMO_MIN, riscv_defines_AMO_MAX, riscv_defines_AMO_MINU, riscv_defines_AMO_MAXU: begin
								data_we_o = 1'b1;
								alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
							end
							default: illegal_insn_o = 1'b1;
						endcase
					end
					else
						illegal_insn_o = 1'b1;
				end
				else begin : no_decode_amo
					illegal_insn_o = 1'b1;
				end
			riscv_defines_OPCODE_LUI: begin
				alu_op_a_mux_sel_o = riscv_defines_OP_A_IMM;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_a_mux_sel_o = riscv_defines_IMMA_ZERO;
				imm_b_mux_sel_o = riscv_defines_IMMB_U;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
			end
			riscv_defines_OPCODE_AUIPC: begin
				alu_op_a_mux_sel_o = riscv_defines_OP_A_CURRPC;
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_U;
				alu_operator_o = riscv_defines_ALU_ADD;
				regfile_alu_we = 1'b1;
			end
			riscv_defines_OPCODE_OPIMM: begin
				alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
				imm_b_mux_sel_o = riscv_defines_IMMB_I;
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				case (instr_rdata_i[14:12])
					3'b000: alu_operator_o = riscv_defines_ALU_ADD;
					3'b010: alu_operator_o = riscv_defines_ALU_SLTS;
					3'b011: alu_operator_o = riscv_defines_ALU_SLTU;
					3'b100: alu_operator_o = riscv_defines_ALU_XOR;
					3'b110: alu_operator_o = riscv_defines_ALU_OR;
					3'b111: alu_operator_o = riscv_defines_ALU_AND;
					3'b001: begin
						alu_operator_o = riscv_defines_ALU_SLL;
						if (instr_rdata_i[31:25] != 7'b0000000)
							illegal_insn_o = 1'b1;
					end
					3'b101:
						if (instr_rdata_i[31:25] == 7'b0000000)
							alu_operator_o = riscv_defines_ALU_SRL;
						else if (instr_rdata_i[31:25] == 7'b0100000)
							alu_operator_o = riscv_defines_ALU_SRA;
						else
							illegal_insn_o = 1'b1;
				endcase
			end
			riscv_defines_OPCODE_OP:
				if (instr_rdata_i[31:30] == 2'b11) begin
					regfile_alu_we = 1'b1;
					rega_used_o = 1'b1;
					bmask_a_mux_o = riscv_defines_BMASK_A_S3;
					bmask_b_mux_o = riscv_defines_BMASK_B_S2;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					case (instr_rdata_i[14:12])
						3'b000: begin
							alu_operator_o = riscv_defines_ALU_BEXT;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
						end
						3'b001: begin
							alu_operator_o = riscv_defines_ALU_BEXTU;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
						end
						3'b010: begin
							alu_operator_o = riscv_defines_ALU_BINS;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
						end
						3'b011: alu_operator_o = riscv_defines_ALU_BCLR;
						3'b100: alu_operator_o = riscv_defines_ALU_BSET;
						3'b101: begin
							alu_operator_o = riscv_defines_ALU_BREV;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							imm_b_mux_sel_o = riscv_defines_IMMB_S2;
							alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_IMM;
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
				else if (instr_rdata_i[31:30] == 2'b10) begin
					if (instr_rdata_i[29:25] == 5'b00000) begin
						regfile_alu_we = 1'b1;
						rega_used_o = 1'b1;
						bmask_a_mux_o = riscv_defines_BMASK_A_S3;
						bmask_b_mux_o = riscv_defines_BMASK_B_S2;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
						case (instr_rdata_i[14:12])
							3'b000: begin
								alu_operator_o = riscv_defines_ALU_BEXT;
								imm_b_mux_sel_o = riscv_defines_IMMB_S2;
								bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
								alu_op_b_mux_sel_o = riscv_defines_OP_B_BMASK;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								regb_used_o = 1'b1;
							end
							3'b001: begin
								alu_operator_o = riscv_defines_ALU_BEXTU;
								imm_b_mux_sel_o = riscv_defines_IMMB_S2;
								bmask_b_mux_o = riscv_defines_BMASK_B_ZERO;
								alu_op_b_mux_sel_o = riscv_defines_OP_B_BMASK;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								regb_used_o = 1'b1;
							end
							3'b010: begin
								alu_operator_o = riscv_defines_ALU_BINS;
								imm_b_mux_sel_o = riscv_defines_IMMB_S2;
								regc_used_o = 1'b1;
								regc_mux_o = riscv_defines_REGC_RD;
								alu_op_b_mux_sel_o = riscv_defines_OP_B_BMASK;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
								regb_used_o = 1'b1;
							end
							3'b011: begin
								alu_operator_o = riscv_defines_ALU_BCLR;
								regb_used_o = 1'b1;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							end
							3'b100: begin
								alu_operator_o = riscv_defines_ALU_BSET;
								regb_used_o = 1'b1;
								alu_bmask_a_mux_sel_o = riscv_defines_BMASK_A_REG;
								alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							end
							default: illegal_insn_o = 1'b1;
						endcase
					end
					else if (((FPU == 1) && riscv_defines_C_XFVEC) && (SHARED_FP != 1)) begin
						apu_en = 1'b1;
						alu_en_o = 1'b0;
						apu_flags_src_o = apu_core_package_APU_FLAGS_FPNEW;
						rega_used_o = 1'b1;
						regb_used_o = 1'b1;
						reg_fp_a_o = 1'b1;
						reg_fp_b_o = 1'b1;
						reg_fp_d_o = 1'b1;
						fpu_vec_op = 1'b1;
						scalar_replication_o = instr_rdata_i[14];
						check_fprm = 1'b1;
						fp_rnd_mode_o = frm_i;
						case (instr_rdata_i[13:12])
							2'b00: begin
								fpu_dst_fmt_o = sv2v_cast_0BC43('d0);
								alu_vec_mode_o = riscv_defines_VEC_MODE32;
							end
							2'b01: begin
								fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
								alu_vec_mode_o = riscv_defines_VEC_MODE16;
							end
							2'b10: begin
								fpu_dst_fmt_o = sv2v_cast_0BC43('d2);
								alu_vec_mode_o = riscv_defines_VEC_MODE16;
							end
							2'b11: begin
								fpu_dst_fmt_o = sv2v_cast_0BC43('d3);
								alu_vec_mode_o = riscv_defines_VEC_MODE8;
							end
						endcase
						fpu_src_fmt_o = fpu_dst_fmt_o;
						if (instr_rdata_i[29:25] == 5'b00001) begin
							fpu_op = sv2v_cast_A53F3(2);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_ADDSUB;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
							alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
							scalar_replication_o = 1'b0;
							scalar_replication_c_o = instr_rdata_i[14];
						end
						else if (instr_rdata_i[29:25] == 5'b00010) begin
							fpu_op = sv2v_cast_A53F3(2);
							fpu_op_mod = 1'b1;
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_ADDSUB;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
							alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
							scalar_replication_o = 1'b0;
							scalar_replication_c_o = instr_rdata_i[14];
						end
						else if (instr_rdata_i[29:25] == 5'b00011) begin
							fpu_op = sv2v_cast_A53F3(3);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_MULT;
						end
						else if (instr_rdata_i[29:25] == 5'b00100) begin
							if (FP_DIVSQRT) begin
								fpu_op = sv2v_cast_A53F3(4);
								fp_op_group = 2'd1;
								apu_type_o = APUTYPE_DIV;
							end
							else
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[29:25] == 5'b00101) begin
							fpu_op = sv2v_cast_A53F3(7);
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b00110) begin
							fpu_op = sv2v_cast_A53F3(7);
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b00111) begin
							if (FP_DIVSQRT) begin
								regb_used_o = 1'b0;
								fpu_op = sv2v_cast_A53F3(5);
								fp_op_group = 2'd1;
								apu_type_o = APUTYPE_SQRT;
								if ((instr_rdata_i[24:20] != 5'b00000) || instr_rdata_i[14])
									illegal_insn_o = 1'b1;
							end
							else
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[29:25] == 5'b01000) begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							reg_fp_c_o = 1'b1;
							fpu_op = sv2v_cast_A53F3(0);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_MAC;
						end
						else if (instr_rdata_i[29:25] == 5'b01001) begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							reg_fp_c_o = 1'b1;
							fpu_op = sv2v_cast_A53F3(0);
							fpu_op_mod = 1'b1;
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_MAC;
						end
						else if (instr_rdata_i[29:25] == 5'b01100) begin
							regb_used_o = 1'b0;
							scalar_replication_o = 1'b0;
							if (instr_rdata_i[24:20] == 5'b00000) begin
								alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
								fpu_op = sv2v_cast_A53F3(6);
								fp_rnd_mode_o = 3'b011;
								fp_op_group = 2'd2;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if (instr_rdata_i[14]) begin
									reg_fp_a_o = 1'b0;
									fpu_op_mod = 1'b0;
								end
								else begin
									reg_fp_d_o = 1'b0;
									fpu_op_mod = 1'b1;
								end
							end
							else if (instr_rdata_i[24:20] == 5'b00001) begin
								reg_fp_d_o = 1'b0;
								fpu_op = sv2v_cast_A53F3(9);
								fp_rnd_mode_o = 3'b000;
								fp_op_group = 2'd2;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if (instr_rdata_i[14])
									illegal_insn_o = 1'b1;
							end
							else if ((instr_rdata_i[24:20] | 5'b00001) == 5'b00011) begin
								fp_op_group = 2'd3;
								fpu_op_mod = instr_rdata_i[14];
								apu_type_o = APUTYPE_CAST;
								case (instr_rdata_i[13:12])
									2'b00: fpu_int_fmt_o = sv2v_cast_87CC5(2);
									2'b01, 2'b10: fpu_int_fmt_o = sv2v_cast_87CC5(1);
									2'b11: fpu_int_fmt_o = sv2v_cast_87CC5(0);
								endcase
								if (instr_rdata_i[20]) begin
									reg_fp_a_o = 1'b0;
									fpu_op = sv2v_cast_A53F3(12);
								end
								else begin
									reg_fp_d_o = 1'b0;
									fpu_op = sv2v_cast_A53F3(11);
								end
							end
							else if ((instr_rdata_i[24:20] | 5'b00011) == 5'b00111) begin
								fpu_op = sv2v_cast_A53F3(10);
								fp_op_group = 2'd3;
								apu_type_o = APUTYPE_CAST;
								case (instr_rdata_i[21:20])
									2'b00: begin
										fpu_src_fmt_o = sv2v_cast_0BC43('d0);
										if (~riscv_defines_C_RVF)
											illegal_insn_o = 1'b1;
									end
									2'b01: begin
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
										if (~riscv_defines_C_XF16ALT)
											illegal_insn_o = 1'b1;
									end
									2'b10: begin
										fpu_src_fmt_o = sv2v_cast_0BC43('d2);
										if (~riscv_defines_C_XF16)
											illegal_insn_o = 1'b1;
									end
									2'b11: begin
										fpu_src_fmt_o = sv2v_cast_0BC43('d3);
										if (~riscv_defines_C_XF8)
											illegal_insn_o = 1'b1;
									end
								endcase
								if (instr_rdata_i[14])
									illegal_insn_o = 1'b1;
							end
							else
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[29:25] == 5'b01101) begin
							fpu_op = sv2v_cast_A53F3(6);
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b01110) begin
							fpu_op = sv2v_cast_A53F3(6);
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b01111) begin
							fpu_op = sv2v_cast_A53F3(6);
							fp_rnd_mode_o = 3'b010;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10000) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(8);
							fp_rnd_mode_o = 3'b010;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10001) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(8);
							fpu_op_mod = 1'b1;
							fp_rnd_mode_o = 3'b010;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10010) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(8);
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10011) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(8);
							fpu_op_mod = 1'b1;
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10100) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(8);
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10101) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(8);
							fpu_op_mod = 1'b1;
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							apu_type_o = APUTYPE_FP;
							check_fprm = 1'b0;
						end
						else if ((instr_rdata_i[29:25] | 5'b00011) == 5'b11011) begin
							fpu_op_mod = instr_rdata_i[14];
							fp_op_group = 2'd3;
							apu_type_o = APUTYPE_CAST;
							scalar_replication_o = 1'b0;
							if (instr_rdata_i[25])
								fpu_op = sv2v_cast_A53F3(14);
							else
								fpu_op = sv2v_cast_A53F3(13);
							if (instr_rdata_i[26]) begin
								fpu_src_fmt_o = sv2v_cast_0BC43('d1);
								if (~riscv_defines_C_RVD)
									illegal_insn_o = 1'b1;
							end
							else begin
								fpu_src_fmt_o = sv2v_cast_0BC43('d0);
								if (~riscv_defines_C_RVF)
									illegal_insn_o = 1'b1;
							end
							if (fpu_op == sv2v_cast_A53F3(14)) begin
								if (~riscv_defines_C_XF8 || ~riscv_defines_C_RVD)
									illegal_insn_o = 1'b1;
							end
							else if (instr_rdata_i[14]) begin
								if (fpu_dst_fmt_o == sv2v_cast_0BC43('d0))
									illegal_insn_o = 1'b1;
								if (~riscv_defines_C_RVD && (fpu_dst_fmt_o != sv2v_cast_0BC43('d3)))
									illegal_insn_o = 1'b1;
							end
						end
						else
							illegal_insn_o = 1'b1;
						if ((~riscv_defines_C_RVF || ~riscv_defines_C_RVD) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d0)))
							illegal_insn_o = 1'b1;
						if ((~riscv_defines_C_XF16 || ~riscv_defines_C_RVF) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d2)))
							illegal_insn_o = 1'b1;
						if ((~riscv_defines_C_XF16ALT || ~riscv_defines_C_RVF) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d4)))
							illegal_insn_o = 1'b1;
						if ((~riscv_defines_C_XF8 || (~riscv_defines_C_XF16 && ~riscv_defines_C_XF16ALT)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d3)))
							illegal_insn_o = 1'b1;
						if (check_fprm) begin
							if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
								;
							else
								illegal_insn_o = 1'b1;
						end
						case (fp_op_group)
							2'd0:
								case (fpu_dst_fmt_o)
									sv2v_cast_0BC43('d0): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d2): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d4): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d3): apu_lat_o = 2'h3;
									default:
										;
								endcase
							2'd1: apu_lat_o = 2'h3;
							2'd2: apu_lat_o = 2'h3;
							2'd3: apu_lat_o = 2'h3;
						endcase
						apu_op_o = {fpu_vec_op, fpu_op_mod, fpu_op};
					end
					else
						illegal_insn_o = 1'b1;
				end
				else begin
					regfile_alu_we = 1'b1;
					rega_used_o = 1'b1;
					if (~instr_rdata_i[28])
						regb_used_o = 1'b1;
					case ({instr_rdata_i[30:25], instr_rdata_i[14:12]})
						9'b000000000: alu_operator_o = riscv_defines_ALU_ADD;
						9'b100000000: alu_operator_o = riscv_defines_ALU_SUB;
						9'b000000010: alu_operator_o = riscv_defines_ALU_SLTS;
						9'b000000011: alu_operator_o = riscv_defines_ALU_SLTU;
						9'b000000100: alu_operator_o = riscv_defines_ALU_XOR;
						9'b000000110: alu_operator_o = riscv_defines_ALU_OR;
						9'b000000111: alu_operator_o = riscv_defines_ALU_AND;
						9'b000000001: alu_operator_o = riscv_defines_ALU_SLL;
						9'b000000101: alu_operator_o = riscv_defines_ALU_SRL;
						9'b100000101: alu_operator_o = riscv_defines_ALU_SRA;
						9'b000001000: begin
							alu_en_o = 1'b0;
							mult_int_en = 1'b1;
							mult_operator_o = riscv_defines_MUL_MAC32;
							regc_mux_o = riscv_defines_REGC_ZERO;
						end
						9'b000001001: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_ZERO;
							mult_signed_mode_o = 2'b11;
							mult_int_en = 1'b1;
							mult_operator_o = riscv_defines_MUL_H;
							instr_multicycle_o = 1'b1;
						end
						9'b000001010: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_ZERO;
							mult_signed_mode_o = 2'b01;
							mult_int_en = 1'b1;
							mult_operator_o = riscv_defines_MUL_H;
							instr_multicycle_o = 1'b1;
						end
						9'b000001011: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_ZERO;
							mult_signed_mode_o = 2'b00;
							mult_int_en = 1'b1;
							mult_operator_o = riscv_defines_MUL_H;
							instr_multicycle_o = 1'b1;
						end
						9'b000001100: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_DIV;
							instr_multicycle_o = 1'b1;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b000001101: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_DIVU;
							instr_multicycle_o = 1'b1;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b000001110: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_REM;
							instr_multicycle_o = 1'b1;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b000001111: begin
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
							regc_mux_o = riscv_defines_REGC_S1;
							regc_used_o = 1'b1;
							regb_used_o = 1'b1;
							rega_used_o = 1'b0;
							alu_operator_o = riscv_defines_ALU_REMU;
							instr_multicycle_o = 1'b1;
							if (SHARED_INT_DIV) begin
								alu_en_o = 1'b0;
								apu_en = 1'b1;
								apu_type_o = APUTYPE_INT_DIV;
								apu_op_o = alu_operator_o;
								apu_lat_o = 2'h3;
							end
						end
						9'b100001000: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							mult_int_en = 1'b1;
							mult_operator_o = riscv_defines_MUL_MAC32;
							if (apu_core_package_SHARED_INT_MULT) begin
								mult_int_en = 1'b0;
								mult_dot_en = 1'b0;
								apu_en = 1'b1;
								apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
								apu_op_o = mult_operator_o;
								apu_type_o = APUTYPE_INT_MULT;
								apu_lat_o = 2'h1;
							end
						end
						9'b100001001: begin
							alu_en_o = 1'b0;
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							mult_int_en = 1'b1;
							mult_operator_o = riscv_defines_MUL_MSU32;
							if (apu_core_package_SHARED_INT_MULT) begin
								mult_int_en = 1'b0;
								mult_dot_en = 1'b0;
								apu_en = 1'b1;
								apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
								apu_op_o = mult_operator_o;
								apu_type_o = APUTYPE_INT_MULT;
								apu_lat_o = 2'h1;
							end
						end
						9'b000010010: alu_operator_o = riscv_defines_ALU_SLETS;
						9'b000010011: alu_operator_o = riscv_defines_ALU_SLETU;
						9'b000010100: alu_operator_o = riscv_defines_ALU_MIN;
						9'b000010101: alu_operator_o = riscv_defines_ALU_MINU;
						9'b000010110: alu_operator_o = riscv_defines_ALU_MAX;
						9'b000010111: alu_operator_o = riscv_defines_ALU_MAXU;
						9'b000100101: alu_operator_o = riscv_defines_ALU_ROR;
						9'b001000000: alu_operator_o = riscv_defines_ALU_FF1;
						9'b001000001: alu_operator_o = riscv_defines_ALU_FL1;
						9'b001000010: alu_operator_o = riscv_defines_ALU_CLB;
						9'b001000011: alu_operator_o = riscv_defines_ALU_CNT;
						9'b001000100: begin
							alu_operator_o = riscv_defines_ALU_EXTS;
							alu_vec_mode_o = riscv_defines_VEC_MODE16;
						end
						9'b001000101: begin
							alu_operator_o = riscv_defines_ALU_EXT;
							alu_vec_mode_o = riscv_defines_VEC_MODE16;
						end
						9'b001000110: begin
							alu_operator_o = riscv_defines_ALU_EXTS;
							alu_vec_mode_o = riscv_defines_VEC_MODE8;
						end
						9'b001000111: begin
							alu_operator_o = riscv_defines_ALU_EXT;
							alu_vec_mode_o = riscv_defines_VEC_MODE8;
						end
						9'b000010000: alu_operator_o = riscv_defines_ALU_ABS;
						9'b001010001: begin
							alu_operator_o = riscv_defines_ALU_CLIP;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
							imm_b_mux_sel_o = riscv_defines_IMMB_CLIP;
						end
						9'b001010010: begin
							alu_operator_o = riscv_defines_ALU_CLIPU;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
							imm_b_mux_sel_o = riscv_defines_IMMB_CLIP;
						end
						9'b001010101: begin
							alu_operator_o = riscv_defines_ALU_CLIP;
							regb_used_o = 1'b1;
						end
						9'b001010110: begin
							alu_operator_o = riscv_defines_ALU_CLIPU;
							regb_used_o = 1'b1;
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
			riscv_defines_OPCODE_OP_FP:
				if (FPU == 1) begin
					apu_en = 1'b1;
					alu_en_o = 1'b0;
					apu_flags_src_o = (SHARED_FP == 1 ? apu_core_package_APU_FLAGS_FP : apu_core_package_APU_FLAGS_FPNEW);
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					reg_fp_a_o = 1'b1;
					reg_fp_b_o = 1'b1;
					reg_fp_d_o = 1'b1;
					check_fprm = 1'b1;
					fp_rnd_mode_o = instr_rdata_i[14:12];
					case (instr_rdata_i[26:25])
						2'b00: fpu_dst_fmt_o = sv2v_cast_0BC43('d0);
						2'b01: fpu_dst_fmt_o = sv2v_cast_0BC43('d1);
						2'b10:
							if (instr_rdata_i[14:12] == 3'b101)
								fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
							else
								fpu_dst_fmt_o = sv2v_cast_0BC43('d2);
						2'b11: fpu_dst_fmt_o = sv2v_cast_0BC43('d3);
					endcase
					fpu_src_fmt_o = fpu_dst_fmt_o;
					case (instr_rdata_i[31:27])
						5'b00000: begin
							fpu_op = sv2v_cast_A53F3(2);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_ADDSUB;
							apu_op_o = 2'b00;
							apu_lat_o = 2'h2;
							if (SHARED_FP != 1) begin
								alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
								alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
							end
						end
						5'b00001: begin
							fpu_op = sv2v_cast_A53F3(2);
							fpu_op_mod = 1'b1;
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_ADDSUB;
							apu_op_o = 2'b01;
							apu_lat_o = 2'h2;
							if (SHARED_FP != 1) begin
								alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
								alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
							end
						end
						5'b00010: begin
							fpu_op = sv2v_cast_A53F3(3);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_MULT;
							apu_lat_o = 2'h2;
						end
						5'b00011:
							if (FP_DIVSQRT) begin
								fpu_op = sv2v_cast_A53F3(4);
								fp_op_group = 2'd1;
								apu_type_o = APUTYPE_DIV;
								apu_lat_o = 2'h3;
							end
							else
								illegal_insn_o = 1'b1;
						5'b01011:
							if (FP_DIVSQRT) begin
								regb_used_o = 1'b0;
								fpu_op = sv2v_cast_A53F3(5);
								fp_op_group = 2'd1;
								apu_type_o = APUTYPE_SQRT;
								apu_op_o = 1'b1;
								apu_lat_o = 2'h3;
								if (instr_rdata_i[24:20] != 5'b00000)
									illegal_insn_o = 1'b1;
							end
							else
								illegal_insn_o = 1'b1;
						5'b00100:
							if (SHARED_FP == 1) begin
								apu_en = 1'b0;
								alu_en_o = 1'b1;
								regfile_alu_we = 1'b1;
								case (instr_rdata_i[14:12])
									3'h0: alu_operator_o = riscv_defines_ALU_FSGNJ;
									3'h1: alu_operator_o = riscv_defines_ALU_FSGNJN;
									3'h2: alu_operator_o = riscv_defines_ALU_FSGNJX;
									default: illegal_insn_o = 1'b1;
								endcase
							end
							else begin
								fpu_op = sv2v_cast_A53F3(6);
								fp_op_group = 2'd2;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if (riscv_defines_C_XF16ALT) begin
									if (!(|{(3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12]), (3'b100 <= instr_rdata_i[14:12]) && (3'b110 >= instr_rdata_i[14:12])}))
										illegal_insn_o = 1'b1;
									if (instr_rdata_i[14]) begin
										fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
									else
										fp_rnd_mode_o = {1'b0, instr_rdata_i[13:12]};
								end
								else if (!((3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12])))
									illegal_insn_o = 1'b1;
							end
						5'b00101:
							if (SHARED_FP == 1) begin
								apu_en = 1'b0;
								alu_en_o = 1'b1;
								regfile_alu_we = 1'b1;
								case (instr_rdata_i[14:12])
									3'h0: alu_operator_o = riscv_defines_ALU_FMIN;
									3'h1: alu_operator_o = riscv_defines_ALU_FMAX;
									default: illegal_insn_o = 1'b1;
								endcase
							end
							else begin
								fpu_op = sv2v_cast_A53F3(7);
								fp_op_group = 2'd2;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if (riscv_defines_C_XF16ALT) begin
									if (!(|{(3'b000 <= instr_rdata_i[14:12]) && (3'b001 >= instr_rdata_i[14:12]), (3'b100 <= instr_rdata_i[14:12]) && (3'b101 >= instr_rdata_i[14:12])}))
										illegal_insn_o = 1'b1;
									if (instr_rdata_i[14]) begin
										fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
									else
										fp_rnd_mode_o = {1'b0, instr_rdata_i[13:12]};
								end
								else if (!((3'b000 <= instr_rdata_i[14:12]) && (3'b001 >= instr_rdata_i[14:12])))
									illegal_insn_o = 1'b1;
							end
						5'b01000:
							if (SHARED_FP == 1) begin
								apu_en = 1'b0;
								alu_en_o = 1'b1;
								regfile_alu_we = 1'b1;
								regb_used_o = 1'b0;
								alu_operator_o = riscv_defines_ALU_FKEEP;
							end
							else begin
								regb_used_o = 1'b0;
								fpu_op = sv2v_cast_A53F3(10);
								fp_op_group = 2'd3;
								apu_type_o = APUTYPE_CAST;
								if (instr_rdata_i[24:23])
									illegal_insn_o = 1'b1;
								case (instr_rdata_i[22:20])
									3'b000: begin
										if (~riscv_defines_C_RVF)
											illegal_insn_o = 1'b1;
										fpu_src_fmt_o = sv2v_cast_0BC43('d0);
									end
									3'b001: begin
										if (~riscv_defines_C_RVD)
											illegal_insn_o = 1'b1;
										fpu_src_fmt_o = sv2v_cast_0BC43('d1);
									end
									3'b010: begin
										if (~riscv_defines_C_XF16)
											illegal_insn_o = 1'b1;
										fpu_src_fmt_o = sv2v_cast_0BC43('d2);
									end
									3'b110: begin
										if (~riscv_defines_C_XF16ALT)
											illegal_insn_o = 1'b1;
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
									3'b011: begin
										if (~riscv_defines_C_XF8)
											illegal_insn_o = 1'b1;
										fpu_src_fmt_o = sv2v_cast_0BC43('d3);
									end
									default: illegal_insn_o = 1'b1;
								endcase
							end
						5'b01001: begin
							fpu_op = sv2v_cast_A53F3(3);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_MULT;
							apu_lat_o = 2'h2;
							fpu_dst_fmt_o = sv2v_cast_0BC43('d0);
						end
						5'b01010: begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							reg_fp_c_o = 1'b1;
							fpu_op = sv2v_cast_A53F3(0);
							fp_op_group = 2'd0;
							apu_type_o = APUTYPE_MAC;
							apu_lat_o = 2'h2;
							fpu_dst_fmt_o = sv2v_cast_0BC43('d0);
						end
						5'b10100:
							if (SHARED_FP == 1) begin
								apu_en = 1'b0;
								alu_en_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_d_o = 1'b0;
								case (instr_rdata_i[14:12])
									3'h0: alu_operator_o = riscv_defines_ALU_FLE;
									3'h1: alu_operator_o = riscv_defines_ALU_FLT;
									3'h2: alu_operator_o = riscv_defines_ALU_FEQ;
									default: illegal_insn_o = 1'b1;
								endcase
							end
							else begin
								fpu_op = sv2v_cast_A53F3(8);
								fp_op_group = 2'd2;
								reg_fp_d_o = 1'b0;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if (riscv_defines_C_XF16ALT) begin
									if (!(|{(3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12]), (3'b100 <= instr_rdata_i[14:12]) && (3'b110 >= instr_rdata_i[14:12])}))
										illegal_insn_o = 1'b1;
									if (instr_rdata_i[14]) begin
										fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
									else
										fp_rnd_mode_o = {1'b0, instr_rdata_i[13:12]};
								end
								else if (!((3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12])))
									illegal_insn_o = 1'b1;
							end
						5'b11000: begin
							regb_used_o = 1'b0;
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(11);
							fp_op_group = 2'd3;
							fpu_op_mod = instr_rdata_i[20];
							apu_type_o = APUTYPE_CAST;
							apu_op_o = 2'b01;
							apu_lat_o = 2'h2;
							case (instr_rdata_i[26:25])
								2'b00:
									if (~riscv_defines_C_RVF)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_0BC43('d0);
								2'b01:
									if (~riscv_defines_C_RVD)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_0BC43('d1);
								2'b10:
									if (instr_rdata_i[14:12] == 3'b101) begin
										if (~riscv_defines_C_XF16ALT)
											illegal_insn_o = 1;
										else
											fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
									else if (~riscv_defines_C_XF16)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_0BC43('d2);
								2'b11:
									if (~riscv_defines_C_XF8)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_0BC43('d3);
							endcase
							if (instr_rdata_i[24:21])
								illegal_insn_o = 1'b1;
						end
						5'b11010: begin
							regb_used_o = 1'b0;
							reg_fp_a_o = 1'b0;
							fpu_op = sv2v_cast_A53F3(12);
							fp_op_group = 2'd3;
							fpu_op_mod = instr_rdata_i[20];
							apu_type_o = APUTYPE_CAST;
							apu_op_o = 2'b00;
							apu_lat_o = 2'h2;
							if (instr_rdata_i[24:21])
								illegal_insn_o = 1'b1;
						end
						5'b11100:
							if (SHARED_FP == 1) begin
								apu_en = 1'b0;
								alu_en_o = 1'b1;
								regfile_alu_we = 1'b1;
								case (instr_rdata_i[14:12])
									3'b000: begin
										reg_fp_d_o = 1'b0;
										alu_operator_o = riscv_defines_ALU_ADD;
									end
									3'b001: begin
										regb_used_o = 1'b0;
										reg_fp_d_o = 1'b0;
										alu_operator_o = riscv_defines_ALU_FCLASS;
									end
									default: illegal_insn_o = 1'b1;
								endcase
							end
							else begin
								regb_used_o = 1'b0;
								reg_fp_d_o = 1'b0;
								fp_op_group = 2'd2;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if ((instr_rdata_i[14:12] == 3'b000) || 1'd0) begin
									alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
									fpu_op = sv2v_cast_A53F3(6);
									fpu_op_mod = 1'b1;
									fp_rnd_mode_o = 3'b011;
									if (instr_rdata_i[14]) begin
										fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
								end
								else if ((instr_rdata_i[14:12] == 3'b001) || 1'd0) begin
									fpu_op = sv2v_cast_A53F3(9);
									fp_rnd_mode_o = 3'b000;
									if (instr_rdata_i[14]) begin
										fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
								end
								else
									illegal_insn_o = 1'b1;
								if (instr_rdata_i[24:20])
									illegal_insn_o = 1'b1;
							end
						5'b11110:
							if (SHARED_FP == 1) begin
								apu_en = 1'b0;
								alu_en_o = 1'b1;
								regfile_alu_we = 1'b1;
								reg_fp_a_o = 1'b0;
								alu_operator_o = riscv_defines_ALU_ADD;
							end
							else begin
								regb_used_o = 1'b0;
								reg_fp_a_o = 1'b0;
								alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
								fpu_op = sv2v_cast_A53F3(6);
								fpu_op_mod = 1'b0;
								fp_op_group = 2'd2;
								fp_rnd_mode_o = 3'b011;
								apu_type_o = APUTYPE_FP;
								check_fprm = 1'b0;
								if ((instr_rdata_i[14:12] == 3'b000) || 1'd0) begin
									if (instr_rdata_i[14]) begin
										fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
										fpu_src_fmt_o = sv2v_cast_0BC43('d4);
									end
								end
								else
									illegal_insn_o = 1'b1;
								if (instr_rdata_i[24:20] != 5'b00000)
									illegal_insn_o = 1'b1;
							end
						default: illegal_insn_o = 1'b1;
					endcase
					if (~riscv_defines_C_RVF && (fpu_dst_fmt_o == sv2v_cast_0BC43('d0)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_RVD || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d1)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_XF16 || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d2)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_XF16ALT || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d4)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_XF8 || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d3)))
						illegal_insn_o = 1'b1;
					if (check_fprm) begin
						if ((3'b000 <= instr_rdata_i[14:12]) && (3'b100 >= instr_rdata_i[14:12]))
							;
						else if (instr_rdata_i[14:12] == 3'b101) begin
							if (~riscv_defines_C_XF16ALT || (fpu_dst_fmt_o != sv2v_cast_0BC43('d4)))
								illegal_insn_o = 1'b1;
							if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
								fp_rnd_mode_o = frm_i;
							else
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[14:12] == 3'b111) begin
							if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
								fp_rnd_mode_o = frm_i;
							else
								illegal_insn_o = 1'b1;
						end
						else
							illegal_insn_o = 1'b1;
					end
					if (SHARED_FP != 1)
						case (fp_op_group)
							2'd0:
								case (fpu_dst_fmt_o)
									sv2v_cast_0BC43('d0): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d1): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d2): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d4): apu_lat_o = 2'h3;
									sv2v_cast_0BC43('d3): apu_lat_o = 2'h3;
									default:
										;
								endcase
							2'd1: apu_lat_o = 2'h3;
							2'd2: apu_lat_o = 2'h3;
							2'd3: apu_lat_o = 2'h3;
						endcase
					if (SHARED_FP != 1)
						apu_op_o = {fpu_vec_op, fpu_op_mod, fpu_op};
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_OP_FMADD, riscv_defines_OPCODE_OP_FMSUB, riscv_defines_OPCODE_OP_FNMSUB, riscv_defines_OPCODE_OP_FNMADD:
				if (FPU == 1) begin
					apu_en = 1'b1;
					alu_en_o = 1'b0;
					apu_flags_src_o = (SHARED_FP == 1 ? apu_core_package_APU_FLAGS_FP : apu_core_package_APU_FLAGS_FPNEW);
					apu_type_o = APUTYPE_MAC;
					apu_lat_o = 2'h3;
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					regc_used_o = 1'b1;
					regc_mux_o = riscv_defines_REGC_S4;
					reg_fp_a_o = 1'b1;
					reg_fp_b_o = 1'b1;
					reg_fp_c_o = 1'b1;
					reg_fp_d_o = 1'b1;
					fp_rnd_mode_o = instr_rdata_i[14:12];
					case (instr_rdata_i[26:25])
						2'b00: fpu_dst_fmt_o = sv2v_cast_0BC43('d0);
						2'b01: fpu_dst_fmt_o = sv2v_cast_0BC43('d1);
						2'b10:
							if (instr_rdata_i[14:12] == 3'b101)
								fpu_dst_fmt_o = sv2v_cast_0BC43('d4);
							else
								fpu_dst_fmt_o = sv2v_cast_0BC43('d2);
						2'b11: fpu_dst_fmt_o = sv2v_cast_0BC43('d3);
					endcase
					fpu_src_fmt_o = fpu_dst_fmt_o;
					case (instr_rdata_i[6:0])
						riscv_defines_OPCODE_OP_FMADD: begin
							fpu_op = sv2v_cast_A53F3(0);
							apu_op_o = 2'b00;
						end
						riscv_defines_OPCODE_OP_FMSUB: begin
							fpu_op = sv2v_cast_A53F3(0);
							fpu_op_mod = 1'b1;
							apu_op_o = 2'b01;
						end
						riscv_defines_OPCODE_OP_FNMSUB: begin
							fpu_op = sv2v_cast_A53F3(1);
							apu_op_o = 2'b10;
						end
						riscv_defines_OPCODE_OP_FNMADD: begin
							fpu_op = sv2v_cast_A53F3(1);
							fpu_op_mod = 1'b1;
							apu_op_o = 2'b11;
						end
					endcase
					if (~riscv_defines_C_RVF && (fpu_dst_fmt_o == sv2v_cast_0BC43('d0)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_RVD || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d1)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_XF16 || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d2)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_XF16ALT || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d4)))
						illegal_insn_o = 1'b1;
					if ((~riscv_defines_C_XF8 || (SHARED_FP == 1)) && (fpu_dst_fmt_o == sv2v_cast_0BC43('d3)))
						illegal_insn_o = 1'b1;
					if ((3'b000 <= instr_rdata_i[14:12]) && (3'b100 >= instr_rdata_i[14:12]))
						;
					else if (instr_rdata_i[14:12] == 3'b101) begin
						if (~riscv_defines_C_XF16ALT || (fpu_dst_fmt_o != sv2v_cast_0BC43('d4)))
							illegal_insn_o = 1'b1;
						if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
							fp_rnd_mode_o = frm_i;
						else
							illegal_insn_o = 1'b1;
					end
					else if (instr_rdata_i[14:12] == 3'b111) begin
						if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
							fp_rnd_mode_o = frm_i;
						else
							illegal_insn_o = 1'b1;
					end
					else
						illegal_insn_o = 1'b1;
					if (SHARED_FP != 1)
						case (fpu_dst_fmt_o)
							sv2v_cast_0BC43('d0): apu_lat_o = 2'h3;
							sv2v_cast_0BC43('d1): apu_lat_o = 2'h3;
							sv2v_cast_0BC43('d2): apu_lat_o = 2'h3;
							sv2v_cast_0BC43('d4): apu_lat_o = 2'h3;
							sv2v_cast_0BC43('d3): apu_lat_o = 2'h3;
							default:
								;
						endcase
					if (SHARED_FP != 1)
						apu_op_o = {fpu_vec_op, fpu_op_mod, fpu_op};
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_STORE_FP:
				if (FPU == 1) begin
					data_req = 1'b1;
					data_we_o = 1'b1;
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					alu_operator_o = riscv_defines_ALU_ADD;
					reg_fp_b_o = 1'b1;
					instr_multicycle_o = 1'b1;
					imm_b_mux_sel_o = riscv_defines_IMMB_S;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					alu_op_c_mux_sel_o = riscv_defines_OP_C_REGB_OR_FWD;
					case (instr_rdata_i[14:12])
						3'b000:
							if (riscv_defines_C_XF8)
								data_type_o = 2'b10;
							else
								illegal_insn_o = 1'b1;
						3'b001:
							if (riscv_defines_C_XF16 | riscv_defines_C_XF16ALT)
								data_type_o = 2'b01;
							else
								illegal_insn_o = 1'b1;
						3'b010:
							if (riscv_defines_C_RVF)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						3'b011:
							if (riscv_defines_C_RVD)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						default: illegal_insn_o = 1'b1;
					endcase
					if (illegal_insn_o) begin
						data_req = 1'b0;
						data_we_o = 1'b0;
					end
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_LOAD_FP:
				if (FPU == 1) begin
					data_req = 1'b1;
					regfile_mem_we = 1'b1;
					reg_fp_d_o = 1'b1;
					rega_used_o = 1'b1;
					alu_operator_o = riscv_defines_ALU_ADD;
					instr_multicycle_o = 1'b1;
					imm_b_mux_sel_o = riscv_defines_IMMB_I;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					data_sign_extension_o = 2'b10;
					case (instr_rdata_i[14:12])
						3'b000:
							if (riscv_defines_C_XF8)
								data_type_o = 2'b10;
							else
								illegal_insn_o = 1'b1;
						3'b001:
							if (riscv_defines_C_XF16 | riscv_defines_C_XF16ALT)
								data_type_o = 2'b01;
							else
								illegal_insn_o = 1'b1;
						3'b010:
							if (riscv_defines_C_RVF)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						3'b011:
							if (riscv_defines_C_RVD)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						default: illegal_insn_o = 1'b1;
					endcase
				end
				else
					illegal_insn_o = 1'b1;
			riscv_defines_OPCODE_PULP_OP: begin
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				case (instr_rdata_i[13:12])
					2'b00: begin
						alu_en_o = 1'b0;
						mult_sel_subword_o = instr_rdata_i[30];
						mult_signed_mode_o = {2 {instr_rdata_i[31]}};
						mult_imm_mux_o = riscv_defines_MIMM_S3;
						regc_mux_o = riscv_defines_REGC_ZERO;
						mult_int_en = 1'b1;
						if (instr_rdata_i[14])
							mult_operator_o = riscv_defines_MUL_IR;
						else
							mult_operator_o = riscv_defines_MUL_I;
						if (apu_core_package_SHARED_INT_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
							apu_op_o = mult_operator_o;
							apu_type_o = APUTYPE_INT_MULT;
							apu_lat_o = 2'h1;
						end
					end
					2'b01: begin
						alu_en_o = 1'b0;
						mult_sel_subword_o = instr_rdata_i[30];
						mult_signed_mode_o = {2 {instr_rdata_i[31]}};
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						mult_imm_mux_o = riscv_defines_MIMM_S3;
						mult_int_en = 1'b1;
						if (instr_rdata_i[14])
							mult_operator_o = riscv_defines_MUL_IR;
						else
							mult_operator_o = riscv_defines_MUL_I;
						if (apu_core_package_SHARED_INT_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_flags_src_o = apu_core_package_APU_FLAGS_INT_MULT;
							apu_op_o = mult_operator_o;
							apu_type_o = APUTYPE_INT_MULT;
							apu_lat_o = 2'h1;
						end
					end
					2'b10: begin
						case ({instr_rdata_i[31], instr_rdata_i[14]})
							2'b00: alu_operator_o = riscv_defines_ALU_ADD;
							2'b01: alu_operator_o = riscv_defines_ALU_ADDR;
							2'b10: alu_operator_o = riscv_defines_ALU_ADDU;
							2'b11: alu_operator_o = riscv_defines_ALU_ADDUR;
						endcase
						bmask_a_mux_o = riscv_defines_BMASK_A_ZERO;
						bmask_b_mux_o = riscv_defines_BMASK_B_S3;
						if (instr_rdata_i[30]) begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGC_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
						end
					end
					2'b11: begin
						case ({instr_rdata_i[31], instr_rdata_i[14]})
							2'b00: alu_operator_o = riscv_defines_ALU_SUB;
							2'b01: alu_operator_o = riscv_defines_ALU_SUBR;
							2'b10: alu_operator_o = riscv_defines_ALU_SUBU;
							2'b11: alu_operator_o = riscv_defines_ALU_SUBUR;
						endcase
						bmask_a_mux_o = riscv_defines_BMASK_A_ZERO;
						bmask_b_mux_o = riscv_defines_BMASK_B_S3;
						if (instr_rdata_i[30]) begin
							regc_used_o = 1'b1;
							regc_mux_o = riscv_defines_REGC_RD;
							alu_bmask_b_mux_sel_o = riscv_defines_BMASK_B_REG;
							alu_op_a_mux_sel_o = riscv_defines_OP_A_REGC_OR_FWD;
							alu_op_b_mux_sel_o = riscv_defines_OP_B_REGA_OR_FWD;
						end
					end
				endcase
			end
			riscv_defines_OPCODE_VECOP: begin
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				imm_b_mux_sel_o = riscv_defines_IMMB_VS;
				if (instr_rdata_i[12]) begin
					alu_vec_mode_o = riscv_defines_VEC_MODE8;
					mult_operator_o = riscv_defines_MUL_DOT8;
				end
				else begin
					alu_vec_mode_o = riscv_defines_VEC_MODE16;
					mult_operator_o = riscv_defines_MUL_DOT16;
				end
				if (instr_rdata_i[14]) begin
					scalar_replication_o = 1'b1;
					if (instr_rdata_i[13])
						alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					else
						regb_used_o = 1'b1;
				end
				else
					regb_used_o = 1'b1;
				case (instr_rdata_i[31:26])
					6'b000000: begin
						alu_operator_o = riscv_defines_ALU_ADD;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000010: begin
						alu_operator_o = riscv_defines_ALU_SUB;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000100: begin
						alu_operator_o = riscv_defines_ALU_ADD;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
						bmask_b_mux_o = riscv_defines_BMASK_B_ONE;
					end
					6'b000110: begin
						alu_operator_o = riscv_defines_ALU_ADDU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
						bmask_b_mux_o = riscv_defines_BMASK_B_ONE;
					end
					6'b001000: begin
						alu_operator_o = riscv_defines_ALU_MIN;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001010: begin
						alu_operator_o = riscv_defines_ALU_MINU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b001100: begin
						alu_operator_o = riscv_defines_ALU_MAX;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001110: begin
						alu_operator_o = riscv_defines_ALU_MAXU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b010000: begin
						alu_operator_o = riscv_defines_ALU_SRL;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b010010: begin
						alu_operator_o = riscv_defines_ALU_SRA;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b010100: begin
						alu_operator_o = riscv_defines_ALU_SLL;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b010110: begin
						alu_operator_o = riscv_defines_ALU_OR;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b011000: begin
						alu_operator_o = riscv_defines_ALU_XOR;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b011010: begin
						alu_operator_o = riscv_defines_ALU_AND;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b011100: begin
						alu_operator_o = riscv_defines_ALU_ABS;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b111010, 6'b111100, 6'b111110, 6'b110000: begin
						alu_operator_o = riscv_defines_ALU_SHUF;
						imm_b_mux_sel_o = riscv_defines_IMMB_SHUF;
						regb_used_o = 1'b1;
						scalar_replication_o = 1'b0;
					end
					6'b110010: begin
						alu_operator_o = riscv_defines_ALU_SHUF2;
						regb_used_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						scalar_replication_o = 1'b0;
					end
					6'b110100: begin
						alu_operator_o = (instr_rdata_i[25] ? riscv_defines_ALU_PCKHI : riscv_defines_ALU_PCKLO);
						regb_used_o = 1'b1;
					end
					6'b110110: begin
						alu_operator_o = riscv_defines_ALU_PCKHI;
						regb_used_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
					end
					6'b111000: begin
						alu_operator_o = riscv_defines_ALU_PCKLO;
						regb_used_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
					end
					6'b011110: alu_operator_o = riscv_defines_ALU_EXTS;
					6'b100100: alu_operator_o = riscv_defines_ALU_EXT;
					6'b101100: begin
						alu_operator_o = riscv_defines_ALU_INS;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_REGC_OR_FWD;
					end
					6'b100000: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b00;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b100010: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b01;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b100110: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b11;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b101000: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b00;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b101010: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b01;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b101110: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b11;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b010101: begin
						alu_en_o = 1'b0;
						mult_dot_en = 1'b1;
						mult_dot_signed_o = 2'b11;
						is_clpx_o = 1'b1;
						regc_used_o = 1'b1;
						regc_mux_o = riscv_defines_REGC_RD;
						scalar_replication_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
						regb_used_o = 1'b1;
						if (SHARED_DSP_MULT) begin
							mult_int_en = 1'b0;
							mult_dot_en = 1'b0;
							apu_en = 1'b1;
							apu_type_o = APUTYPE_DSP_MULT;
							apu_flags_src_o = apu_core_package_APU_FLAGS_DSP_MULT;
							apu_op_o = mult_operator_o;
							apu_lat_o = 2'h2;
						end
					end
					6'b011011: begin
						alu_operator_o = riscv_defines_ALU_SUB;
						is_clpx_o = 1'b1;
						scalar_replication_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
						regb_used_o = 1'b1;
						is_subrot_o = 1'b1;
					end
					6'b010111: begin
						alu_operator_o = riscv_defines_ALU_ABS;
						is_clpx_o = 1'b1;
						scalar_replication_o = 1'b0;
						regb_used_o = 1'b0;
					end
					6'b011101: begin
						alu_operator_o = riscv_defines_ALU_ADD;
						is_clpx_o = 1'b1;
						scalar_replication_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
						regb_used_o = 1'b1;
					end
					6'b011001: begin
						alu_operator_o = riscv_defines_ALU_SUB;
						is_clpx_o = 1'b1;
						scalar_replication_o = 1'b0;
						alu_op_b_mux_sel_o = riscv_defines_OP_B_REGB_OR_FWD;
						regb_used_o = 1'b1;
					end
					6'b000001: begin
						alu_operator_o = riscv_defines_ALU_EQ;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000011: begin
						alu_operator_o = riscv_defines_ALU_NE;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000101: begin
						alu_operator_o = riscv_defines_ALU_GTS;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b000111: begin
						alu_operator_o = riscv_defines_ALU_GES;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001001: begin
						alu_operator_o = riscv_defines_ALU_LTS;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001011: begin
						alu_operator_o = riscv_defines_ALU_LES;
						imm_b_mux_sel_o = riscv_defines_IMMB_VS;
					end
					6'b001101: begin
						alu_operator_o = riscv_defines_ALU_GTU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b001111: begin
						alu_operator_o = riscv_defines_ALU_GEU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b010001: begin
						alu_operator_o = riscv_defines_ALU_LTU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					6'b010011: begin
						alu_operator_o = riscv_defines_ALU_LEU;
						imm_b_mux_sel_o = riscv_defines_IMMB_VU;
					end
					default: illegal_insn_o = 1'b1;
				endcase
			end
			riscv_defines_OPCODE_FENCE:
				case (instr_rdata_i[14:12])
					3'b000: fencei_insn_o = 1'b1;
					3'b001: fencei_insn_o = 1'b1;
					default: illegal_insn_o = 1'b1;
				endcase
			riscv_defines_OPCODE_SYSTEM:
				if (instr_rdata_i[14:12] == 3'b000) begin
					if ({instr_rdata_i[19:15], instr_rdata_i[11:7]} == {10 {1'sb0}})
						case (instr_rdata_i[31:20])
							12'h000: ecall_insn_o = 1'b1;
							12'h001: ebrk_insn_o = 1'b1;
							12'h302: begin
								illegal_insn_o = (PULP_SECURE ? current_priv_lvl_i != 2'b11 : 1'b0);
								mret_insn_o = ~illegal_insn_o;
								mret_dec_o = 1'b1;
							end
							12'h002: begin
								uret_insn_o = (PULP_SECURE ? 1'b1 : 1'b0);
								uret_dec_o = 1'b1;
							end
							12'h7b2: begin
								illegal_insn_o = (PULP_SECURE ? current_priv_lvl_i != 2'b11 : 1'b0);
								dret_insn_o = ~illegal_insn_o;
								dret_dec_o = 1'b1;
							end
							12'h105: pipe_flush_o = 1'b1;
							default: illegal_insn_o = 1'b1;
						endcase
					else
						illegal_insn_o = 1'b1;
				end
				else begin
					csr_access_o = 1'b1;
					regfile_alu_we = 1'b1;
					alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
					imm_a_mux_sel_o = riscv_defines_IMMA_Z;
					imm_b_mux_sel_o = riscv_defines_IMMB_I;
					instr_multicycle_o = 1'b1;
					if (instr_rdata_i[14] == 1'b1)
						alu_op_a_mux_sel_o = riscv_defines_OP_A_IMM;
					else begin
						rega_used_o = 1'b1;
						alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
					end
					case (instr_rdata_i[13:12])
						2'b01: csr_op = riscv_defines_CSR_OP_WRITE;
						2'b10: csr_op = riscv_defines_CSR_OP_SET;
						2'b11: csr_op = riscv_defines_CSR_OP_CLEAR;
						default: csr_illegal = 1'b1;
					endcase
					if (instr_rdata_i[29:28] > current_priv_lvl_i)
						csr_illegal = 1'b1;
					if (~csr_illegal) begin
						if (((((((instr_rdata_i[31:20] == 12'h300) || (instr_rdata_i[31:20] == 12'h000)) || (instr_rdata_i[31:20] == 12'h041)) || (instr_rdata_i[31:20] == 12'h7b0)) || (instr_rdata_i[31:20] == 12'h7b1)) || (instr_rdata_i[31:20] == 12'h7b2)) || (instr_rdata_i[31:20] == 12'h7b3))
							csr_status_o = 1'b1;
					end
					illegal_insn_o = csr_illegal;
				end
			riscv_defines_OPCODE_HWLOOP: begin
				hwloop_target_mux_sel_o = 1'b0;
				case (instr_rdata_i[14:12])
					3'b000: begin
						hwloop_we[0] = 1'b1;
						hwloop_start_mux_sel_o = 1'b0;
					end
					3'b001: hwloop_we[1] = 1'b1;
					3'b010: begin
						hwloop_we[2] = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b1;
						rega_used_o = 1'b1;
					end
					3'b011: begin
						hwloop_we[2] = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b0;
					end
					3'b100: begin
						hwloop_we = 3'b111;
						hwloop_start_mux_sel_o = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b1;
						rega_used_o = 1'b1;
					end
					3'b101: begin
						hwloop_we = 3'b111;
						hwloop_target_mux_sel_o = 1'b1;
						hwloop_start_mux_sel_o = 1'b1;
						hwloop_cnt_mux_sel_o = 1'b0;
					end
					default: illegal_insn_o = 1'b1;
				endcase
			end
			default: illegal_insn_o = 1'b1;
		endcase
		if (illegal_c_insn_i)
			illegal_insn_o = 1'b1;
		if (data_misaligned_i == 1'b1) begin
			alu_op_a_mux_sel_o = riscv_defines_OP_A_REGA_OR_FWD;
			alu_op_b_mux_sel_o = riscv_defines_OP_B_IMM;
			imm_b_mux_sel_o = riscv_defines_IMMB_PCINCR;
			regfile_alu_we = 1'b0;
			prepost_useincr_o = 1'b1;
			scalar_replication_o = 1'b0;
		end
		else if (mult_multicycle_i)
			alu_op_c_mux_sel_o = riscv_defines_OP_C_REGC_OR_FWD;
	end
	assign apu_en_o = (deassert_we_i ? 1'b0 : apu_en);
	assign mult_int_en_o = (deassert_we_i ? 1'b0 : mult_int_en);
	assign mult_dot_en_o = (deassert_we_i ? 1'b0 : mult_dot_en);
	assign regfile_mem_we_o = (deassert_we_i ? 1'b0 : regfile_mem_we);
	assign regfile_alu_we_o = (deassert_we_i ? 1'b0 : regfile_alu_we);
	assign data_req_o = (deassert_we_i ? 1'b0 : data_req);
	assign hwloop_we_o = (deassert_we_i ? 3'b000 : hwloop_we);
	assign csr_op_o = (deassert_we_i ? riscv_defines_CSR_OP_NONE : csr_op);
	assign jump_in_id_o = (deassert_we_i ? riscv_defines_BRANCH_NONE : jump_in_id);
	assign jump_in_dec_o = jump_in_id;
	assign regfile_alu_we_dec_o = regfile_alu_we;
	initial _sv2v_0 = 0;
endmodule
module riscv_compressed_decoder (
	instr_i,
	instr_o,
	is_compressed_o,
	illegal_instr_o
);
	reg _sv2v_0;
	parameter FPU = 0;
	input wire [31:0] instr_i;
	output reg [31:0] instr_o;
	output wire is_compressed_o;
	output reg illegal_instr_o;
	localparam riscv_defines_OPCODE_BRANCH = 7'h63;
	localparam riscv_defines_OPCODE_JAL = 7'h6f;
	localparam riscv_defines_OPCODE_JALR = 7'h67;
	localparam riscv_defines_OPCODE_LOAD = 7'h03;
	localparam riscv_defines_OPCODE_LOAD_FP = 7'h07;
	localparam riscv_defines_OPCODE_LUI = 7'h37;
	localparam riscv_defines_OPCODE_OP = 7'h33;
	localparam riscv_defines_OPCODE_OPIMM = 7'h13;
	localparam riscv_defines_OPCODE_STORE = 7'h23;
	localparam riscv_defines_OPCODE_STORE_FP = 7'h27;
	always @(*) begin
		if (_sv2v_0)
			;
		illegal_instr_o = 1'b0;
		instr_o = 1'sb0;
		case (instr_i[1:0])
			2'b00:
				case (instr_i[15:13])
					3'b000: begin
						instr_o = {2'b00, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 12'h041, instr_i[4:2], riscv_defines_OPCODE_OPIMM};
						if (instr_i[12:5] == 8'b00000000)
							illegal_instr_o = 1'b1;
					end
					3'b001:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[6:5], instr_i[12:10], 5'b00001, instr_i[9:7], 5'b01101, instr_i[4:2], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b010: instr_o = {5'b00000, instr_i[5], instr_i[12:10], instr_i[6], 4'b0001, instr_i[9:7], 5'b01001, instr_i[4:2], riscv_defines_OPCODE_LOAD};
					3'b011:
						if (FPU == 1)
							instr_o = {5'b00000, instr_i[5], instr_i[12:10], instr_i[6], 4'b0001, instr_i[9:7], 5'b01001, instr_i[4:2], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b101:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[6:5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b011, instr_i[11:10], 3'b000, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					3'b110: instr_o = {5'b00000, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, riscv_defines_OPCODE_STORE};
					3'b111:
						if (FPU == 1)
							instr_o = {5'b00000, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					default: illegal_instr_o = 1'b1;
				endcase
			2'b01:
				case (instr_i[15:13])
					3'b000: instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], riscv_defines_OPCODE_OPIMM};
					3'b001, 3'b101: instr_o = {instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], {9 {instr_i[12]}}, 4'b0000, ~instr_i[15], riscv_defines_OPCODE_JAL};
					3'b010: begin
						instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 8'b00000000, instr_i[11:7], riscv_defines_OPCODE_OPIMM};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
					end
					3'b011: begin
						instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], riscv_defines_OPCODE_LUI};
						if (instr_i[11:7] == 5'h02)
							instr_o = {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6], 17'h00202, riscv_defines_OPCODE_OPIMM};
						else if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
						if ({instr_i[12], instr_i[6:2]} == 6'b000000)
							illegal_instr_o = 1'b1;
					end
					3'b100:
						case (instr_i[11:10])
							2'b00, 2'b01: begin
								instr_o = {1'b0, instr_i[10], 5'b00000, instr_i[6:2], 2'b01, instr_i[9:7], 5'b10101, instr_i[9:7], riscv_defines_OPCODE_OPIMM};
								if (instr_i[12] == 1'b1)
									illegal_instr_o = 1'b1;
								if (instr_i[6:2] == 5'b00000)
									illegal_instr_o = 1'b1;
							end
							2'b10: instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 2'b01, instr_i[9:7], 5'b11101, instr_i[9:7], riscv_defines_OPCODE_OPIMM};
							2'b11:
								case ({instr_i[12], instr_i[6:5]})
									3'b000: instr_o = {9'b010000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b00001, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b001: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b10001, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b010: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b11001, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b011: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b11101, instr_i[9:7], riscv_defines_OPCODE_OP};
									3'b100, 3'b101, 3'b110, 3'b111: illegal_instr_o = 1'b1;
								endcase
						endcase
					3'b110, 3'b111: instr_o = {{4 {instr_i[12]}}, instr_i[6:5], instr_i[2], 7'b0000001, instr_i[9:7], 2'b00, instr_i[13], instr_i[11:10], instr_i[4:3], instr_i[12], riscv_defines_OPCODE_BRANCH};
				endcase
			2'b10:
				case (instr_i[15:13])
					3'b000: begin
						instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], riscv_defines_OPCODE_OPIMM};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
						if ((instr_i[12] == 1'b1) || (instr_i[6:2] == 5'b00000))
							illegal_instr_o = 1'b1;
					end
					3'b001:
						if (FPU == 1)
							instr_o = {3'b000, instr_i[4:2], instr_i[12], instr_i[6:5], 11'h013, instr_i[11:7], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b010: begin
						instr_o = {4'b0000, instr_i[3:2], instr_i[12], instr_i[6:4], 10'h012, instr_i[11:7], riscv_defines_OPCODE_LOAD};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
					end
					3'b011:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[3:2], instr_i[12], instr_i[6:4], 10'h012, instr_i[11:7], riscv_defines_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b100:
						if (instr_i[12] == 1'b0) begin
							instr_o = {7'b0000000, instr_i[6:2], 8'b00000000, instr_i[11:7], riscv_defines_OPCODE_OP};
							if (instr_i[6:2] == 5'b00000)
								instr_o = {12'b000000000000, instr_i[11:7], 8'b00000000, riscv_defines_OPCODE_JALR};
						end
						else begin
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], riscv_defines_OPCODE_OP};
							if (instr_i[11:7] == 5'b00000) begin
								if (instr_i[6:2] != 5'b00000)
									illegal_instr_o = 1'b1;
								else
									instr_o = 32'h00100073;
							end
							else if (instr_i[6:2] == 5'b00000)
								instr_o = {12'b000000000000, instr_i[11:7], 8'b00000001, riscv_defines_OPCODE_JALR};
						end
					3'b101:
						if (FPU == 1)
							instr_o = {3'b000, instr_i[9:7], instr_i[12], instr_i[6:2], 8'h13, instr_i[11:10], 3'b000, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					3'b110: instr_o = {4'b0000, instr_i[8:7], instr_i[12], instr_i[6:2], 8'h12, instr_i[11:9], 2'b00, riscv_defines_OPCODE_STORE};
					3'b111:
						if (FPU == 1)
							instr_o = {4'b0000, instr_i[8:7], instr_i[12], instr_i[6:2], 8'h12, instr_i[11:9], 2'b00, riscv_defines_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
				endcase
			default: instr_o = instr_i;
		endcase
	end
	assign is_compressed_o = instr_i[1:0] != 2'b11;
	initial _sv2v_0 = 0;
endmodule
module riscv_fetch_fifo (
	clk,
	rst_n,
	clear_i,
	in_addr_i,
	in_rdata_i,
	in_valid_i,
	in_ready_o,
	in_replace2_i,
	in_is_hwlp_i,
	out_valid_o,
	out_ready_i,
	out_rdata_o,
	out_addr_o,
	unaligned_is_compressed_o,
	out_valid_stored_o,
	out_is_hwlp_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire clear_i;
	input wire [31:0] in_addr_i;
	input wire [31:0] in_rdata_i;
	input wire in_valid_i;
	output wire in_ready_o;
	input wire in_replace2_i;
	input wire in_is_hwlp_i;
	output reg out_valid_o;
	input wire out_ready_i;
	output reg [31:0] out_rdata_o;
	output wire [31:0] out_addr_o;
	output wire unaligned_is_compressed_o;
	output reg out_valid_stored_o;
	output wire out_is_hwlp_o;
	localparam DEPTH = 4;
	reg [127:0] addr_n;
	reg [127:0] addr_int;
	reg [127:0] addr_Q;
	reg [127:0] rdata_n;
	reg [127:0] rdata_int;
	reg [127:0] rdata_Q;
	reg [0:3] valid_n;
	reg [0:3] valid_int;
	reg [0:3] valid_Q;
	reg [0:1] is_hwlp_n;
	reg [0:1] is_hwlp_int;
	reg [0:1] is_hwlp_Q;
	wire [31:0] addr_next;
	wire [31:0] rdata;
	wire [31:0] rdata_unaligned;
	wire valid;
	wire valid_unaligned;
	wire aligned_is_compressed;
	wire unaligned_is_compressed;
	wire aligned_is_compressed_st;
	wire unaligned_is_compressed_st;
	assign rdata = (valid_Q[0] ? rdata_Q[96+:32] : in_rdata_i & {32 {in_valid_i}});
	assign valid = (valid_Q[0] || in_valid_i) || is_hwlp_Q[1];
	assign rdata_unaligned = (valid_Q[1] ? {rdata_Q[79-:16], rdata[31:16]} : {in_rdata_i[15:0], rdata[31:16]});
	assign valid_unaligned = valid_Q[1] || (valid_Q[0] && in_valid_i);
	assign unaligned_is_compressed_o = unaligned_is_compressed;
	assign unaligned_is_compressed = rdata[17:16] != 2'b11;
	assign aligned_is_compressed = rdata[1:0] != 2'b11;
	assign unaligned_is_compressed_st = valid_Q[0] && (rdata_Q[113-:2] != 2'b11);
	assign aligned_is_compressed_st = valid_Q[0] && (rdata_Q[97-:2] != 2'b11);
	always @(*) begin
		if (_sv2v_0)
			;
		if (out_addr_o[1] && ~is_hwlp_Q[1]) begin
			out_rdata_o = rdata_unaligned;
			if (unaligned_is_compressed)
				out_valid_o = valid;
			else
				out_valid_o = valid_unaligned;
		end
		else begin
			out_rdata_o = rdata;
			out_valid_o = valid;
		end
	end
	assign out_addr_o = (valid_Q[0] ? addr_Q[96+:32] : in_addr_i);
	assign out_is_hwlp_o = (valid_Q[0] ? is_hwlp_Q[0] : in_is_hwlp_i);
	always @(*) begin
		if (_sv2v_0)
			;
		out_valid_stored_o = 1'b1;
		if (out_addr_o[1] && ~is_hwlp_Q[1]) begin
			if (unaligned_is_compressed_st)
				out_valid_stored_o = 1'b1;
			else
				out_valid_stored_o = valid_Q[1];
		end
		else
			out_valid_stored_o = valid_Q[0];
	end
	assign in_ready_o = ~valid_Q[2];
	always @(*) begin : sv2v_autoblock_1
		reg [0:1] _sv2v_jump;
		_sv2v_jump = 2'b00;
		if (_sv2v_0)
			;
		addr_int = addr_Q;
		rdata_int = rdata_Q;
		valid_int = valid_Q;
		is_hwlp_int = is_hwlp_Q;
		if (in_valid_i) begin
			begin : sv2v_autoblock_2
				reg signed [31:0] j;
				begin : sv2v_autoblock_3
					reg signed [31:0] _sv2v_value_on_break;
					for (j = 0; j < DEPTH; j = j + 1)
						if (_sv2v_jump < 2'b10) begin
							_sv2v_jump = 2'b00;
							if (~valid_Q[j]) begin
								addr_int[(3 - j) * 32+:32] = in_addr_i;
								rdata_int[(3 - j) * 32+:32] = in_rdata_i;
								valid_int[j] = 1'b1;
								_sv2v_jump = 2'b10;
							end
							_sv2v_value_on_break = j;
						end
					if (!(_sv2v_jump < 2'b10))
						j = _sv2v_value_on_break;
					if (_sv2v_jump != 2'b11)
						_sv2v_jump = 2'b00;
				end
			end
			if (_sv2v_jump == 2'b00) begin
				if (in_replace2_i) begin
					if (valid_Q[0]) begin
						addr_int[64+:32] = in_addr_i;
						rdata_int[96+:32] = out_rdata_o;
						rdata_int[64+:32] = in_rdata_i;
						valid_int[1] = 1'b1;
						valid_int[2:3] = 1'sb0;
						is_hwlp_int[1] = in_is_hwlp_i;
					end
					else
						is_hwlp_int[0] = in_is_hwlp_i;
				end
			end
		end
	end
	assign addr_next = {addr_int[127-:30], 2'b00} + 32'h00000004;
	always @(*) begin
		if (_sv2v_0)
			;
		addr_n = addr_int;
		rdata_n = rdata_int;
		valid_n = valid_int;
		is_hwlp_n = is_hwlp_int;
		if (out_ready_i && out_valid_o) begin
			is_hwlp_n = {is_hwlp_int[1], 1'b0};
			if (is_hwlp_int[1]) begin
				addr_n[96+:32] = addr_int[95-:32];
				begin : sv2v_autoblock_4
					reg signed [31:0] i;
					for (i = 0; i < 3; i = i + 1)
						rdata_n[(3 - i) * 32+:32] = rdata_int[(3 - (i + 1)) * 32+:32];
				end
				rdata_n[0+:32] = 32'b00000000000000000000000000000000;
				valid_n = {valid_int[1:3], 1'b0};
			end
			else if (addr_int[97]) begin
				if (unaligned_is_compressed)
					addr_n[96+:32] = {addr_next[31:2], 2'b00};
				else
					addr_n[96+:32] = {addr_next[31:2], 2'b10};
				begin : sv2v_autoblock_5
					reg signed [31:0] i;
					for (i = 0; i < 3; i = i + 1)
						rdata_n[(3 - i) * 32+:32] = rdata_int[(3 - (i + 1)) * 32+:32];
				end
				rdata_n[0+:32] = 32'b00000000000000000000000000000000;
				valid_n = {valid_int[1:3], 1'b0};
			end
			else if (aligned_is_compressed)
				addr_n[96+:32] = {addr_int[127-:30], 2'b10};
			else begin
				addr_n[96+:32] = {addr_next[31:2], 2'b00};
				begin : sv2v_autoblock_6
					reg signed [31:0] i;
					for (i = 0; i < 3; i = i + 1)
						rdata_n[(3 - i) * 32+:32] = rdata_int[(3 - (i + 1)) * 32+:32];
				end
				rdata_n[0+:32] = 32'b00000000000000000000000000000000;
				valid_n = {valid_int[1:3], 1'b0};
			end
		end
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			addr_Q <= {DEPTH {32'b00000000000000000000000000000000}};
			rdata_Q <= {DEPTH {32'b00000000000000000000000000000000}};
			valid_Q <= 1'sb0;
			is_hwlp_Q <= 1'sb0;
		end
		else if (clear_i) begin
			valid_Q <= 1'sb0;
			is_hwlp_Q <= 1'sb0;
		end
		else begin
			addr_Q <= addr_n;
			rdata_Q <= rdata_n;
			valid_Q <= valid_n;
			is_hwlp_Q <= is_hwlp_n;
		end
	initial _sv2v_0 = 0;
endmodule
module riscv_prefetch_buffer (
	clk,
	rst_n,
	req_i,
	branch_i,
	addr_i,
	hwloop_i,
	hwloop_target_i,
	hwlp_branch_o,
	ready_i,
	valid_o,
	rdata_o,
	addr_o,
	is_hwlp_o,
	instr_req_o,
	instr_gnt_i,
	instr_addr_o,
	instr_rdata_i,
	instr_rvalid_i,
	instr_err_pmp_i,
	fetch_failed_o,
	busy_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire req_i;
	input wire branch_i;
	input wire [31:0] addr_i;
	input wire hwloop_i;
	input wire [31:0] hwloop_target_i;
	output wire hwlp_branch_o;
	input wire ready_i;
	output wire valid_o;
	output wire [31:0] rdata_o;
	output wire [31:0] addr_o;
	output wire is_hwlp_o;
	output reg instr_req_o;
	input wire instr_gnt_i;
	output reg [31:0] instr_addr_o;
	input wire [31:0] instr_rdata_i;
	input wire instr_rvalid_i;
	input wire instr_err_pmp_i;
	output reg fetch_failed_o;
	output wire busy_o;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [2:0] hwlp_CS;
	reg [2:0] hwlp_NS;
	reg [31:0] instr_addr_q;
	wire [31:0] fetch_addr;
	reg fetch_is_hwlp;
	reg addr_valid;
	reg fifo_valid;
	wire fifo_ready;
	reg fifo_clear;
	reg fifo_hwlp;
	wire valid_stored;
	reg hwlp_masked;
	reg hwlp_branch;
	reg hwloop_speculative;
	wire unaligned_is_compressed;
	assign busy_o = (CS != 3'd0) || instr_req_o;
	riscv_fetch_fifo fifo_i(
		.clk(clk),
		.rst_n(rst_n),
		.clear_i(fifo_clear),
		.in_addr_i(instr_addr_q),
		.in_rdata_i(instr_rdata_i),
		.in_valid_i(fifo_valid),
		.in_ready_o(fifo_ready),
		.in_replace2_i(fifo_hwlp),
		.in_is_hwlp_i(fifo_hwlp),
		.out_valid_o(valid_o),
		.out_ready_i(ready_i),
		.out_rdata_o(rdata_o),
		.out_addr_o(addr_o),
		.unaligned_is_compressed_o(unaligned_is_compressed),
		.out_valid_stored_o(valid_stored),
		.out_is_hwlp_o(is_hwlp_o)
	);
	assign fetch_addr = {instr_addr_q[31:2], 2'b00} + 32'd4;
	assign hwlp_branch_o = hwlp_branch;
	always @(*) begin
		if (_sv2v_0)
			;
		hwlp_NS = hwlp_CS;
		fifo_hwlp = 1'b0;
		fifo_clear = 1'b0;
		hwlp_branch = 1'b0;
		hwloop_speculative = 1'b0;
		hwlp_masked = 1'b0;
		case (hwlp_CS)
			3'd0:
				if (hwloop_i) begin
					hwlp_masked = ~instr_addr_q[1];
					if ((valid_o & unaligned_is_compressed) & instr_addr_q[1]) begin
						hwlp_NS = 3'd4;
						hwloop_speculative = 1'b1;
					end
					else if (instr_addr_q[1] && ~valid_o) begin
						hwlp_NS = 3'd5;
						hwloop_speculative = 1'b1;
					end
					else if (fetch_is_hwlp)
						hwlp_NS = 3'd2;
					else
						hwlp_NS = 3'd1;
					if (ready_i)
						fifo_clear = 1'b1;
				end
				else
					hwlp_masked = 1'b0;
			3'd5: begin
				hwlp_masked = 1'b1;
				if (valid_o) begin
					hwlp_NS = 3'd2;
					if (ready_i)
						fifo_clear = 1'b1;
				end
			end
			3'd4: begin
				hwlp_branch = 1'b1;
				hwlp_NS = 3'd2;
				fifo_clear = 1'b1;
			end
			3'd1: begin
				hwlp_masked = 1'b1;
				if (fetch_is_hwlp)
					hwlp_NS = 3'd2;
				if (ready_i)
					fifo_clear = 1'b1;
			end
			3'd2: begin
				hwlp_masked = 1'b0;
				fifo_hwlp = 1'b1;
				if (instr_rvalid_i & (CS != 3'd3)) begin
					if (valid_o & is_hwlp_o)
						hwlp_NS = 3'd0;
					else
						hwlp_NS = 3'd3;
				end
				else if (ready_i)
					fifo_clear = 1'b1;
			end
			3'd3: begin
				hwlp_masked = 1'b0;
				if (valid_o & is_hwlp_o)
					hwlp_NS = 3'd0;
			end
			default: begin
				hwlp_masked = 1'b0;
				hwlp_NS = 3'd0;
			end
		endcase
		if (branch_i) begin
			hwlp_NS = 3'd0;
			fifo_clear = 1'b1;
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		instr_req_o = 1'b0;
		instr_addr_o = fetch_addr;
		fifo_valid = 1'b0;
		addr_valid = 1'b0;
		fetch_is_hwlp = 1'b0;
		fetch_failed_o = 1'b0;
		NS = CS;
		case (CS)
			3'd0: begin
				instr_addr_o = fetch_addr;
				instr_req_o = 1'b0;
				if (branch_i | hwlp_branch)
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
				else if (hwlp_masked & valid_stored)
					instr_addr_o = hwloop_target_i;
				if (req_i & (((fifo_ready | branch_i) | hwlp_branch) | (hwlp_masked & valid_stored))) begin
					instr_req_o = 1'b1;
					addr_valid = 1'b1;
					if (hwlp_masked & valid_stored)
						fetch_is_hwlp = 1'b1;
					if (instr_gnt_i)
						NS = 3'd2;
					else
						NS = 3'd1;
					if (instr_err_pmp_i)
						NS = 3'd4;
				end
			end
			3'd4: begin
				instr_req_o = 1'b0;
				fetch_failed_o = valid_o == 1'b0;
				if (branch_i) begin
					instr_addr_o = addr_i;
					addr_valid = 1'b1;
					instr_req_o = 1'b1;
					fetch_failed_o = 1'b0;
					if (instr_gnt_i)
						NS = 3'd2;
					else
						NS = 3'd1;
				end
			end
			3'd1: begin
				instr_addr_o = instr_addr_q;
				instr_req_o = 1'b1;
				if (branch_i | hwlp_branch) begin
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
					addr_valid = 1'b1;
				end
				else if (hwlp_masked & valid_stored) begin
					instr_addr_o = hwloop_target_i;
					addr_valid = 1'b1;
					fetch_is_hwlp = 1'b1;
				end
				if (instr_gnt_i)
					NS = 3'd2;
				else
					NS = 3'd1;
				if (instr_err_pmp_i)
					NS = 3'd4;
			end
			3'd2: begin
				instr_addr_o = fetch_addr;
				if (branch_i | hwlp_branch)
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
				else if (hwlp_masked)
					instr_addr_o = hwloop_target_i;
				if (req_i & (((fifo_ready | branch_i) | hwlp_branch) | hwlp_masked)) begin
					if (instr_rvalid_i) begin
						instr_req_o = 1'b1;
						fifo_valid = 1'b1;
						addr_valid = 1'b1;
						if (hwlp_masked)
							fetch_is_hwlp = 1'b1;
						if (instr_gnt_i)
							NS = 3'd2;
						else
							NS = 3'd1;
						if (instr_err_pmp_i)
							NS = 3'd4;
					end
					else if (branch_i | hwlp_branch) begin
						addr_valid = 1'b1;
						NS = 3'd3;
					end
					else if (hwlp_masked & valid_o) begin
						addr_valid = 1'b1;
						fetch_is_hwlp = 1'b1;
						NS = 3'd3;
					end
				end
				else if (instr_rvalid_i) begin
					fifo_valid = 1'b1;
					NS = 3'd0;
				end
			end
			3'd3: begin
				instr_addr_o = instr_addr_q;
				if (branch_i | hwlp_branch) begin
					instr_addr_o = (branch_i ? addr_i : instr_addr_q);
					addr_valid = 1'b1;
				end
				if (instr_rvalid_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd2;
					else
						NS = 3'd1;
					if (instr_err_pmp_i)
						NS = 3'd4;
				end
			end
			default: begin
				NS = 3'd0;
				instr_req_o = 1'b0;
			end
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			CS <= 3'd0;
			hwlp_CS <= 3'd0;
			instr_addr_q <= 1'sb0;
		end
		else begin
			CS <= NS;
			hwlp_CS <= hwlp_NS;
			if (addr_valid)
				instr_addr_q <= (hwloop_speculative & ~branch_i ? hwloop_target_i : instr_addr_o);
		end
	initial _sv2v_0 = 0;
endmodule
module riscv_prefetch_L0_buffer (
	clk,
	rst_n,
	req_i,
	branch_i,
	addr_i,
	hwloop_i,
	hwloop_target_i,
	ready_i,
	valid_o,
	rdata_o,
	addr_o,
	is_hwlp_o,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	busy_o
);
	reg _sv2v_0;
	parameter RDATA_IN_WIDTH = 128;
	input wire clk;
	input wire rst_n;
	input wire req_i;
	input wire branch_i;
	input wire [31:0] addr_i;
	input wire hwloop_i;
	input wire [31:0] hwloop_target_i;
	input wire ready_i;
	output wire valid_o;
	output wire [31:0] rdata_o;
	output wire [31:0] addr_o;
	output wire is_hwlp_o;
	output wire instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] instr_rdata_i;
	output wire busy_o;
	wire busy_L0;
	reg [3:0] CS;
	reg [3:0] NS;
	reg do_fetch;
	reg do_hwlp;
	reg do_hwlp_int;
	reg use_last;
	reg save_rdata_last;
	reg use_hwlp;
	reg save_rdata_hwlp;
	reg valid;
	wire hwlp_is_crossword;
	wire is_crossword;
	wire next_is_crossword;
	wire next_valid;
	wire next_upper_compressed;
	wire fetch_possible;
	wire upper_is_compressed;
	reg [31:0] addr_q;
	reg [31:0] addr_n;
	reg [31:0] addr_int;
	wire [31:0] addr_aligned_next;
	wire [31:0] addr_real_next;
	reg is_hwlp_q;
	reg is_hwlp_n;
	reg [31:0] rdata_last_q;
	wire valid_L0;
	wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] rdata_L0;
	wire [31:0] addr_L0;
	wire fetch_valid;
	wire fetch_gnt;
	wire [31:0] rdata;
	reg [31:0] rdata_unaligned;
	wire aligned_is_compressed;
	wire unaligned_is_compressed;
	wire hwlp_aligned_is_compressed;
	wire hwlp_unaligned_is_compressed;
	riscv_L0_buffer #(.RDATA_IN_WIDTH(RDATA_IN_WIDTH)) L0_buffer_i(
		.clk(clk),
		.rst_n(rst_n),
		.prefetch_i(do_fetch),
		.prefetch_addr_i(addr_real_next),
		.branch_i(branch_i),
		.branch_addr_i(addr_i),
		.hwlp_i(do_hwlp | do_hwlp_int),
		.hwlp_addr_i(hwloop_target_i),
		.fetch_gnt_o(fetch_gnt),
		.fetch_valid_o(fetch_valid),
		.valid_o(valid_L0),
		.rdata_o(rdata_L0),
		.addr_o(addr_L0),
		.instr_req_o(instr_req_o),
		.instr_addr_o(instr_addr_o),
		.instr_gnt_i(instr_gnt_i),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_rdata_i(instr_rdata_i),
		.busy_o(busy_L0)
	);
	assign rdata = (use_last || use_hwlp ? rdata_last_q : rdata_L0[addr_o[3:2] * 32+:32]);
	wire [16:1] sv2v_tmp_34AB2;
	assign sv2v_tmp_34AB2 = rdata[31:16];
	always @(*) rdata_unaligned[15:0] = sv2v_tmp_34AB2;
	always @(*) begin
		if (_sv2v_0)
			;
		case (addr_o[3:2])
			2'b00: rdata_unaligned[31:16] = rdata_L0[47-:16];
			2'b01: rdata_unaligned[31:16] = rdata_L0[79-:16];
			2'b10: rdata_unaligned[31:16] = rdata_L0[111-:16];
			2'b11: rdata_unaligned[31:16] = rdata_L0[15-:16];
		endcase
	end
	assign unaligned_is_compressed = rdata[17:16] != 2'b11;
	assign aligned_is_compressed = rdata[1:0] != 2'b11;
	assign upper_is_compressed = rdata_L0[113-:2] != 2'b11;
	assign is_crossword = (addr_o[3:1] == 3'b111) && ~upper_is_compressed;
	assign next_is_crossword = (((addr_o[3:1] == 3'b110) && aligned_is_compressed) && ~upper_is_compressed) || (((addr_o[3:1] == 3'b101) && ~unaligned_is_compressed) && ~upper_is_compressed);
	assign next_upper_compressed = (((addr_o[3:1] == 3'b110) && aligned_is_compressed) && upper_is_compressed) || (((addr_o[3:1] == 3'b101) && ~unaligned_is_compressed) && upper_is_compressed);
	assign next_valid = (((addr_o[3:2] != 2'b11) || next_upper_compressed) && ~next_is_crossword) && valid;
	assign fetch_possible = addr_o[3:2] == 2'b11;
	assign addr_aligned_next = {addr_o[31:2], 2'b00} + 32'h00000004;
	assign addr_real_next = (next_is_crossword ? {addr_o[31:4], 4'b0000} + 32'h00000016 : {addr_o[31:2], 2'b00} + 32'h00000004);
	assign hwlp_unaligned_is_compressed = rdata_L0[81-:2] != 2'b11;
	assign hwlp_aligned_is_compressed = rdata_L0[97-:2] != 2'b11;
	assign hwlp_is_crossword = (hwloop_target_i[3:1] == 3'b111) && ~upper_is_compressed;
	always @(*) begin
		if (_sv2v_0)
			;
		addr_int = addr_o;
		if (ready_i) begin
			if (addr_o[1]) begin
				if (unaligned_is_compressed)
					addr_int = {addr_aligned_next[31:2], 2'b00};
				else
					addr_int = {addr_aligned_next[31:2], 2'b10};
			end
			else if (aligned_is_compressed)
				addr_int = {addr_o[31:2], 2'b10};
			else
				addr_int = {addr_aligned_next[31:2], 2'b00};
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		do_fetch = 1'b0;
		do_hwlp = 1'b0;
		do_hwlp_int = 1'b0;
		use_last = 1'b0;
		use_hwlp = 1'b0;
		save_rdata_last = 1'b0;
		save_rdata_hwlp = 1'b0;
		valid = 1'b0;
		addr_n = addr_int;
		is_hwlp_n = is_hwlp_q;
		if (ready_i)
			is_hwlp_n = 1'b0;
		case (CS)
			4'd0:
				;
			4'd1: begin
				valid = 1'b0;
				do_fetch = fetch_possible;
				if (fetch_valid && ~is_crossword)
					valid = 1'b1;
				if (ready_i) begin
					if (hwloop_i) begin
						addr_n = addr_o;
						NS = 4'd2;
					end
					else if (next_valid) begin
						if (fetch_gnt) begin
							save_rdata_last = 1'b1;
							NS = 4'd12;
						end
						else
							NS = 4'd10;
					end
					else if (next_is_crossword) begin
						if (fetch_gnt) begin
							save_rdata_last = 1'b1;
							NS = 4'd9;
						end
						else
							NS = 4'd8;
					end
					else if (fetch_gnt)
						NS = 4'd7;
					else
						NS = 4'd6;
				end
				else if (fetch_valid) begin
					if (is_crossword) begin
						save_rdata_last = 1'b1;
						if (fetch_gnt)
							NS = 4'd9;
						else
							NS = 4'd8;
					end
					else if (fetch_gnt) begin
						save_rdata_last = 1'b1;
						NS = 4'd12;
					end
					else
						NS = 4'd10;
				end
			end
			4'd6: begin
				do_fetch = 1'b1;
				if (fetch_gnt)
					NS = 4'd7;
			end
			4'd7: begin
				valid = fetch_valid;
				do_hwlp = hwloop_i;
				if (fetch_valid)
					NS = 4'd10;
			end
			4'd8: begin
				do_fetch = 1'b1;
				if (fetch_gnt) begin
					save_rdata_last = 1'b1;
					NS = 4'd9;
				end
			end
			4'd9: begin
				valid = fetch_valid;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (fetch_valid) begin
					if (ready_i)
						NS = 4'd10;
					else
						NS = 4'd11;
				end
			end
			4'd10: begin
				valid = 1'b1;
				do_fetch = fetch_possible;
				do_hwlp = hwloop_i;
				if (ready_i) begin
					if (next_is_crossword) begin
						do_fetch = 1'b1;
						if (fetch_gnt) begin
							save_rdata_last = 1'b1;
							NS = 4'd9;
						end
						else
							NS = 4'd8;
					end
					else if (~next_valid) begin
						if (fetch_gnt)
							NS = 4'd7;
						else
							NS = 4'd6;
					end
					else if (fetch_gnt) begin
						if (next_upper_compressed) begin
							save_rdata_last = 1'b1;
							NS = 4'd12;
						end
					end
				end
				else if (fetch_gnt) begin
					save_rdata_last = 1'b1;
					NS = 4'd12;
				end
			end
			4'd11: begin
				valid = 1'b1;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (ready_i)
					NS = 4'd10;
			end
			4'd12: begin
				valid = 1'b1;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (ready_i) begin
					if (fetch_valid) begin
						if (next_is_crossword)
							NS = 4'd11;
						else if (next_upper_compressed)
							NS = 4'd13;
						else
							NS = 4'd10;
					end
					else if (next_is_crossword)
						NS = 4'd9;
					else if (next_upper_compressed)
						NS = 4'd12;
					else
						NS = 4'd7;
				end
				else if (fetch_valid)
					NS = 4'd13;
			end
			4'd13: begin
				valid = 1'b1;
				use_last = 1'b1;
				do_hwlp = hwloop_i;
				if (ready_i) begin
					if (next_is_crossword)
						NS = 4'd11;
					else if (next_upper_compressed)
						NS = 4'd13;
					else
						NS = 4'd10;
				end
			end
			4'd2: begin
				do_hwlp_int = 1'b1;
				if (fetch_gnt) begin
					is_hwlp_n = 1'b1;
					addr_n = hwloop_target_i;
					NS = 4'd1;
				end
			end
			4'd3: begin
				valid = 1'b1;
				use_hwlp = 1'b1;
				if (ready_i) begin
					addr_n = hwloop_target_i;
					if (fetch_valid) begin
						is_hwlp_n = 1'b1;
						if (hwlp_is_crossword)
							NS = 4'd8;
						else
							NS = 4'd10;
					end
					else
						NS = 4'd4;
				end
				else if (fetch_valid)
					NS = 4'd5;
			end
			4'd4: begin
				use_hwlp = 1'b1;
				if (fetch_valid) begin
					is_hwlp_n = 1'b1;
					if ((addr_L0[3:1] == 3'b111) && ~upper_is_compressed)
						NS = 4'd8;
					else
						NS = 4'd10;
				end
			end
			4'd5: begin
				valid = 1'b1;
				use_hwlp = 1'b1;
				if (ready_i) begin
					is_hwlp_n = 1'b1;
					addr_n = hwloop_target_i;
					if (hwlp_is_crossword)
						NS = 4'd8;
					else
						NS = 4'd10;
				end
			end
		endcase
		if (branch_i) begin
			is_hwlp_n = 1'b0;
			addr_n = addr_i;
			NS = 4'd1;
		end
		else if (hwloop_i) begin
			if (do_hwlp) begin
				if (ready_i) begin
					if (fetch_gnt) begin
						is_hwlp_n = 1'b1;
						addr_n = hwloop_target_i;
						NS = 4'd1;
					end
					else begin
						addr_n = addr_o;
						NS = 4'd2;
					end
				end
				else if (fetch_gnt) begin
					save_rdata_hwlp = 1'b1;
					NS = 4'd3;
				end
			end
		end
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			addr_q <= 1'sb0;
			is_hwlp_q <= 1'b0;
			CS <= 4'd0;
			rdata_last_q <= 1'sb0;
		end
		else begin
			addr_q <= addr_n;
			is_hwlp_q <= is_hwlp_n;
			CS <= NS;
			if (save_rdata_hwlp)
				rdata_last_q <= rdata_o;
			else if (save_rdata_last) begin
				if (ready_i)
					rdata_last_q <= rdata_L0[96+:32];
				else
					rdata_last_q <= rdata;
			end
		end
	assign rdata_o = (~addr_o[1] || use_hwlp ? rdata : rdata_unaligned);
	assign valid_o = valid & ~branch_i;
	assign addr_o = addr_q;
	assign is_hwlp_o = is_hwlp_q & ~branch_i;
	assign busy_o = busy_L0;
	initial _sv2v_0 = 0;
endmodule
module riscv_L0_buffer (
	clk,
	rst_n,
	prefetch_i,
	prefetch_addr_i,
	branch_i,
	branch_addr_i,
	hwlp_i,
	hwlp_addr_i,
	fetch_gnt_o,
	fetch_valid_o,
	valid_o,
	rdata_o,
	addr_o,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	busy_o
);
	reg _sv2v_0;
	parameter RDATA_IN_WIDTH = 128;
	input wire clk;
	input wire rst_n;
	input wire prefetch_i;
	input wire [31:0] prefetch_addr_i;
	input wire branch_i;
	input wire [31:0] branch_addr_i;
	input wire hwlp_i;
	input wire [31:0] hwlp_addr_i;
	output wire fetch_gnt_o;
	output reg fetch_valid_o;
	output wire valid_o;
	output wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] rdata_o;
	output wire [31:0] addr_o;
	output reg instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [((RDATA_IN_WIDTH / 32) * 32) - 1:0] instr_rdata_i;
	output wire busy_o;
	reg [2:0] CS;
	reg [2:0] NS;
	reg [127:0] L0_buffer;
	reg [31:0] addr_q;
	reg [31:0] instr_addr_int;
	reg valid;
	always @(*) begin
		if (_sv2v_0)
			;
		NS = CS;
		valid = 1'b0;
		instr_req_o = 1'b0;
		instr_addr_int = 1'sb0;
		fetch_valid_o = 1'b0;
		case (CS)
			3'd0: begin
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = prefetch_addr_i;
				if ((branch_i | hwlp_i) | prefetch_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			3'd2: begin
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = addr_q;
				if (branch_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
				else begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			3'd3: begin
				valid = instr_rvalid_i;
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = prefetch_addr_i;
				if (branch_i) begin
					if (instr_rvalid_i) begin
						fetch_valid_o = 1'b1;
						instr_req_o = 1'b1;
						if (instr_gnt_i)
							NS = 3'd3;
						else
							NS = 3'd2;
					end
					else
						NS = 3'd4;
				end
				else if (instr_rvalid_i) begin
					fetch_valid_o = 1'b1;
					if (prefetch_i | hwlp_i) begin
						instr_req_o = 1'b1;
						if (instr_gnt_i)
							NS = 3'd3;
						else
							NS = 3'd2;
					end
					else
						NS = 3'd1;
				end
			end
			3'd1: begin
				valid = 1'b1;
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else if (hwlp_i)
					instr_addr_int = hwlp_addr_i;
				else
					instr_addr_int = prefetch_addr_i;
				if ((branch_i | hwlp_i) | prefetch_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			3'd4: begin
				if (branch_i)
					instr_addr_int = branch_addr_i;
				else
					instr_addr_int = addr_q;
				if (instr_rvalid_i) begin
					instr_req_o = 1'b1;
					if (instr_gnt_i)
						NS = 3'd3;
					else
						NS = 3'd2;
				end
			end
			default: NS = 3'd0;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			CS <= 3'd0;
			L0_buffer <= 1'sb0;
			addr_q <= 1'sb0;
		end
		else begin
			CS <= NS;
			if (instr_rvalid_i)
				L0_buffer <= instr_rdata_i;
			if ((branch_i | hwlp_i) | prefetch_i)
				addr_q <= instr_addr_int;
		end
	assign instr_addr_o = {instr_addr_int[31:4], 4'b0000};
	assign rdata_o = (instr_rvalid_i ? instr_rdata_i : L0_buffer);
	assign addr_o = addr_q;
	assign valid_o = valid & ~branch_i;
	assign busy_o = ((CS != 3'd0) && (CS != 3'd1)) || instr_req_o;
	assign fetch_gnt_o = instr_gnt_i;
	initial _sv2v_0 = 0;
endmodule
module riscv_hwloop_regs (
	clk,
	rst_n,
	hwlp_start_data_i,
	hwlp_end_data_i,
	hwlp_cnt_data_i,
	hwlp_we_i,
	hwlp_regid_i,
	valid_i,
	hwlp_dec_cnt_i,
	hwlp_start_addr_o,
	hwlp_end_addr_o,
	hwlp_counter_o
);
	parameter N_REGS = 2;
	parameter N_REG_BITS = $clog2(N_REGS);
	input wire clk;
	input wire rst_n;
	input wire [31:0] hwlp_start_data_i;
	input wire [31:0] hwlp_end_data_i;
	input wire [31:0] hwlp_cnt_data_i;
	input wire [2:0] hwlp_we_i;
	input wire [N_REG_BITS - 1:0] hwlp_regid_i;
	input wire valid_i;
	input wire [N_REGS - 1:0] hwlp_dec_cnt_i;
	output wire [(N_REGS * 32) - 1:0] hwlp_start_addr_o;
	output wire [(N_REGS * 32) - 1:0] hwlp_end_addr_o;
	output wire [(N_REGS * 32) - 1:0] hwlp_counter_o;
	reg [(N_REGS * 32) - 1:0] hwlp_start_q;
	reg [(N_REGS * 32) - 1:0] hwlp_end_q;
	reg [(N_REGS * 32) - 1:0] hwlp_counter_q;
	wire [(N_REGS * 32) - 1:0] hwlp_counter_n;
	reg [31:0] i;
	assign hwlp_start_addr_o = hwlp_start_q;
	assign hwlp_end_addr_o = hwlp_end_q;
	assign hwlp_counter_o = hwlp_counter_q;
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_START
		if (rst_n == 1'b0)
			hwlp_start_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else if (hwlp_we_i[0] == 1'b1)
			hwlp_start_q[hwlp_regid_i * 32+:32] <= hwlp_start_data_i;
	end
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_END
		if (rst_n == 1'b0)
			hwlp_end_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else if (hwlp_we_i[1] == 1'b1)
			hwlp_end_q[hwlp_regid_i * 32+:32] <= hwlp_end_data_i;
	end
	genvar _gv_k_1;
	generate
		for (_gv_k_1 = 0; _gv_k_1 < N_REGS; _gv_k_1 = _gv_k_1 + 1) begin : genblk1
			localparam k = _gv_k_1;
			assign hwlp_counter_n[k * 32+:32] = hwlp_counter_q[k * 32+:32] - 1;
		end
	endgenerate
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_COUNTER
		if (rst_n == 1'b0)
			hwlp_counter_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else
			for (i = 0; i < N_REGS; i = i + 1)
				if ((hwlp_we_i[2] == 1'b1) && (i == hwlp_regid_i))
					hwlp_counter_q[i * 32+:32] <= hwlp_cnt_data_i;
				else if (hwlp_dec_cnt_i[i] && valid_i)
					hwlp_counter_q[i * 32+:32] <= hwlp_counter_n[i * 32+:32];
	end
endmodule
module riscv_hwloop_controller (
	current_pc_i,
	hwlp_start_addr_i,
	hwlp_end_addr_i,
	hwlp_counter_i,
	hwlp_dec_cnt_o,
	hwlp_dec_cnt_id_i,
	hwlp_jump_o,
	hwlp_targ_addr_o
);
	reg _sv2v_0;
	parameter N_REGS = 2;
	input wire [31:0] current_pc_i;
	input wire [(N_REGS * 32) - 1:0] hwlp_start_addr_i;
	input wire [(N_REGS * 32) - 1:0] hwlp_end_addr_i;
	input wire [(N_REGS * 32) - 1:0] hwlp_counter_i;
	output reg [N_REGS - 1:0] hwlp_dec_cnt_o;
	input wire [N_REGS - 1:0] hwlp_dec_cnt_id_i;
	output wire hwlp_jump_o;
	output reg [31:0] hwlp_targ_addr_o;
	reg [N_REGS - 1:0] pc_is_end_addr;
	integer j;
	genvar _gv_i_4;
	generate
		for (_gv_i_4 = 0; _gv_i_4 < N_REGS; _gv_i_4 = _gv_i_4 + 1) begin : genblk1
			localparam i = _gv_i_4;
			always @(*) begin
				pc_is_end_addr[i] = 1'b0;
				if (current_pc_i == hwlp_end_addr_i[i * 32+:32]) begin
					if (hwlp_counter_i[(i * 32) + 31-:30] != 30'h00000000)
						pc_is_end_addr[i] = 1'b1;
					else
						case (hwlp_counter_i[(i * 32) + 1-:2])
							2'b11: pc_is_end_addr[i] = 1'b1;
							2'b10: pc_is_end_addr[i] = ~hwlp_dec_cnt_id_i[i];
							2'b01, 2'b00: pc_is_end_addr[i] = 1'b0;
						endcase
				end
			end
		end
	endgenerate
	always @(*) begin : sv2v_autoblock_1
		reg [0:1] _sv2v_jump;
		_sv2v_jump = 2'b00;
		if (_sv2v_0)
			;
		hwlp_targ_addr_o = 1'sb0;
		hwlp_dec_cnt_o = 1'sb0;
		begin : sv2v_autoblock_2
			integer _sv2v_value_on_break;
			for (j = 0; j < N_REGS; j = j + 1)
				if (_sv2v_jump < 2'b10) begin
					_sv2v_jump = 2'b00;
					if (pc_is_end_addr[j]) begin
						hwlp_targ_addr_o = hwlp_start_addr_i[j * 32+:32];
						hwlp_dec_cnt_o[j] = 1'b1;
						_sv2v_jump = 2'b10;
					end
					_sv2v_value_on_break = j;
				end
			if (!(_sv2v_jump < 2'b10))
				j = _sv2v_value_on_break;
			if (_sv2v_jump != 2'b11)
				_sv2v_jump = 2'b00;
		end
	end
	assign hwlp_jump_o = |pc_is_end_addr;
	initial _sv2v_0 = 0;
endmodule
module riscv_mult (
	clk,
	rst_n,
	enable_i,
	operator_i,
	short_subword_i,
	short_signed_i,
	op_a_i,
	op_b_i,
	op_c_i,
	imm_i,
	dot_signed_i,
	dot_op_a_i,
	dot_op_b_i,
	dot_op_c_i,
	is_clpx_i,
	clpx_shift_i,
	clpx_img_i,
	result_o,
	multicycle_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	parameter SHARED_DSP_MULT = 1;
	input wire clk;
	input wire rst_n;
	input wire enable_i;
	input wire [2:0] operator_i;
	input wire short_subword_i;
	input wire [1:0] short_signed_i;
	input wire [31:0] op_a_i;
	input wire [31:0] op_b_i;
	input wire [31:0] op_c_i;
	input wire [4:0] imm_i;
	input wire [1:0] dot_signed_i;
	input wire [31:0] dot_op_a_i;
	input wire [31:0] dot_op_b_i;
	input wire [31:0] dot_op_c_i;
	input wire is_clpx_i;
	input wire [1:0] clpx_shift_i;
	input wire clpx_img_i;
	output reg [31:0] result_o;
	output reg multicycle_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [16:0] short_op_a;
	wire [16:0] short_op_b;
	wire [32:0] short_op_c;
	wire [33:0] short_mul;
	wire [33:0] short_mac;
	wire [31:0] short_round;
	wire [31:0] short_round_tmp;
	wire [33:0] short_result;
	wire short_mac_msb1;
	wire short_mac_msb0;
	wire [4:0] short_imm;
	wire [1:0] short_subword;
	wire [1:0] short_signed;
	wire short_shift_arith;
	reg [4:0] mulh_imm;
	reg [1:0] mulh_subword;
	reg [1:0] mulh_signed;
	reg mulh_shift_arith;
	reg mulh_carry_q;
	reg mulh_active;
	reg mulh_save;
	reg mulh_clearcarry;
	reg mulh_ready;
	reg [2:0] mulh_CS;
	reg [2:0] mulh_NS;
	assign short_round_tmp = 32'h00000001 << imm_i;
	localparam riscv_defines_MUL_IR = 3'b011;
	assign short_round = (operator_i == riscv_defines_MUL_IR ? {1'b0, short_round_tmp[31:1]} : {32 {1'sb0}});
	assign short_op_a[15:0] = (short_subword[0] ? op_a_i[31:16] : op_a_i[15:0]);
	assign short_op_b[15:0] = (short_subword[1] ? op_b_i[31:16] : op_b_i[15:0]);
	assign short_op_a[16] = short_signed[0] & short_op_a[15];
	assign short_op_b[16] = short_signed[1] & short_op_b[15];
	assign short_op_c = (mulh_active ? $signed({mulh_carry_q, op_c_i}) : $signed(op_c_i));
	assign short_mul = $signed(short_op_a) * $signed(short_op_b);
	assign short_mac = ($signed(short_op_c) + $signed(short_mul)) + $signed(short_round);
	assign short_result = $signed({short_shift_arith & short_mac_msb1, short_shift_arith & short_mac_msb0, short_mac[31:0]}) >>> short_imm;
	assign short_imm = (mulh_active ? mulh_imm : imm_i);
	assign short_subword = (mulh_active ? mulh_subword : {2 {short_subword_i}});
	assign short_signed = (mulh_active ? mulh_signed : short_signed_i);
	assign short_shift_arith = (mulh_active ? mulh_shift_arith : short_signed_i[0]);
	assign short_mac_msb1 = (mulh_active ? short_mac[33] : short_mac[31]);
	assign short_mac_msb0 = (mulh_active ? short_mac[32] : short_mac[31]);
	localparam riscv_defines_MUL_H = 3'b110;
	always @(*) begin
		if (_sv2v_0)
			;
		mulh_NS = mulh_CS;
		mulh_imm = 5'd0;
		mulh_subword = 2'b00;
		mulh_signed = 2'b00;
		mulh_shift_arith = 1'b0;
		mulh_ready = 1'b0;
		mulh_active = 1'b1;
		mulh_save = 1'b0;
		mulh_clearcarry = 1'b0;
		multicycle_o = 1'b0;
		case (mulh_CS)
			3'd0: begin
				mulh_active = 1'b0;
				mulh_ready = 1'b1;
				mulh_save = 1'b0;
				if ((operator_i == riscv_defines_MUL_H) && enable_i) begin
					mulh_ready = 1'b0;
					mulh_NS = 3'd1;
				end
			end
			3'd1: begin
				multicycle_o = 1'b1;
				mulh_imm = 5'd16;
				mulh_active = 1'b1;
				mulh_save = 1'b0;
				mulh_NS = 3'd2;
			end
			3'd2: begin
				multicycle_o = 1'b1;
				mulh_signed = {short_signed_i[1], 1'b0};
				mulh_subword = 2'b10;
				mulh_save = 1'b1;
				mulh_shift_arith = 1'b1;
				mulh_NS = 3'd3;
			end
			3'd3: begin
				multicycle_o = 1'b1;
				mulh_signed = {1'b0, short_signed_i[0]};
				mulh_subword = 2'b01;
				mulh_imm = 5'd16;
				mulh_save = 1'b1;
				mulh_clearcarry = 1'b1;
				mulh_shift_arith = 1'b1;
				mulh_NS = 3'd4;
			end
			3'd4: begin
				mulh_signed = short_signed_i;
				mulh_subword = 2'b11;
				mulh_ready = 1'b1;
				if (ex_ready_i)
					mulh_NS = 3'd0;
			end
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			mulh_CS <= 3'd0;
			mulh_carry_q <= 1'b0;
		end
		else begin
			mulh_CS <= mulh_NS;
			if (mulh_save)
				mulh_carry_q <= ~mulh_clearcarry & short_mac[32];
			else if (ex_ready_i)
				mulh_carry_q <= 1'b0;
		end
	wire [31:0] int_op_a_msu;
	wire [31:0] int_op_b_msu;
	wire [31:0] int_result;
	wire int_is_msu;
	localparam riscv_defines_MUL_MSU32 = 3'b001;
	assign int_is_msu = operator_i == riscv_defines_MUL_MSU32;
	assign int_op_a_msu = op_a_i ^ {32 {int_is_msu}};
	assign int_op_b_msu = op_b_i & {32 {int_is_msu}};
	assign int_result = ($signed(op_c_i) + $signed(int_op_b_msu)) + ($signed(int_op_a_msu) * $signed(op_b_i));
	wire [31:0] dot_char_result;
	wire [32:0] dot_short_result;
	wire [31:0] accumulator;
	wire [15:0] clpx_shift_result;
	generate
		if (SHARED_DSP_MULT == 0) begin : genblk1
			wire [35:0] dot_char_op_a;
			wire [35:0] dot_char_op_b;
			wire [71:0] dot_char_mul;
			wire [33:0] dot_short_op_a;
			wire [33:0] dot_short_op_b;
			wire [67:0] dot_short_mul;
			wire [16:0] dot_short_op_a_1_neg;
			wire [31:0] dot_short_op_b_ext;
			assign dot_char_op_a[0+:9] = {dot_signed_i[1] & dot_op_a_i[7], dot_op_a_i[7:0]};
			assign dot_char_op_a[9+:9] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15:8]};
			assign dot_char_op_a[18+:9] = {dot_signed_i[1] & dot_op_a_i[23], dot_op_a_i[23:16]};
			assign dot_char_op_a[27+:9] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:24]};
			assign dot_char_op_b[0+:9] = {dot_signed_i[0] & dot_op_b_i[7], dot_op_b_i[7:0]};
			assign dot_char_op_b[9+:9] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:8]};
			assign dot_char_op_b[18+:9] = {dot_signed_i[0] & dot_op_b_i[23], dot_op_b_i[23:16]};
			assign dot_char_op_b[27+:9] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:24]};
			assign dot_char_mul[0+:18] = $signed(dot_char_op_a[0+:9]) * $signed(dot_char_op_b[0+:9]);
			assign dot_char_mul[18+:18] = $signed(dot_char_op_a[9+:9]) * $signed(dot_char_op_b[9+:9]);
			assign dot_char_mul[36+:18] = $signed(dot_char_op_a[18+:9]) * $signed(dot_char_op_b[18+:9]);
			assign dot_char_mul[54+:18] = $signed(dot_char_op_a[27+:9]) * $signed(dot_char_op_b[27+:9]);
			assign dot_char_result = ((($signed(dot_char_mul[0+:18]) + $signed(dot_char_mul[18+:18])) + $signed(dot_char_mul[36+:18])) + $signed(dot_char_mul[54+:18])) + $signed(dot_op_c_i);
			assign dot_short_op_a[0+:17] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15:0]};
			assign dot_short_op_a[17+:17] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:16]};
			assign dot_short_op_a_1_neg = dot_short_op_a[17+:17] ^ {17 {is_clpx_i & ~clpx_img_i}};
			assign dot_short_op_b[0+:17] = (is_clpx_i & clpx_img_i ? {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]} : {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:0]});
			assign dot_short_op_b[17+:17] = (is_clpx_i & clpx_img_i ? {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:0]} : {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]});
			assign dot_short_mul[0+:34] = $signed(dot_short_op_a[0+:17]) * $signed(dot_short_op_b[0+:17]);
			assign dot_short_mul[34+:34] = $signed(dot_short_op_a_1_neg) * $signed(dot_short_op_b[17+:17]);
			assign dot_short_op_b_ext = $signed(dot_short_op_b[17+:17]);
			assign accumulator = (is_clpx_i ? dot_short_op_b_ext & {32 {~clpx_img_i}} : $signed(dot_op_c_i));
			assign dot_short_result = ($signed(dot_short_mul[31-:32]) + $signed(dot_short_mul[65-:32])) + $signed(accumulator);
			assign clpx_shift_result = $signed(dot_short_result[31:15]) >>> clpx_shift_i;
		end
		else begin : genblk1
			assign dot_char_result = 1'sb0;
			assign dot_short_result = 1'sb0;
		end
	endgenerate
	localparam riscv_defines_MUL_DOT16 = 3'b101;
	localparam riscv_defines_MUL_DOT8 = 3'b100;
	localparam riscv_defines_MUL_I = 3'b010;
	localparam riscv_defines_MUL_MAC32 = 3'b000;
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sb0;
		case (operator_i)
			riscv_defines_MUL_MAC32, riscv_defines_MUL_MSU32: result_o = int_result[31:0];
			riscv_defines_MUL_I, riscv_defines_MUL_IR, riscv_defines_MUL_H: result_o = short_result[31:0];
			riscv_defines_MUL_DOT8: result_o = dot_char_result[31:0];
			riscv_defines_MUL_DOT16:
				if (is_clpx_i) begin
					if (clpx_img_i) begin
						result_o[31:16] = clpx_shift_result;
						result_o[15:0] = dot_op_c_i[15:0];
					end
					else begin
						result_o[15:0] = clpx_shift_result;
						result_o[31:16] = dot_op_c_i[31:16];
					end
				end
				else
					result_o = dot_short_result[31:0];
			default:
				;
		endcase
	end
	assign ready_o = mulh_ready;
	initial _sv2v_0 = 0;
endmodule
module register_file_test_wrap (
	clk,
	rst_n,
	test_en_i,
	raddr_a_i,
	rdata_a_o,
	raddr_b_i,
	rdata_b_o,
	raddr_c_i,
	rdata_c_o,
	waddr_a_i,
	wdata_a_i,
	we_a_i,
	waddr_b_i,
	wdata_b_i,
	we_b_i,
	BIST,
	CSN_T,
	WEN_T,
	A_T,
	D_T,
	Q_T
);
	parameter ADDR_WIDTH = 5;
	parameter DATA_WIDTH = 32;
	parameter FPU = 0;
	parameter Zfinx = 0;
	input wire clk;
	input wire rst_n;
	input wire test_en_i;
	input wire [ADDR_WIDTH - 1:0] raddr_a_i;
	output wire [DATA_WIDTH - 1:0] rdata_a_o;
	input wire [ADDR_WIDTH - 1:0] raddr_b_i;
	output wire [DATA_WIDTH - 1:0] rdata_b_o;
	input wire [ADDR_WIDTH - 1:0] raddr_c_i;
	output wire [DATA_WIDTH - 1:0] rdata_c_o;
	input wire [ADDR_WIDTH - 1:0] waddr_a_i;
	input wire [DATA_WIDTH - 1:0] wdata_a_i;
	input wire we_a_i;
	input wire [ADDR_WIDTH - 1:0] waddr_b_i;
	input wire [DATA_WIDTH - 1:0] wdata_b_i;
	input wire we_b_i;
	input wire BIST;
	input wire CSN_T;
	input wire WEN_T;
	input wire [ADDR_WIDTH - 1:0] A_T;
	input wire [DATA_WIDTH - 1:0] D_T;
	output wire [DATA_WIDTH - 1:0] Q_T;
	wire [ADDR_WIDTH - 1:0] ReadAddr_a_muxed;
	wire WriteEnable_a_muxed;
	wire [ADDR_WIDTH - 1:0] WriteAddr_a_muxed;
	wire [DATA_WIDTH - 1:0] WriteData_a_muxed;
	wire WriteEnable_b_muxed;
	wire [ADDR_WIDTH - 1:0] WriteAddr_b_muxed;
	wire [DATA_WIDTH - 1:0] WriteData_b_muxed;
	reg [ADDR_WIDTH - 1:0] TestReadAddr_Q;
	assign WriteData_a_muxed = (BIST ? D_T : wdata_a_i);
	assign WriteAddr_a_muxed = (BIST ? {1'b0, ~A_T[ADDR_WIDTH - 2:0]} : waddr_a_i);
	assign WriteEnable_a_muxed = (BIST ? (CSN_T == 1'b0) && (WEN_T == 1'b0) : we_a_i);
	assign WriteData_b_muxed = (BIST ? {DATA_WIDTH {1'sb0}} : wdata_b_i);
	assign WriteAddr_b_muxed = (BIST ? {ADDR_WIDTH {1'sb0}} : waddr_b_i);
	assign WriteEnable_b_muxed = (BIST ? 1'b0 : we_b_i);
	assign ReadAddr_a_muxed = (BIST ? TestReadAddr_Q : raddr_a_i);
	assign Q_T = rdata_a_o;
	always @(posedge clk or negedge rst_n) begin : proc_
		if (~rst_n)
			TestReadAddr_Q <= 1'sb0;
		else if ((CSN_T == 1'b0) && (WEN_T == 1'b1))
			TestReadAddr_Q <= {1'b0, ~A_T[ADDR_WIDTH - 2:0]};
	end
	riscv_register_file #(
		.ADDR_WIDTH(ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH),
		.FPU(FPU),
		.Zfinx(Zfinx)
	) riscv_register_file_i(
		.clk(clk),
		.rst_n(rst_n),
		.test_en_i(test_en_i),
		.raddr_a_i(ReadAddr_a_muxed),
		.rdata_a_o(rdata_a_o),
		.raddr_b_i(raddr_b_i),
		.rdata_b_o(rdata_b_o),
		.raddr_c_i(raddr_c_i),
		.rdata_c_o(rdata_c_o),
		.waddr_a_i(WriteAddr_a_muxed),
		.wdata_a_i(WriteData_a_muxed),
		.we_a_i(WriteEnable_a_muxed),
		.waddr_b_i(WriteAddr_b_muxed),
		.wdata_b_i(WriteData_b_muxed),
		.we_b_i(WriteEnable_b_muxed)
	);
endmodule
module riscv_int_controller (
	clk,
	rst_n,
	irq_req_ctrl_o,
	irq_sec_ctrl_o,
	irq_id_ctrl_o,
	ctrl_ack_i,
	ctrl_kill_i,
	irq_pending_i,
	irq_sec_i,
	irq_id_i,
	m_IE_i,
	u_IE_i,
	current_priv_lvl_i
);
	parameter PULP_SECURE = 0;
	input wire clk;
	input wire rst_n;
	output wire irq_req_ctrl_o;
	output wire irq_sec_ctrl_o;
	output wire [5:0] irq_id_ctrl_o;
	input wire ctrl_ack_i;
	input wire ctrl_kill_i;
	input wire irq_pending_i;
	input wire irq_sec_i;
	input wire [5:0] irq_id_i;
	input wire m_IE_i;
	input wire u_IE_i;
	input wire [1:0] current_priv_lvl_i;
	reg [1:0] exc_ctrl_cs;
	wire irq_enable_ext;
	reg [5:0] irq_id_q;
	reg irq_sec_q;
	generate
		if (PULP_SECURE) begin : genblk1
			assign irq_enable_ext = ((u_IE_i | irq_sec_i) & (current_priv_lvl_i == 2'b00)) | (m_IE_i & (current_priv_lvl_i == 2'b11));
		end
		else begin : genblk1
			assign irq_enable_ext = m_IE_i;
		end
	endgenerate
	assign irq_req_ctrl_o = exc_ctrl_cs == 2'd1;
	assign irq_sec_ctrl_o = irq_sec_q;
	assign irq_id_ctrl_o = irq_id_q;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			irq_id_q <= 1'sb0;
			irq_sec_q <= 1'b0;
			exc_ctrl_cs <= 2'd0;
		end
		else
			case (exc_ctrl_cs)
				2'd0:
					if (irq_enable_ext & irq_pending_i) begin
						exc_ctrl_cs <= 2'd1;
						irq_id_q <= irq_id_i;
						irq_sec_q <= irq_sec_i;
					end
				2'd1:
					case (1'b1)
						ctrl_ack_i: exc_ctrl_cs <= 2'd2;
						ctrl_kill_i: exc_ctrl_cs <= 2'd0;
						default: exc_ctrl_cs <= 2'd1;
					endcase
				2'd2: begin
					irq_sec_q <= 1'b0;
					exc_ctrl_cs <= 2'd0;
				end
			endcase
endmodule
module riscv_ex_stage (
	clk,
	rst_n,
	alu_operator_i,
	alu_operand_a_i,
	alu_operand_b_i,
	alu_operand_c_i,
	alu_en_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	alu_vec_mode_i,
	alu_is_clpx_i,
	alu_is_subrot_i,
	alu_clpx_shift_i,
	mult_operator_i,
	mult_operand_a_i,
	mult_operand_b_i,
	mult_operand_c_i,
	mult_en_i,
	mult_sel_subword_i,
	mult_signed_mode_i,
	mult_imm_i,
	mult_dot_op_a_i,
	mult_dot_op_b_i,
	mult_dot_op_c_i,
	mult_dot_signed_i,
	mult_is_clpx_i,
	mult_clpx_shift_i,
	mult_clpx_img_i,
	mult_multicycle_o,
	fpu_prec_i,
	fpu_fflags_o,
	fpu_fflags_we_o,
	apu_en_i,
	apu_op_i,
	apu_lat_i,
	apu_operands_i,
	apu_waddr_i,
	apu_flags_i,
	apu_read_regs_i,
	apu_read_regs_valid_i,
	apu_read_dep_o,
	apu_write_regs_i,
	apu_write_regs_valid_i,
	apu_write_dep_o,
	apu_perf_type_o,
	apu_perf_cont_o,
	apu_perf_wb_o,
	apu_busy_o,
	apu_ready_wb_o,
	apu_master_req_o,
	apu_master_ready_o,
	apu_master_gnt_i,
	apu_master_operands_o,
	apu_master_op_o,
	apu_master_valid_i,
	apu_master_result_i,
	lsu_en_i,
	lsu_rdata_i,
	branch_in_ex_i,
	regfile_alu_waddr_i,
	regfile_alu_we_i,
	regfile_we_i,
	regfile_waddr_i,
	csr_access_i,
	csr_rdata_i,
	regfile_waddr_wb_o,
	regfile_we_wb_o,
	regfile_wdata_wb_o,
	regfile_alu_waddr_fw_o,
	regfile_alu_we_fw_o,
	regfile_alu_wdata_fw_o,
	jump_target_o,
	branch_decision_o,
	lsu_ready_ex_i,
	lsu_err_i,
	ex_ready_o,
	ex_valid_o,
	wb_ready_i
);
	reg _sv2v_0;
	parameter FPU = 0;
	parameter FP_DIVSQRT = 0;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	input wire clk;
	input wire rst_n;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	input wire [6:0] alu_operator_i;
	input wire [31:0] alu_operand_a_i;
	input wire [31:0] alu_operand_b_i;
	input wire [31:0] alu_operand_c_i;
	input wire alu_en_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	input wire [1:0] alu_vec_mode_i;
	input wire alu_is_clpx_i;
	input wire alu_is_subrot_i;
	input wire [1:0] alu_clpx_shift_i;
	input wire [2:0] mult_operator_i;
	input wire [31:0] mult_operand_a_i;
	input wire [31:0] mult_operand_b_i;
	input wire [31:0] mult_operand_c_i;
	input wire mult_en_i;
	input wire mult_sel_subword_i;
	input wire [1:0] mult_signed_mode_i;
	input wire [4:0] mult_imm_i;
	input wire [31:0] mult_dot_op_a_i;
	input wire [31:0] mult_dot_op_b_i;
	input wire [31:0] mult_dot_op_c_i;
	input wire [1:0] mult_dot_signed_i;
	input wire mult_is_clpx_i;
	input wire [1:0] mult_clpx_shift_i;
	input wire mult_clpx_img_i;
	output wire mult_multicycle_o;
	localparam riscv_defines_C_PC = 5;
	input wire [4:0] fpu_prec_i;
	localparam riscv_defines_C_FFLAG = 5;
	output wire [4:0] fpu_fflags_o;
	output wire fpu_fflags_we_o;
	input wire apu_en_i;
	input wire [APU_WOP_CPU - 1:0] apu_op_i;
	input wire [1:0] apu_lat_i;
	input wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands_i;
	input wire [5:0] apu_waddr_i;
	input wire [APU_NDSFLAGS_CPU - 1:0] apu_flags_i;
	input wire [17:0] apu_read_regs_i;
	input wire [2:0] apu_read_regs_valid_i;
	output wire apu_read_dep_o;
	input wire [11:0] apu_write_regs_i;
	input wire [1:0] apu_write_regs_valid_i;
	output wire apu_write_dep_o;
	output wire apu_perf_type_o;
	output wire apu_perf_cont_o;
	output wire apu_perf_wb_o;
	output wire apu_busy_o;
	output wire apu_ready_wb_o;
	output wire apu_master_req_o;
	output wire apu_master_ready_o;
	input wire apu_master_gnt_i;
	output wire [(APU_NARGS_CPU * 32) - 1:0] apu_master_operands_o;
	output wire [APU_WOP_CPU - 1:0] apu_master_op_o;
	input wire apu_master_valid_i;
	input wire [31:0] apu_master_result_i;
	input wire lsu_en_i;
	input wire [31:0] lsu_rdata_i;
	input wire branch_in_ex_i;
	input wire [5:0] regfile_alu_waddr_i;
	input wire regfile_alu_we_i;
	input wire regfile_we_i;
	input wire [5:0] regfile_waddr_i;
	input wire csr_access_i;
	input wire [31:0] csr_rdata_i;
	output reg [5:0] regfile_waddr_wb_o;
	output reg regfile_we_wb_o;
	output reg [31:0] regfile_wdata_wb_o;
	output reg [5:0] regfile_alu_waddr_fw_o;
	output reg regfile_alu_we_fw_o;
	output reg [31:0] regfile_alu_wdata_fw_o;
	output wire [31:0] jump_target_o;
	output wire branch_decision_o;
	input wire lsu_ready_ex_i;
	input wire lsu_err_i;
	output wire ex_ready_o;
	output wire ex_valid_o;
	input wire wb_ready_i;
	wire [31:0] alu_result;
	wire [31:0] mult_result;
	wire alu_cmp_result;
	reg regfile_we_lsu;
	reg [5:0] regfile_waddr_lsu;
	reg wb_contention;
	reg wb_contention_lsu;
	wire alu_ready;
	wire mult_ready;
	wire fpu_ready;
	wire fpu_valid;
	wire apu_valid;
	wire [5:0] apu_waddr;
	wire [31:0] apu_result;
	wire apu_stall;
	wire apu_active;
	wire apu_singlecycle;
	wire apu_multicycle;
	wire apu_req;
	wire apu_ready;
	wire apu_gnt;
	always @(*) begin
		if (_sv2v_0)
			;
		regfile_alu_wdata_fw_o = 1'sb0;
		regfile_alu_waddr_fw_o = 1'sb0;
		regfile_alu_we_fw_o = 1'sb0;
		wb_contention = 1'b0;
		if (apu_valid & (apu_singlecycle | apu_multicycle)) begin
			regfile_alu_we_fw_o = 1'b1;
			regfile_alu_waddr_fw_o = apu_waddr;
			regfile_alu_wdata_fw_o = apu_result;
			if (regfile_alu_we_i & ~apu_en_i)
				wb_contention = 1'b1;
		end
		else begin
			regfile_alu_we_fw_o = regfile_alu_we_i & ~apu_en_i;
			regfile_alu_waddr_fw_o = regfile_alu_waddr_i;
			if (alu_en_i)
				regfile_alu_wdata_fw_o = alu_result;
			if (mult_en_i)
				regfile_alu_wdata_fw_o = mult_result;
			if (csr_access_i)
				regfile_alu_wdata_fw_o = csr_rdata_i;
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		regfile_we_wb_o = 1'b0;
		regfile_waddr_wb_o = regfile_waddr_lsu;
		regfile_wdata_wb_o = lsu_rdata_i;
		wb_contention_lsu = 1'b0;
		if (regfile_we_lsu) begin
			regfile_we_wb_o = 1'b1;
			if (apu_valid & (!apu_singlecycle & !apu_multicycle))
				wb_contention_lsu = 1'b1;
		end
		else if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
			regfile_we_wb_o = 1'b1;
			regfile_waddr_wb_o = apu_waddr;
			regfile_wdata_wb_o = apu_result;
		end
	end
	assign branch_decision_o = alu_cmp_result;
	assign jump_target_o = alu_operand_c_i;
	riscv_alu #(
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.FPU(FPU)
	) alu_i(
		.clk(clk),
		.rst_n(rst_n),
		.enable_i(alu_en_i),
		.operator_i(alu_operator_i),
		.operand_a_i(alu_operand_a_i),
		.operand_b_i(alu_operand_b_i),
		.operand_c_i(alu_operand_c_i),
		.vector_mode_i(alu_vec_mode_i),
		.bmask_a_i(bmask_a_i),
		.bmask_b_i(bmask_b_i),
		.imm_vec_ext_i(imm_vec_ext_i),
		.is_clpx_i(alu_is_clpx_i),
		.clpx_shift_i(alu_clpx_shift_i),
		.is_subrot_i(alu_is_subrot_i),
		.result_o(alu_result),
		.comparison_result_o(alu_cmp_result),
		.ready_o(alu_ready),
		.ex_ready_i(ex_ready_o)
	);
	riscv_mult #(.SHARED_DSP_MULT(SHARED_DSP_MULT)) mult_i(
		.clk(clk),
		.rst_n(rst_n),
		.enable_i(mult_en_i),
		.operator_i(mult_operator_i),
		.short_subword_i(mult_sel_subword_i),
		.short_signed_i(mult_signed_mode_i),
		.op_a_i(mult_operand_a_i),
		.op_b_i(mult_operand_b_i),
		.op_c_i(mult_operand_c_i),
		.imm_i(mult_imm_i),
		.dot_op_a_i(mult_dot_op_a_i),
		.dot_op_b_i(mult_dot_op_b_i),
		.dot_op_c_i(mult_dot_op_c_i),
		.dot_signed_i(mult_dot_signed_i),
		.is_clpx_i(mult_is_clpx_i),
		.clpx_shift_i(mult_clpx_shift_i),
		.clpx_img_i(mult_clpx_img_i),
		.result_o(mult_result),
		.multicycle_o(mult_multicycle_o),
		.ready_o(mult_ready),
		.ex_ready_i(ex_ready_o)
	);
	localparam [31:0] fpnew_pkg_NUM_FP_FORMATS = 5;
	localparam [31:0] fpnew_pkg_FP_FORMAT_BITS = 3;
	localparam [31:0] fpnew_pkg_NUM_INT_FORMATS = 4;
	localparam [31:0] fpnew_pkg_NUM_OPGROUPS = 4;
	localparam [31:0] fpnew_pkg_INT_FORMAT_BITS = 2;
	localparam [31:0] fpnew_pkg_OP_BITS = 4;
	localparam [0:0] riscv_defines_C_RVD = 1'b0;
	localparam [0:0] riscv_defines_C_RVF = 1'b1;
	localparam [0:0] riscv_defines_C_XF16 = 1'b0;
	localparam [0:0] riscv_defines_C_XF16ALT = 1'b0;
	localparam [0:0] riscv_defines_C_XF8 = 1'b0;
	localparam riscv_defines_C_FLEN = (riscv_defines_C_RVD ? 64 : (riscv_defines_C_RVF ? 32 : (riscv_defines_C_XF16 ? 16 : (riscv_defines_C_XF16ALT ? 16 : (riscv_defines_C_XF8 ? 8 : 0)))));
	localparam riscv_defines_C_FPNEW_FMTBITS = fpnew_pkg_FP_FORMAT_BITS;
	localparam riscv_defines_C_FPNEW_IFMTBITS = fpnew_pkg_INT_FORMAT_BITS;
	localparam riscv_defines_C_FPNEW_OPBITS = fpnew_pkg_OP_BITS;
	localparam [31:0] riscv_defines_C_LAT_CONV = 'd5;
	localparam [31:0] riscv_defines_C_LAT_DIVSQRT = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP16 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP16ALT = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP32 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP64 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_FP8 = 'd5;
	localparam [31:0] riscv_defines_C_LAT_NONCOMP = 'd5;
	localparam riscv_defines_C_RM = 3;
	localparam [0:0] riscv_defines_C_XFVEC = 1'b0;
	function automatic [3:0] sv2v_cast_A53F3;
		input reg [3:0] inp;
		sv2v_cast_A53F3 = inp;
	endfunction
	function automatic [2:0] sv2v_cast_0BC43;
		input reg [2:0] inp;
		sv2v_cast_0BC43 = inp;
	endfunction
	function automatic [1:0] sv2v_cast_87CC5;
		input reg [1:0] inp;
		sv2v_cast_87CC5 = inp;
	endfunction
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	function automatic [4:0] sv2v_cast_5;
		input reg [4:0] inp;
		sv2v_cast_5 = inp;
	endfunction
	function automatic [((32'd4 * 32'd5) * 32) - 1:0] sv2v_cast_CDC93;
		input reg [((32'd4 * 32'd5) * 32) - 1:0] inp;
		sv2v_cast_CDC93 = inp;
	endfunction
	function automatic [((32'd4 * 32'd5) * 2) - 1:0] sv2v_cast_15FEF;
		input reg [((32'd4 * 32'd5) * 2) - 1:0] inp;
		sv2v_cast_15FEF = inp;
	endfunction
	generate
		if (FPU == 1) begin : genblk1
			riscv_apu_disp apu_disp_i(
				.clk_i(clk),
				.rst_ni(rst_n),
				.enable_i(apu_en_i),
				.apu_lat_i(apu_lat_i),
				.apu_waddr_i(apu_waddr_i),
				.apu_waddr_o(apu_waddr),
				.apu_multicycle_o(apu_multicycle),
				.apu_singlecycle_o(apu_singlecycle),
				.active_o(apu_active),
				.stall_o(apu_stall),
				.read_regs_i(apu_read_regs_i),
				.read_regs_valid_i(apu_read_regs_valid_i),
				.read_dep_o(apu_read_dep_o),
				.write_regs_i(apu_write_regs_i),
				.write_regs_valid_i(apu_write_regs_valid_i),
				.write_dep_o(apu_write_dep_o),
				.perf_type_o(apu_perf_type_o),
				.perf_cont_o(apu_perf_cont_o),
				.apu_master_req_o(apu_req),
				.apu_master_ready_o(apu_ready),
				.apu_master_gnt_i(apu_gnt),
				.apu_master_valid_i(apu_valid)
			);
			assign apu_perf_wb_o = wb_contention | wb_contention_lsu;
			assign apu_ready_wb_o = ~((apu_active | apu_en_i) | apu_stall) | apu_valid;
			if (SHARED_FP) begin : genblk1
				assign apu_master_req_o = apu_req;
				assign apu_master_ready_o = apu_ready;
				assign apu_gnt = apu_master_gnt_i;
				assign apu_valid = apu_master_valid_i;
				assign apu_master_operands_o = apu_operands_i;
				assign apu_master_op_o = apu_op_i;
				assign apu_result = apu_master_result_i;
				assign fpu_fflags_we_o = apu_valid;
				assign fpu_ready = 1'b1;
			end
			else begin : genblk1
				wire [3:0] fpu_op;
				wire fpu_op_mod;
				wire fpu_vec_op;
				wire [2:0] fpu_dst_fmt;
				wire [2:0] fpu_src_fmt;
				wire [1:0] fpu_int_fmt;
				wire [2:0] fp_rnd_mode;
				assign {fpu_vec_op, fpu_op_mod, fpu_op} = apu_op_i;
				assign {fpu_int_fmt, fpu_src_fmt, fpu_dst_fmt, fp_rnd_mode} = apu_flags_i;
				localparam C_DIV = (FP_DIVSQRT ? 2'd2 : 2'd0);
				wire FPU_ready_int;
				localparam [42:0] FPU_FEATURES = {sv2v_cast_32(riscv_defines_C_FLEN), riscv_defines_C_XFVEC, 1'b0, sv2v_cast_5({riscv_defines_C_RVF, riscv_defines_C_RVD, riscv_defines_C_XF16, riscv_defines_C_XF8, riscv_defines_C_XF16ALT}), 4'h2};
				localparam [(((fpnew_pkg_NUM_OPGROUPS * fpnew_pkg_NUM_FP_FORMATS) * 32) + ((fpnew_pkg_NUM_OPGROUPS * fpnew_pkg_NUM_FP_FORMATS) * 2)) + 1:0] FPU_IMPLEMENTATION = {sv2v_cast_CDC93({riscv_defines_C_LAT_FP32, riscv_defines_C_LAT_FP64, riscv_defines_C_LAT_FP16, riscv_defines_C_LAT_FP8, riscv_defines_C_LAT_FP16ALT, {fpnew_pkg_NUM_FP_FORMATS {riscv_defines_C_LAT_DIVSQRT}}, {fpnew_pkg_NUM_FP_FORMATS {riscv_defines_C_LAT_NONCOMP}}, {fpnew_pkg_NUM_FP_FORMATS {riscv_defines_C_LAT_CONV}}}), sv2v_cast_15FEF({{fpnew_pkg_NUM_FP_FORMATS {2'd2}}, {fpnew_pkg_NUM_FP_FORMATS {C_DIV}}, {fpnew_pkg_NUM_FP_FORMATS {2'd1}}, {fpnew_pkg_NUM_FP_FORMATS {2'd2}}}), 2'd3};
				fpnew_top_FF541 #(
					.Features(FPU_FEATURES),
					.Implementation(FPU_IMPLEMENTATION)
				) i_fpnew_bulk(
					.clk_i(clk),
					.rst_ni(rst_n),
					.operands_i(apu_operands_i),
					.rnd_mode_i(fp_rnd_mode),
					.op_i(sv2v_cast_A53F3(fpu_op)),
					.op_mod_i(fpu_op_mod),
					.src_fmt_i(sv2v_cast_0BC43(fpu_src_fmt)),
					.dst_fmt_i(sv2v_cast_0BC43(fpu_dst_fmt)),
					.int_fmt_i(sv2v_cast_87CC5(fpu_int_fmt)),
					.vectorial_op_i(fpu_vec_op),
					.tag_i(1'b0),
					.in_valid_i(apu_req),
					.in_ready_o(FPU_ready_int),
					.flush_i(1'b0),
					.result_o(apu_result),
					.status_o(fpu_fflags_o),
					.tag_o(),
					.out_valid_o(apu_valid),
					.out_ready_i(1'b1),
					.busy_o()
				);
				assign fpu_fflags_we_o = apu_valid;
				assign apu_master_req_o = 1'sb0;
				assign apu_master_ready_o = 1'b1;
				assign apu_master_operands_o[0+:32] = 1'sb0;
				assign apu_master_operands_o[32+:32] = 1'sb0;
				assign apu_master_operands_o[64+:32] = 1'sb0;
				assign apu_master_op_o = 1'sb0;
				assign apu_gnt = 1'b1;
				assign fpu_ready = (FPU_ready_int & apu_req) | ~apu_req;
			end
		end
		else begin : genblk1
			assign apu_master_req_o = 1'sb0;
			assign apu_master_ready_o = 1'b1;
			assign apu_master_operands_o[0+:32] = 1'sb0;
			assign apu_master_operands_o[32+:32] = 1'sb0;
			assign apu_master_operands_o[64+:32] = 1'sb0;
			assign apu_master_op_o = 1'sb0;
			assign apu_valid = 1'b0;
			assign apu_waddr = 6'b000000;
			assign apu_stall = 1'b0;
			assign apu_active = 1'b0;
			assign apu_ready_wb_o = 1'b1;
			assign apu_perf_wb_o = 1'b0;
			assign apu_perf_cont_o = 1'b0;
			assign apu_perf_type_o = 1'b0;
			assign apu_singlecycle = 1'b0;
			assign apu_multicycle = 1'b0;
			assign apu_read_dep_o = 1'b0;
			assign apu_write_dep_o = 1'b0;
			assign fpu_fflags_we_o = 1'b0;
			assign fpu_fflags_o = 1'sb0;
			assign fpu_ready = 1'b1;
		end
	endgenerate
	assign apu_busy_o = apu_active;
	always @(posedge clk or negedge rst_n) begin : EX_WB_Pipeline_Register
		if (~rst_n) begin
			regfile_waddr_lsu <= 1'sb0;
			regfile_we_lsu <= 1'b0;
		end
		else if (ex_valid_o) begin
			regfile_we_lsu <= regfile_we_i & ~lsu_err_i;
			if (regfile_we_i & ~lsu_err_i)
				regfile_waddr_lsu <= regfile_waddr_i;
		end
		else if (wb_ready_i)
			regfile_we_lsu <= 1'b0;
	end
	assign ex_ready_o = ((((((~apu_stall & alu_ready) & mult_ready) & lsu_ready_ex_i) & wb_ready_i) & ~wb_contention) & fpu_ready) | branch_in_ex_i;
	assign ex_valid_o = ((((apu_valid | alu_en_i) | mult_en_i) | csr_access_i) | lsu_en_i) & (((alu_ready & mult_ready) & lsu_ready_ex_i) & wb_ready_i);
	initial _sv2v_0 = 0;
endmodule
module riscv_alu_div (
	Clk_CI,
	Rst_RBI,
	OpA_DI,
	OpB_DI,
	OpBShift_DI,
	OpBIsZero_SI,
	OpBSign_SI,
	OpCode_SI,
	InVld_SI,
	OutRdy_SI,
	OutVld_SO,
	Res_DO
);
	reg _sv2v_0;
	parameter C_WIDTH = 32;
	parameter C_LOG_WIDTH = 6;
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire [C_WIDTH - 1:0] OpA_DI;
	input wire [C_WIDTH - 1:0] OpB_DI;
	input wire [C_LOG_WIDTH - 1:0] OpBShift_DI;
	input wire OpBIsZero_SI;
	input wire OpBSign_SI;
	input wire [1:0] OpCode_SI;
	input wire InVld_SI;
	input wire OutRdy_SI;
	output reg OutVld_SO;
	output wire [C_WIDTH - 1:0] Res_DO;
	reg [C_WIDTH - 1:0] ResReg_DP;
	wire [C_WIDTH - 1:0] ResReg_DN;
	wire [C_WIDTH - 1:0] ResReg_DP_rev;
	reg [C_WIDTH - 1:0] AReg_DP;
	wire [C_WIDTH - 1:0] AReg_DN;
	reg [C_WIDTH - 1:0] BReg_DP;
	wire [C_WIDTH - 1:0] BReg_DN;
	wire RemSel_SN;
	reg RemSel_SP;
	wire CompInv_SN;
	reg CompInv_SP;
	wire ResInv_SN;
	reg ResInv_SP;
	wire [C_WIDTH - 1:0] AddMux_D;
	wire [C_WIDTH - 1:0] AddOut_D;
	wire [C_WIDTH - 1:0] AddTmp_D;
	wire [C_WIDTH - 1:0] BMux_D;
	wire [C_WIDTH - 1:0] OutMux_D;
	reg [C_LOG_WIDTH - 1:0] Cnt_DP;
	wire [C_LOG_WIDTH - 1:0] Cnt_DN;
	wire CntZero_S;
	reg ARegEn_S;
	reg BRegEn_S;
	reg ResRegEn_S;
	wire ABComp_S;
	wire PmSel_S;
	reg LoadEn_S;
	reg [1:0] State_SN;
	reg [1:0] State_SP;
	assign PmSel_S = LoadEn_S & ~(OpCode_SI[0] & (OpA_DI[C_WIDTH - 1] ^ OpBSign_SI));
	assign AddMux_D = (LoadEn_S ? OpA_DI : BReg_DP);
	assign BMux_D = (LoadEn_S ? OpB_DI : {CompInv_SP, BReg_DP[C_WIDTH - 1:1]});
	genvar _gv_index_1;
	generate
		for (_gv_index_1 = 0; _gv_index_1 < C_WIDTH; _gv_index_1 = _gv_index_1 + 1) begin : bit_swapping
			localparam index = _gv_index_1;
			assign ResReg_DP_rev[index] = ResReg_DP[(C_WIDTH - 1) - index];
		end
	endgenerate
	assign OutMux_D = (RemSel_SP ? AReg_DP : ResReg_DP_rev);
	assign Res_DO = (ResInv_SP ? -$signed(OutMux_D) : OutMux_D);
	assign ABComp_S = ((AReg_DP == BReg_DP) | ((AReg_DP > BReg_DP) ^ CompInv_SP)) & (|AReg_DP | OpBIsZero_SI);
	assign AddTmp_D = (LoadEn_S ? 0 : AReg_DP);
	assign AddOut_D = (PmSel_S ? AddTmp_D + AddMux_D : AddTmp_D - $signed(AddMux_D));
	assign Cnt_DN = (LoadEn_S ? OpBShift_DI : (~CntZero_S ? Cnt_DP - 1 : Cnt_DP));
	assign CntZero_S = ~(|Cnt_DP);
	always @(*) begin : p_fsm
		if (_sv2v_0)
			;
		State_SN = State_SP;
		OutVld_SO = 1'b0;
		LoadEn_S = 1'b0;
		ARegEn_S = 1'b0;
		BRegEn_S = 1'b0;
		ResRegEn_S = 1'b0;
		case (State_SP)
			2'd0: begin
				OutVld_SO = 1'b1;
				if (InVld_SI) begin
					OutVld_SO = 1'b0;
					ARegEn_S = 1'b1;
					BRegEn_S = 1'b1;
					LoadEn_S = 1'b1;
					State_SN = 2'd1;
				end
			end
			2'd1: begin
				ARegEn_S = ABComp_S;
				BRegEn_S = 1'b1;
				ResRegEn_S = 1'b1;
				if (CntZero_S)
					State_SN = 2'd2;
			end
			2'd2: begin
				OutVld_SO = 1'b1;
				if (OutRdy_SI)
					State_SN = 2'd0;
			end
			default:
				;
		endcase
	end
	assign RemSel_SN = (LoadEn_S ? OpCode_SI[1] : RemSel_SP);
	assign CompInv_SN = (LoadEn_S ? OpBSign_SI : CompInv_SP);
	assign ResInv_SN = (LoadEn_S ? ((~OpBIsZero_SI | OpCode_SI[1]) & OpCode_SI[0]) & (OpA_DI[C_WIDTH - 1] ^ OpBSign_SI) : ResInv_SP);
	assign AReg_DN = (ARegEn_S ? AddOut_D : AReg_DP);
	assign BReg_DN = (BRegEn_S ? BMux_D : BReg_DP);
	assign ResReg_DN = (LoadEn_S ? {C_WIDTH {1'sb0}} : (ResRegEn_S ? {ABComp_S, ResReg_DP[C_WIDTH - 1:1]} : ResReg_DP));
	always @(posedge Clk_CI or negedge Rst_RBI) begin : p_regs
		if (~Rst_RBI) begin
			State_SP <= 2'd0;
			AReg_DP <= 1'sb0;
			BReg_DP <= 1'sb0;
			ResReg_DP <= 1'sb0;
			Cnt_DP <= 1'sb0;
			RemSel_SP <= 1'b0;
			CompInv_SP <= 1'b0;
			ResInv_SP <= 1'b0;
		end
		else begin
			State_SP <= State_SN;
			AReg_DP <= AReg_DN;
			BReg_DP <= BReg_DN;
			ResReg_DP <= ResReg_DN;
			Cnt_DP <= Cnt_DN;
			RemSel_SP <= RemSel_SN;
			CompInv_SP <= CompInv_SN;
			ResInv_SP <= ResInv_SN;
		end
	end
	initial _sv2v_0 = 0;
endmodule
module riscv_alu (
	clk,
	rst_n,
	enable_i,
	operator_i,
	operand_a_i,
	operand_b_i,
	operand_c_i,
	vector_mode_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	is_clpx_i,
	is_subrot_i,
	clpx_shift_i,
	result_o,
	comparison_result_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	parameter SHARED_INT_DIV = 0;
	parameter FPU = 0;
	input wire clk;
	input wire rst_n;
	input wire enable_i;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	input wire [6:0] operator_i;
	input wire [31:0] operand_a_i;
	input wire [31:0] operand_b_i;
	input wire [31:0] operand_c_i;
	input wire [1:0] vector_mode_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	input wire is_clpx_i;
	input wire is_subrot_i;
	input wire [1:0] clpx_shift_i;
	output reg [31:0] result_o;
	output wire comparison_result_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [31:0] operand_a_rev;
	wire [31:0] operand_a_neg;
	wire [31:0] operand_a_neg_rev;
	assign operand_a_neg = ~operand_a_i;
	genvar _gv_k_2;
	generate
		for (_gv_k_2 = 0; _gv_k_2 < 32; _gv_k_2 = _gv_k_2 + 1) begin : genblk1
			localparam k = _gv_k_2;
			assign operand_a_rev[k] = operand_a_i[31 - k];
		end
	endgenerate
	genvar _gv_m_1;
	generate
		for (_gv_m_1 = 0; _gv_m_1 < 32; _gv_m_1 = _gv_m_1 + 1) begin : genblk2
			localparam m = _gv_m_1;
			assign operand_a_neg_rev[m] = operand_a_neg[31 - m];
		end
	endgenerate
	wire [31:0] operand_b_neg;
	assign operand_b_neg = ~operand_b_i;
	wire [5:0] div_shift;
	wire div_valid;
	wire [31:0] bmask;
	wire adder_op_b_negate;
	wire [31:0] adder_op_a;
	wire [31:0] adder_op_b;
	reg [35:0] adder_in_a;
	reg [35:0] adder_in_b;
	wire [31:0] adder_result;
	wire [36:0] adder_result_expanded;
	localparam riscv_defines_ALU_SUB = 7'b0011001;
	localparam riscv_defines_ALU_SUBR = 7'b0011101;
	localparam riscv_defines_ALU_SUBU = 7'b0011011;
	localparam riscv_defines_ALU_SUBUR = 7'b0011111;
	assign adder_op_b_negate = ((((operator_i == riscv_defines_ALU_SUB) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_SUBU)) || (operator_i == riscv_defines_ALU_SUBUR)) || is_subrot_i;
	localparam riscv_defines_ALU_ABS = 7'b0010100;
	assign adder_op_a = (operator_i == riscv_defines_ALU_ABS ? operand_a_neg : (is_subrot_i ? {operand_b_i[15:0], operand_a_i[31:16]} : operand_a_i));
	assign adder_op_b = (adder_op_b_negate ? (is_subrot_i ? ~{operand_a_i[15:0], operand_b_i[31:16]} : operand_b_neg) : operand_b_i);
	localparam riscv_defines_ALU_CLIP = 7'b0010110;
	localparam riscv_defines_VEC_MODE16 = 2'b10;
	localparam riscv_defines_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		adder_in_a[0] = 1'b1;
		adder_in_a[8:1] = adder_op_a[7:0];
		adder_in_a[9] = 1'b1;
		adder_in_a[17:10] = adder_op_a[15:8];
		adder_in_a[18] = 1'b1;
		adder_in_a[26:19] = adder_op_a[23:16];
		adder_in_a[27] = 1'b1;
		adder_in_a[35:28] = adder_op_a[31:24];
		adder_in_b[0] = 1'b0;
		adder_in_b[8:1] = adder_op_b[7:0];
		adder_in_b[9] = 1'b0;
		adder_in_b[17:10] = adder_op_b[15:8];
		adder_in_b[18] = 1'b0;
		adder_in_b[26:19] = adder_op_b[23:16];
		adder_in_b[27] = 1'b0;
		adder_in_b[35:28] = adder_op_b[31:24];
		if (adder_op_b_negate || ((operator_i == riscv_defines_ALU_ABS) || (operator_i == riscv_defines_ALU_CLIP))) begin
			adder_in_b[0] = 1'b1;
			case (vector_mode_i)
				riscv_defines_VEC_MODE16: adder_in_b[18] = 1'b1;
				riscv_defines_VEC_MODE8: begin
					adder_in_b[9] = 1'b1;
					adder_in_b[18] = 1'b1;
					adder_in_b[27] = 1'b1;
				end
			endcase
		end
		else
			case (vector_mode_i)
				riscv_defines_VEC_MODE16: adder_in_a[18] = 1'b0;
				riscv_defines_VEC_MODE8: begin
					adder_in_a[9] = 1'b0;
					adder_in_a[18] = 1'b0;
					adder_in_a[27] = 1'b0;
				end
			endcase
	end
	assign adder_result_expanded = $signed(adder_in_a) + $signed(adder_in_b);
	assign adder_result = {adder_result_expanded[35:28], adder_result_expanded[26:19], adder_result_expanded[17:10], adder_result_expanded[8:1]};
	wire [31:0] adder_round_value;
	wire [31:0] adder_round_result;
	localparam riscv_defines_ALU_ADDR = 7'b0011100;
	localparam riscv_defines_ALU_ADDUR = 7'b0011110;
	assign adder_round_value = ((((operator_i == riscv_defines_ALU_ADDR) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_ADDUR)) || (operator_i == riscv_defines_ALU_SUBUR) ? {1'b0, bmask[31:1]} : {32 {1'sb0}});
	assign adder_round_result = adder_result + adder_round_value;
	wire shift_left;
	wire shift_use_round;
	wire shift_arithmetic;
	reg [31:0] shift_amt_left;
	wire [31:0] shift_amt;
	wire [31:0] shift_amt_int;
	wire [31:0] shift_amt_norm;
	wire [31:0] shift_op_a;
	wire [31:0] shift_result;
	reg [31:0] shift_right_result;
	wire [31:0] shift_left_result;
	wire [15:0] clpx_shift_ex;
	assign shift_amt = (div_valid ? div_shift : operand_b_i);
	always @(*) begin
		if (_sv2v_0)
			;
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				shift_amt_left[15:0] = shift_amt[31:16];
				shift_amt_left[31:16] = shift_amt[15:0];
			end
			riscv_defines_VEC_MODE8: begin
				shift_amt_left[7:0] = shift_amt[31:24];
				shift_amt_left[15:8] = shift_amt[23:16];
				shift_amt_left[23:16] = shift_amt[15:8];
				shift_amt_left[31:24] = shift_amt[7:0];
			end
			default: shift_amt_left[31:0] = shift_amt[31:0];
		endcase
	end
	localparam riscv_defines_ALU_BINS = 7'b0101010;
	localparam riscv_defines_ALU_BREV = 7'b1001001;
	localparam riscv_defines_ALU_CLB = 7'b0110101;
	localparam riscv_defines_ALU_DIV = 7'b0110001;
	localparam riscv_defines_ALU_DIVU = 7'b0110000;
	localparam riscv_defines_ALU_FL1 = 7'b0110111;
	localparam riscv_defines_ALU_REM = 7'b0110011;
	localparam riscv_defines_ALU_REMU = 7'b0110010;
	localparam riscv_defines_ALU_SLL = 7'b0100111;
	assign shift_left = ((((((((operator_i == riscv_defines_ALU_SLL) || (operator_i == riscv_defines_ALU_BINS)) || (operator_i == riscv_defines_ALU_FL1)) || (operator_i == riscv_defines_ALU_CLB)) || (operator_i == riscv_defines_ALU_DIV)) || (operator_i == riscv_defines_ALU_DIVU)) || (operator_i == riscv_defines_ALU_REM)) || (operator_i == riscv_defines_ALU_REMU)) || (operator_i == riscv_defines_ALU_BREV);
	localparam riscv_defines_ALU_ADD = 7'b0011000;
	localparam riscv_defines_ALU_ADDU = 7'b0011010;
	assign shift_use_round = (((((((operator_i == riscv_defines_ALU_ADD) || (operator_i == riscv_defines_ALU_SUB)) || (operator_i == riscv_defines_ALU_ADDR)) || (operator_i == riscv_defines_ALU_SUBR)) || (operator_i == riscv_defines_ALU_ADDU)) || (operator_i == riscv_defines_ALU_SUBU)) || (operator_i == riscv_defines_ALU_ADDUR)) || (operator_i == riscv_defines_ALU_SUBUR);
	localparam riscv_defines_ALU_BEXT = 7'b0101000;
	localparam riscv_defines_ALU_SRA = 7'b0100100;
	assign shift_arithmetic = (((((operator_i == riscv_defines_ALU_SRA) || (operator_i == riscv_defines_ALU_BEXT)) || (operator_i == riscv_defines_ALU_ADD)) || (operator_i == riscv_defines_ALU_SUB)) || (operator_i == riscv_defines_ALU_ADDR)) || (operator_i == riscv_defines_ALU_SUBR);
	assign shift_op_a = (shift_left ? operand_a_rev : (shift_use_round ? adder_round_result : operand_a_i));
	assign shift_amt_int = (shift_use_round ? shift_amt_norm : (shift_left ? shift_amt_left : shift_amt));
	assign shift_amt_norm = (is_clpx_i ? {clpx_shift_ex, clpx_shift_ex} : {4 {3'b000, bmask_b_i}});
	assign clpx_shift_ex = $unsigned(clpx_shift_i);
	wire [63:0] shift_op_a_32;
	localparam riscv_defines_ALU_ROR = 7'b0100110;
	assign shift_op_a_32 = (operator_i == riscv_defines_ALU_ROR ? {shift_op_a, shift_op_a} : $signed({{32 {shift_arithmetic & shift_op_a[31]}}, shift_op_a}));
	always @(*) begin
		if (_sv2v_0)
			;
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				shift_right_result[31:16] = $signed({shift_arithmetic & shift_op_a[31], shift_op_a[31:16]}) >>> shift_amt_int[19:16];
				shift_right_result[15:0] = $signed({shift_arithmetic & shift_op_a[15], shift_op_a[15:0]}) >>> shift_amt_int[3:0];
			end
			riscv_defines_VEC_MODE8: begin
				shift_right_result[31:24] = $signed({shift_arithmetic & shift_op_a[31], shift_op_a[31:24]}) >>> shift_amt_int[26:24];
				shift_right_result[23:16] = $signed({shift_arithmetic & shift_op_a[23], shift_op_a[23:16]}) >>> shift_amt_int[18:16];
				shift_right_result[15:8] = $signed({shift_arithmetic & shift_op_a[15], shift_op_a[15:8]}) >>> shift_amt_int[10:8];
				shift_right_result[7:0] = $signed({shift_arithmetic & shift_op_a[7], shift_op_a[7:0]}) >>> shift_amt_int[2:0];
			end
			default: shift_right_result = shift_op_a_32 >> shift_amt_int[4:0];
		endcase
	end
	genvar _gv_j_2;
	generate
		for (_gv_j_2 = 0; _gv_j_2 < 32; _gv_j_2 = _gv_j_2 + 1) begin : genblk3
			localparam j = _gv_j_2;
			assign shift_left_result[j] = shift_right_result[31 - j];
		end
	endgenerate
	assign shift_result = (shift_left ? shift_left_result : shift_right_result);
	reg [3:0] is_equal;
	reg [3:0] is_greater;
	wire [3:0] f_is_greater;
	reg [3:0] cmp_signed;
	wire [3:0] is_equal_vec;
	wire [3:0] is_greater_vec;
	reg [31:0] operand_b_eq;
	wire is_equal_clip;
	localparam riscv_defines_ALU_CLIPU = 7'b0010111;
	always @(*) begin
		if (_sv2v_0)
			;
		operand_b_eq = operand_b_neg;
		if (operator_i == riscv_defines_ALU_CLIPU)
			operand_b_eq = 1'sb0;
		else
			operand_b_eq = operand_b_neg;
	end
	assign is_equal_clip = operand_a_i == operand_b_eq;
	localparam riscv_defines_ALU_FLE = 7'b1000101;
	localparam riscv_defines_ALU_FLT = 7'b1000100;
	localparam riscv_defines_ALU_FMAX = 7'b1000110;
	localparam riscv_defines_ALU_FMIN = 7'b1000111;
	localparam riscv_defines_ALU_GES = 7'b0001010;
	localparam riscv_defines_ALU_GTS = 7'b0001000;
	localparam riscv_defines_ALU_LES = 7'b0000100;
	localparam riscv_defines_ALU_LTS = 7'b0000000;
	localparam riscv_defines_ALU_MAX = 7'b0010010;
	localparam riscv_defines_ALU_MIN = 7'b0010000;
	localparam riscv_defines_ALU_SLETS = 7'b0000110;
	localparam riscv_defines_ALU_SLTS = 7'b0000010;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_signed = 4'b0000;
		case (operator_i)
			riscv_defines_ALU_GTS, riscv_defines_ALU_GES, riscv_defines_ALU_LTS, riscv_defines_ALU_LES, riscv_defines_ALU_SLTS, riscv_defines_ALU_SLETS, riscv_defines_ALU_MIN, riscv_defines_ALU_MAX, riscv_defines_ALU_ABS, riscv_defines_ALU_CLIP, riscv_defines_ALU_CLIPU, riscv_defines_ALU_FLE, riscv_defines_ALU_FLT, riscv_defines_ALU_FMAX, riscv_defines_ALU_FMIN:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: cmp_signed[3:0] = 4'b1111;
					riscv_defines_VEC_MODE16: cmp_signed[3:0] = 4'b1010;
					default: cmp_signed[3:0] = 4'b1000;
				endcase
			default:
				;
		endcase
	end
	genvar _gv_i_5;
	generate
		for (_gv_i_5 = 0; _gv_i_5 < 4; _gv_i_5 = _gv_i_5 + 1) begin : genblk4
			localparam i = _gv_i_5;
			assign is_equal_vec[i] = operand_a_i[(8 * i) + 7:8 * i] == operand_b_i[(8 * i) + 7:i * 8];
			assign is_greater_vec[i] = $signed({operand_a_i[(8 * i) + 7] & cmp_signed[i], operand_a_i[(8 * i) + 7:8 * i]}) > $signed({operand_b_i[(8 * i) + 7] & cmp_signed[i], operand_b_i[(8 * i) + 7:i * 8]});
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		is_equal[3:0] = {4 {((is_equal_vec[3] & is_equal_vec[2]) & is_equal_vec[1]) & is_equal_vec[0]}};
		is_greater[3:0] = {4 {is_greater_vec[3] | (is_equal_vec[3] & (is_greater_vec[2] | (is_equal_vec[2] & (is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])))))}};
		case (vector_mode_i)
			riscv_defines_VEC_MODE16: begin
				is_equal[1:0] = {2 {is_equal_vec[0] & is_equal_vec[1]}};
				is_equal[3:2] = {2 {is_equal_vec[2] & is_equal_vec[3]}};
				is_greater[1:0] = {2 {is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])}};
				is_greater[3:2] = {2 {is_greater_vec[3] | (is_equal_vec[3] & is_greater_vec[2])}};
			end
			riscv_defines_VEC_MODE8: begin
				is_equal[3:0] = is_equal_vec[3:0];
				is_greater[3:0] = is_greater_vec[3:0];
			end
			default:
				;
		endcase
	end
	assign f_is_greater[3:0] = {4 {is_greater[3] ^ ((operand_a_i[31] & operand_b_i[31]) & !is_equal[3])}};
	reg [3:0] cmp_result;
	wire f_is_qnan;
	wire f_is_snan;
	reg [3:0] f_is_nan;
	localparam riscv_defines_ALU_EQ = 7'b0001100;
	localparam riscv_defines_ALU_FEQ = 7'b1000011;
	localparam riscv_defines_ALU_GEU = 7'b0001011;
	localparam riscv_defines_ALU_GTU = 7'b0001001;
	localparam riscv_defines_ALU_LEU = 7'b0000101;
	localparam riscv_defines_ALU_LTU = 7'b0000001;
	localparam riscv_defines_ALU_NE = 7'b0001101;
	localparam riscv_defines_ALU_SLETU = 7'b0000111;
	localparam riscv_defines_ALU_SLTU = 7'b0000011;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_result = is_equal;
		f_is_nan = {4 {f_is_qnan | f_is_snan}};
		case (operator_i)
			riscv_defines_ALU_EQ: cmp_result = is_equal;
			riscv_defines_ALU_NE: cmp_result = ~is_equal;
			riscv_defines_ALU_GTS, riscv_defines_ALU_GTU: cmp_result = is_greater;
			riscv_defines_ALU_GES, riscv_defines_ALU_GEU: cmp_result = is_greater | is_equal;
			riscv_defines_ALU_LTS, riscv_defines_ALU_SLTS, riscv_defines_ALU_LTU, riscv_defines_ALU_SLTU: cmp_result = ~(is_greater | is_equal);
			riscv_defines_ALU_SLETS, riscv_defines_ALU_SLETU, riscv_defines_ALU_LES, riscv_defines_ALU_LEU: cmp_result = ~is_greater;
			riscv_defines_ALU_FEQ: cmp_result = is_equal & ~f_is_nan;
			riscv_defines_ALU_FLE: cmp_result = ~f_is_greater & ~f_is_nan;
			riscv_defines_ALU_FLT: cmp_result = ~(f_is_greater | is_equal) & ~f_is_nan;
			default:
				;
		endcase
	end
	assign comparison_result_o = cmp_result[3];
	wire [31:0] result_minmax;
	wire [31:0] fp_canonical_nan;
	wire [3:0] sel_minmax;
	wire do_min;
	wire minmax_is_fp_special;
	wire [31:0] minmax_b;
	assign minmax_b = (operator_i == riscv_defines_ALU_ABS ? adder_result : operand_b_i);
	localparam riscv_defines_ALU_MINU = 7'b0010001;
	assign do_min = ((((operator_i == riscv_defines_ALU_MIN) || (operator_i == riscv_defines_ALU_MINU)) || (operator_i == riscv_defines_ALU_CLIP)) || (operator_i == riscv_defines_ALU_CLIPU)) || (operator_i == riscv_defines_ALU_FMIN);
	assign sel_minmax[3:0] = ((operator_i == riscv_defines_ALU_FMIN) || (operator_i == riscv_defines_ALU_FMAX) ? f_is_greater : is_greater) ^ {4 {do_min}};
	assign result_minmax[31:24] = (sel_minmax[3] == 1'b1 ? operand_a_i[31:24] : minmax_b[31:24]);
	assign result_minmax[23:16] = (sel_minmax[2] == 1'b1 ? operand_a_i[23:16] : minmax_b[23:16]);
	assign result_minmax[15:8] = (sel_minmax[1] == 1'b1 ? operand_a_i[15:8] : minmax_b[15:8]);
	assign result_minmax[7:0] = (sel_minmax[0] == 1'b1 ? operand_a_i[7:0] : minmax_b[7:0]);
	wire [31:0] fclass_result;
	generate
		if (FPU == 1) begin : genblk5
			wire [7:0] fclass_exponent;
			wire [22:0] fclass_mantiassa;
			wire fclass_ninf;
			wire fclass_pinf;
			wire fclass_normal;
			wire fclass_subnormal;
			wire fclass_nzero;
			wire fclass_pzero;
			wire fclass_is_negative;
			wire fclass_snan_a;
			wire fclass_qnan_a;
			wire fclass_snan_b;
			wire fclass_qnan_b;
			assign fclass_exponent = operand_a_i[30:23];
			assign fclass_mantiassa = operand_a_i[22:0];
			assign fclass_is_negative = operand_a_i[31];
			assign fclass_ninf = operand_a_i == 32'hff800000;
			assign fclass_pinf = operand_a_i == 32'h7f800000;
			assign fclass_normal = (fclass_exponent != 0) && (fclass_exponent != 255);
			assign fclass_subnormal = (fclass_exponent == 0) && (fclass_mantiassa != 0);
			assign fclass_nzero = operand_a_i == 32'h80000000;
			assign fclass_pzero = operand_a_i == 32'h00000000;
			assign fclass_snan_a = operand_a_i[30:0] == 32'h7fa00000;
			assign fclass_qnan_a = operand_a_i[30:0] == 32'h7fc00000;
			assign fclass_snan_b = operand_b_i[30:0] == 32'h7fa00000;
			assign fclass_qnan_b = operand_b_i[30:0] == 32'h7fc00000;
			assign fclass_result[31:0] = {{22 {1'b0}}, fclass_qnan_a, fclass_snan_a, fclass_pinf, fclass_normal && !fclass_is_negative, fclass_subnormal && !fclass_is_negative, fclass_pzero, fclass_nzero, fclass_subnormal && fclass_is_negative, fclass_normal && fclass_is_negative, fclass_ninf};
			assign f_is_qnan = fclass_qnan_a | fclass_qnan_b;
			assign f_is_snan = fclass_snan_a | fclass_snan_b;
			assign minmax_is_fp_special = ((operator_i == riscv_defines_ALU_FMIN) || (operator_i == riscv_defines_ALU_FMAX)) & (f_is_snan | f_is_qnan);
			assign fp_canonical_nan = 32'h7fc00000;
		end
		else begin : genblk5
			assign minmax_is_fp_special = 1'sb0;
			assign f_is_qnan = 1'sb0;
			assign f_is_snan = 1'sb0;
			assign fclass_result = 1'sb0;
			assign fp_canonical_nan = 1'sb0;
		end
	endgenerate
	reg [31:0] f_sign_inject_result;
	localparam riscv_defines_ALU_FKEEP = 7'b1111111;
	localparam riscv_defines_ALU_FSGNJ = 7'b1000000;
	localparam riscv_defines_ALU_FSGNJN = 7'b1000001;
	localparam riscv_defines_ALU_FSGNJX = 7'b1000010;
	always @(*) begin
		if (_sv2v_0)
			;
		if (FPU == 1) begin
			f_sign_inject_result[30:0] = operand_a_i[30:0];
			f_sign_inject_result[31] = operand_a_i[31];
			case (operator_i)
				riscv_defines_ALU_FKEEP: f_sign_inject_result[31] = operand_a_i[31];
				riscv_defines_ALU_FSGNJ: f_sign_inject_result[31] = operand_b_i[31];
				riscv_defines_ALU_FSGNJN: f_sign_inject_result[31] = !operand_b_i[31];
				riscv_defines_ALU_FSGNJX: f_sign_inject_result[31] = operand_a_i[31] ^ operand_b_i[31];
				default:
					;
			endcase
		end
		else
			f_sign_inject_result = 1'sb0;
	end
	reg [31:0] clip_result;
	always @(*) begin
		if (_sv2v_0)
			;
		clip_result = result_minmax;
		if (operator_i == riscv_defines_ALU_CLIPU) begin
			if (operand_a_i[31] || is_equal_clip)
				clip_result = 1'sb0;
			else
				clip_result = result_minmax;
		end
		else if (adder_result_expanded[36] || is_equal_clip)
			clip_result = operand_b_neg;
		else
			clip_result = result_minmax;
	end
	reg [7:0] shuffle_byte_sel;
	reg [3:0] shuffle_reg_sel;
	reg [1:0] shuffle_reg1_sel;
	reg [1:0] shuffle_reg0_sel;
	reg [3:0] shuffle_through;
	wire [31:0] shuffle_r1;
	wire [31:0] shuffle_r0;
	wire [31:0] shuffle_r1_in;
	wire [31:0] shuffle_r0_in;
	wire [31:0] shuffle_result;
	wire [31:0] pack_result;
	localparam riscv_defines_ALU_EXT = 7'b0111111;
	localparam riscv_defines_ALU_EXTS = 7'b0111110;
	localparam riscv_defines_ALU_INS = 7'b0101101;
	localparam riscv_defines_ALU_PCKHI = 7'b0111001;
	localparam riscv_defines_ALU_PCKLO = 7'b0111000;
	localparam riscv_defines_ALU_SHUF2 = 7'b0111011;
	always @(*) begin
		if (_sv2v_0)
			;
		shuffle_reg_sel = 1'sb0;
		shuffle_reg1_sel = 2'b01;
		shuffle_reg0_sel = 2'b10;
		shuffle_through = 1'sb1;
		case (operator_i)
			riscv_defines_ALU_EXT, riscv_defines_ALU_EXTS: begin
				if (operator_i == riscv_defines_ALU_EXTS)
					shuffle_reg1_sel = 2'b11;
				if (vector_mode_i == riscv_defines_VEC_MODE8) begin
					shuffle_reg_sel[3:1] = 3'b111;
					shuffle_reg_sel[0] = 1'b0;
				end
				else begin
					shuffle_reg_sel[3:2] = 2'b11;
					shuffle_reg_sel[1:0] = 2'b00;
				end
			end
			riscv_defines_ALU_PCKLO: begin
				shuffle_reg1_sel = 2'b00;
				if (vector_mode_i == riscv_defines_VEC_MODE8) begin
					shuffle_through = 4'b0011;
					shuffle_reg_sel = 4'b0001;
				end
				else
					shuffle_reg_sel = 4'b0011;
			end
			riscv_defines_ALU_PCKHI: begin
				shuffle_reg1_sel = 2'b00;
				if (vector_mode_i == riscv_defines_VEC_MODE8) begin
					shuffle_through = 4'b1100;
					shuffle_reg_sel = 4'b0100;
				end
				else
					shuffle_reg_sel = 4'b0011;
			end
			riscv_defines_ALU_SHUF2:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_reg_sel[3] = ~operand_b_i[26];
						shuffle_reg_sel[2] = ~operand_b_i[18];
						shuffle_reg_sel[1] = ~operand_b_i[10];
						shuffle_reg_sel[0] = ~operand_b_i[2];
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_reg_sel[3] = ~operand_b_i[17];
						shuffle_reg_sel[2] = ~operand_b_i[17];
						shuffle_reg_sel[1] = ~operand_b_i[1];
						shuffle_reg_sel[0] = ~operand_b_i[1];
					end
					default:
						;
				endcase
			riscv_defines_ALU_INS:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_reg0_sel = 2'b00;
						case (imm_vec_ext_i)
							2'b00: shuffle_reg_sel[3:0] = 4'b1110;
							2'b01: shuffle_reg_sel[3:0] = 4'b1101;
							2'b10: shuffle_reg_sel[3:0] = 4'b1011;
							2'b11: shuffle_reg_sel[3:0] = 4'b0111;
							default:
								;
						endcase
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_reg0_sel = 2'b01;
						shuffle_reg_sel[3] = ~imm_vec_ext_i[0];
						shuffle_reg_sel[2] = ~imm_vec_ext_i[0];
						shuffle_reg_sel[1] = imm_vec_ext_i[0];
						shuffle_reg_sel[0] = imm_vec_ext_i[0];
					end
					default:
						;
				endcase
			default:
				;
		endcase
	end
	localparam riscv_defines_ALU_SHUF = 7'b0111010;
	always @(*) begin
		if (_sv2v_0)
			;
		shuffle_byte_sel = 1'sb0;
		case (operator_i)
			riscv_defines_ALU_EXTS, riscv_defines_ALU_EXT:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[4+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[2+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[0+:2] = imm_vec_ext_i[1:0];
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[4+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[2+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[0+:2] = {imm_vec_ext_i[0], 1'b0};
					end
					default:
						;
				endcase
			riscv_defines_ALU_PCKLO:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = 2'b00;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b00;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = 2'b01;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b01;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					default:
						;
				endcase
			riscv_defines_ALU_PCKHI:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = 2'b00;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b00;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = 2'b11;
						shuffle_byte_sel[4+:2] = 2'b10;
						shuffle_byte_sel[2+:2] = 2'b11;
						shuffle_byte_sel[0+:2] = 2'b10;
					end
					default:
						;
				endcase
			riscv_defines_ALU_SHUF2, riscv_defines_ALU_SHUF:
				case (vector_mode_i)
					riscv_defines_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = operand_b_i[25:24];
						shuffle_byte_sel[4+:2] = operand_b_i[17:16];
						shuffle_byte_sel[2+:2] = operand_b_i[9:8];
						shuffle_byte_sel[0+:2] = operand_b_i[1:0];
					end
					riscv_defines_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = {operand_b_i[16], 1'b1};
						shuffle_byte_sel[4+:2] = {operand_b_i[16], 1'b0};
						shuffle_byte_sel[2+:2] = {operand_b_i[0], 1'b1};
						shuffle_byte_sel[0+:2] = {operand_b_i[0], 1'b0};
					end
					default:
						;
				endcase
			riscv_defines_ALU_INS: begin
				shuffle_byte_sel[6+:2] = 2'b11;
				shuffle_byte_sel[4+:2] = 2'b10;
				shuffle_byte_sel[2+:2] = 2'b01;
				shuffle_byte_sel[0+:2] = 2'b00;
			end
			default:
				;
		endcase
	end
	assign shuffle_r0_in = (shuffle_reg0_sel[1] ? operand_a_i : (shuffle_reg0_sel[0] ? {2 {operand_a_i[15:0]}} : {4 {operand_a_i[7:0]}}));
	assign shuffle_r1_in = (shuffle_reg1_sel[1] ? {{8 {operand_a_i[31]}}, {8 {operand_a_i[23]}}, {8 {operand_a_i[15]}}, {8 {operand_a_i[7]}}} : (shuffle_reg1_sel[0] ? operand_c_i : operand_b_i));
	assign shuffle_r0[31:24] = (shuffle_byte_sel[7] ? (shuffle_byte_sel[6] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[6] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[23:16] = (shuffle_byte_sel[5] ? (shuffle_byte_sel[4] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[4] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[15:8] = (shuffle_byte_sel[3] ? (shuffle_byte_sel[2] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[2] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[7:0] = (shuffle_byte_sel[1] ? (shuffle_byte_sel[0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[0] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r1[31:24] = (shuffle_byte_sel[7] ? (shuffle_byte_sel[6] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[6] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[23:16] = (shuffle_byte_sel[5] ? (shuffle_byte_sel[4] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[4] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[15:8] = (shuffle_byte_sel[3] ? (shuffle_byte_sel[2] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[2] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[7:0] = (shuffle_byte_sel[1] ? (shuffle_byte_sel[0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[0] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_result[31:24] = (shuffle_reg_sel[3] ? shuffle_r1[31:24] : shuffle_r0[31:24]);
	assign shuffle_result[23:16] = (shuffle_reg_sel[2] ? shuffle_r1[23:16] : shuffle_r0[23:16]);
	assign shuffle_result[15:8] = (shuffle_reg_sel[1] ? shuffle_r1[15:8] : shuffle_r0[15:8]);
	assign shuffle_result[7:0] = (shuffle_reg_sel[0] ? shuffle_r1[7:0] : shuffle_r0[7:0]);
	assign pack_result[31:24] = (shuffle_through[3] ? shuffle_result[31:24] : operand_c_i[31:24]);
	assign pack_result[23:16] = (shuffle_through[2] ? shuffle_result[23:16] : operand_c_i[23:16]);
	assign pack_result[15:8] = (shuffle_through[1] ? shuffle_result[15:8] : operand_c_i[15:8]);
	assign pack_result[7:0] = (shuffle_through[0] ? shuffle_result[7:0] : operand_c_i[7:0]);
	reg [31:0] ff_input;
	wire [5:0] cnt_result;
	wire [5:0] clb_result;
	wire [4:0] ff1_result;
	wire ff_no_one;
	wire [4:0] fl1_result;
	reg [5:0] bitop_result;
	alu_popcnt alu_popcnt_i(
		.in_i(operand_a_i),
		.result_o(cnt_result)
	);
	localparam riscv_defines_ALU_FF1 = 7'b0110110;
	always @(*) begin
		if (_sv2v_0)
			;
		ff_input = 1'sb0;
		case (operator_i)
			riscv_defines_ALU_FF1: ff_input = operand_a_i;
			riscv_defines_ALU_DIVU, riscv_defines_ALU_REMU, riscv_defines_ALU_FL1: ff_input = operand_a_rev;
			riscv_defines_ALU_DIV, riscv_defines_ALU_REM, riscv_defines_ALU_CLB:
				if (operand_a_i[31])
					ff_input = operand_a_neg_rev;
				else
					ff_input = operand_a_rev;
		endcase
	end
	alu_ff alu_ff_i(
		.in_i(ff_input),
		.first_one_o(ff1_result),
		.no_ones_o(ff_no_one)
	);
	assign fl1_result = 5'd31 - ff1_result;
	assign clb_result = ff1_result - 5'd1;
	localparam riscv_defines_ALU_CNT = 7'b0110100;
	always @(*) begin
		if (_sv2v_0)
			;
		bitop_result = 1'sb0;
		case (operator_i)
			riscv_defines_ALU_FF1: bitop_result = (ff_no_one ? 6'd32 : {1'b0, ff1_result});
			riscv_defines_ALU_FL1: bitop_result = (ff_no_one ? 6'd32 : {1'b0, fl1_result});
			riscv_defines_ALU_CNT: bitop_result = cnt_result;
			riscv_defines_ALU_CLB:
				if (ff_no_one) begin
					if (operand_a_i[31])
						bitop_result = 6'd31;
					else
						bitop_result = 1'sb0;
				end
				else
					bitop_result = clb_result;
			default:
				;
		endcase
	end
	wire extract_is_signed;
	wire extract_sign;
	wire [31:0] bmask_first;
	wire [31:0] bmask_inv;
	wire [31:0] bextins_and;
	wire [31:0] bextins_result;
	wire [31:0] bclr_result;
	wire [31:0] bset_result;
	assign bmask_first = 32'hfffffffe << bmask_a_i;
	assign bmask = ~bmask_first << bmask_b_i;
	assign bmask_inv = ~bmask;
	assign bextins_and = (operator_i == riscv_defines_ALU_BINS ? operand_c_i : {32 {extract_sign}});
	assign extract_is_signed = operator_i == riscv_defines_ALU_BEXT;
	assign extract_sign = extract_is_signed & shift_result[bmask_a_i];
	assign bextins_result = (bmask & shift_result) | (bextins_and & bmask_inv);
	assign bclr_result = operand_a_i & bmask_inv;
	assign bset_result = operand_a_i | bmask;
	wire [31:0] radix_2_rev;
	wire [31:0] radix_4_rev;
	wire [31:0] radix_8_rev;
	reg [31:0] reverse_result;
	wire [1:0] radix_mux_sel;
	assign radix_mux_sel = bmask_a_i[1:0];
	generate
		for (_gv_j_2 = 0; _gv_j_2 < 32; _gv_j_2 = _gv_j_2 + 1) begin : genblk6
			localparam j = _gv_j_2;
			assign radix_2_rev[j] = shift_result[31 - j];
		end
		for (_gv_j_2 = 0; _gv_j_2 < 16; _gv_j_2 = _gv_j_2 + 1) begin : genblk7
			localparam j = _gv_j_2;
			assign radix_4_rev[(2 * j) + 1:2 * j] = shift_result[31 - (j * 2):(31 - (j * 2)) - 1];
		end
		for (_gv_j_2 = 0; _gv_j_2 < 10; _gv_j_2 = _gv_j_2 + 1) begin : genblk8
			localparam j = _gv_j_2;
			assign radix_8_rev[(3 * j) + 2:3 * j] = shift_result[31 - (j * 3):(31 - (j * 3)) - 2];
		end
	endgenerate
	assign radix_8_rev[31:30] = 2'b00;
	always @(*) begin
		if (_sv2v_0)
			;
		reverse_result = 1'sb0;
		case (radix_mux_sel)
			2'b00: reverse_result = radix_2_rev;
			2'b01: reverse_result = radix_4_rev;
			2'b10: reverse_result = radix_8_rev;
			default: reverse_result = radix_2_rev;
		endcase
	end
	wire [31:0] result_div;
	wire div_ready;
	generate
		if (SHARED_INT_DIV == 1) begin : genblk9
			assign result_div = 1'sb0;
			assign div_ready = 1'sb1;
			assign div_valid = 1'sb0;
		end
		else begin : int_div
			wire div_signed;
			wire div_op_a_signed;
			wire div_op_b_signed;
			wire [5:0] div_shift_int;
			assign div_signed = operator_i[0];
			assign div_op_a_signed = operand_a_i[31] & div_signed;
			assign div_op_b_signed = operand_b_i[31] & div_signed;
			assign div_shift_int = (ff_no_one ? 6'd31 : clb_result);
			assign div_shift = div_shift_int + (div_op_a_signed ? 6'd0 : 6'd1);
			assign div_valid = enable_i & ((((operator_i == riscv_defines_ALU_DIV) || (operator_i == riscv_defines_ALU_DIVU)) || (operator_i == riscv_defines_ALU_REM)) || (operator_i == riscv_defines_ALU_REMU));
			riscv_alu_div div_i(
				.Clk_CI(clk),
				.Rst_RBI(rst_n),
				.OpA_DI(operand_b_i),
				.OpB_DI(shift_left_result),
				.OpBShift_DI(div_shift),
				.OpBIsZero_SI(cnt_result == 0),
				.OpBSign_SI(div_op_a_signed),
				.OpCode_SI(operator_i[1:0]),
				.Res_DO(result_div),
				.InVld_SI(div_valid),
				.OutRdy_SI(ex_ready_i),
				.OutVld_SO(div_ready)
			);
		end
	endgenerate
	localparam riscv_defines_ALU_AND = 7'b0010101;
	localparam riscv_defines_ALU_BCLR = 7'b0101011;
	localparam riscv_defines_ALU_BEXTU = 7'b0101001;
	localparam riscv_defines_ALU_BSET = 7'b0101100;
	localparam riscv_defines_ALU_FCLASS = 7'b1001000;
	localparam riscv_defines_ALU_MAXU = 7'b0010011;
	localparam riscv_defines_ALU_OR = 7'b0101110;
	localparam riscv_defines_ALU_SRL = 7'b0100101;
	localparam riscv_defines_ALU_XOR = 7'b0101111;
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sb0;
		case (operator_i)
			riscv_defines_ALU_AND: result_o = operand_a_i & operand_b_i;
			riscv_defines_ALU_OR: result_o = operand_a_i | operand_b_i;
			riscv_defines_ALU_XOR: result_o = operand_a_i ^ operand_b_i;
			riscv_defines_ALU_ADD, riscv_defines_ALU_ADDR, riscv_defines_ALU_ADDU, riscv_defines_ALU_ADDUR, riscv_defines_ALU_SUB, riscv_defines_ALU_SUBR, riscv_defines_ALU_SUBU, riscv_defines_ALU_SUBUR, riscv_defines_ALU_SLL, riscv_defines_ALU_SRL, riscv_defines_ALU_SRA, riscv_defines_ALU_ROR: result_o = shift_result;
			riscv_defines_ALU_BINS, riscv_defines_ALU_BEXT, riscv_defines_ALU_BEXTU: result_o = bextins_result;
			riscv_defines_ALU_BCLR: result_o = bclr_result;
			riscv_defines_ALU_BSET: result_o = bset_result;
			riscv_defines_ALU_BREV: result_o = reverse_result;
			riscv_defines_ALU_SHUF, riscv_defines_ALU_SHUF2, riscv_defines_ALU_PCKLO, riscv_defines_ALU_PCKHI, riscv_defines_ALU_EXT, riscv_defines_ALU_EXTS, riscv_defines_ALU_INS: result_o = pack_result;
			riscv_defines_ALU_MIN, riscv_defines_ALU_MINU, riscv_defines_ALU_MAX, riscv_defines_ALU_MAXU, riscv_defines_ALU_FMIN, riscv_defines_ALU_FMAX: result_o = (minmax_is_fp_special ? fp_canonical_nan : result_minmax);
			riscv_defines_ALU_ABS: result_o = (is_clpx_i ? {adder_result[31:16], operand_a_i[15:0]} : result_minmax);
			riscv_defines_ALU_CLIP, riscv_defines_ALU_CLIPU: result_o = clip_result;
			riscv_defines_ALU_EQ, riscv_defines_ALU_NE, riscv_defines_ALU_GTU, riscv_defines_ALU_GEU, riscv_defines_ALU_LTU, riscv_defines_ALU_LEU, riscv_defines_ALU_GTS, riscv_defines_ALU_GES, riscv_defines_ALU_LTS, riscv_defines_ALU_LES: begin
				result_o[31:24] = {8 {cmp_result[3]}};
				result_o[23:16] = {8 {cmp_result[2]}};
				result_o[15:8] = {8 {cmp_result[1]}};
				result_o[7:0] = {8 {cmp_result[0]}};
			end
			riscv_defines_ALU_FEQ, riscv_defines_ALU_FLT, riscv_defines_ALU_FLE, riscv_defines_ALU_SLTS, riscv_defines_ALU_SLTU, riscv_defines_ALU_SLETS, riscv_defines_ALU_SLETU: result_o = {31'b0000000000000000000000000000000, comparison_result_o};
			riscv_defines_ALU_FF1, riscv_defines_ALU_FL1, riscv_defines_ALU_CLB, riscv_defines_ALU_CNT: result_o = {26'h0000000, bitop_result[5:0]};
			riscv_defines_ALU_DIV, riscv_defines_ALU_DIVU, riscv_defines_ALU_REM, riscv_defines_ALU_REMU: result_o = result_div;
			riscv_defines_ALU_FCLASS: result_o = fclass_result;
			riscv_defines_ALU_FSGNJ, riscv_defines_ALU_FSGNJN, riscv_defines_ALU_FSGNJX, riscv_defines_ALU_FKEEP: result_o = f_sign_inject_result;
			default:
				;
		endcase
	end
	assign ready_o = div_ready;
	initial _sv2v_0 = 0;
endmodule
module alu_ff (
	in_i,
	first_one_o,
	no_ones_o
);
	parameter LEN = 32;
	input wire [LEN - 1:0] in_i;
	output wire [$clog2(LEN) - 1:0] first_one_o;
	output wire no_ones_o;
	localparam NUM_LEVELS = $clog2(LEN);
	wire [(LEN * NUM_LEVELS) - 1:0] index_lut;
	wire [(2 ** NUM_LEVELS) - 1:0] sel_nodes;
	wire [((2 ** NUM_LEVELS) * NUM_LEVELS) - 1:0] index_nodes;
	genvar _gv_j_3;
	generate
		for (_gv_j_3 = 0; _gv_j_3 < LEN; _gv_j_3 = _gv_j_3 + 1) begin : genblk1
			localparam j = _gv_j_3;
			assign index_lut[j * NUM_LEVELS+:NUM_LEVELS] = $unsigned(j);
		end
	endgenerate
	genvar _gv_k_3;
	genvar _gv_l_2;
	genvar _gv_level_1;
	generate
		for (_gv_level_1 = 0; _gv_level_1 < NUM_LEVELS; _gv_level_1 = _gv_level_1 + 1) begin : genblk2
			localparam level = _gv_level_1;
			if (level < (NUM_LEVELS - 1)) begin : genblk1
				for (_gv_l_2 = 0; _gv_l_2 < (2 ** level); _gv_l_2 = _gv_l_2 + 1) begin : genblk1
					localparam l = _gv_l_2;
					assign sel_nodes[((2 ** level) - 1) + l] = sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] | sel_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) + 1];
					assign index_nodes[(((2 ** level) - 1) + l) * NUM_LEVELS+:NUM_LEVELS] = (sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] == 1'b1 ? index_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) * NUM_LEVELS+:NUM_LEVELS] : index_nodes[((((2 ** (level + 1)) - 1) + (l * 2)) + 1) * NUM_LEVELS+:NUM_LEVELS]);
				end
			end
			if (level == (NUM_LEVELS - 1)) begin : genblk2
				for (_gv_k_3 = 0; _gv_k_3 < (2 ** level); _gv_k_3 = _gv_k_3 + 1) begin : genblk1
					localparam k = _gv_k_3;
					if ((k * 2) < (LEN - 1)) begin : genblk1
						assign sel_nodes[((2 ** level) - 1) + k] = in_i[k * 2] | in_i[(k * 2) + 1];
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = (in_i[k * 2] == 1'b1 ? index_lut[(k * 2) * NUM_LEVELS+:NUM_LEVELS] : index_lut[((k * 2) + 1) * NUM_LEVELS+:NUM_LEVELS]);
					end
					if ((k * 2) == (LEN - 1)) begin : genblk2
						assign sel_nodes[((2 ** level) - 1) + k] = in_i[k * 2];
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = index_lut[(k * 2) * NUM_LEVELS+:NUM_LEVELS];
					end
					if ((k * 2) > (LEN - 1)) begin : genblk3
						assign sel_nodes[((2 ** level) - 1) + k] = 1'b0;
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = 1'sb0;
					end
				end
			end
		end
	endgenerate
	assign first_one_o = index_nodes[0+:NUM_LEVELS];
	assign no_ones_o = ~sel_nodes[0];
endmodule
module alu_popcnt (
	in_i,
	result_o
);
	input wire [31:0] in_i;
	output wire [5:0] result_o;
	wire [31:0] cnt_l1;
	wire [23:0] cnt_l2;
	wire [15:0] cnt_l3;
	wire [9:0] cnt_l4;
	genvar _gv_l_3;
	genvar _gv_m_2;
	genvar _gv_n_1;
	genvar _gv_p_1;
	generate
		for (_gv_l_3 = 0; _gv_l_3 < 16; _gv_l_3 = _gv_l_3 + 1) begin : genblk1
			localparam l = _gv_l_3;
			assign cnt_l1[l * 2+:2] = {1'b0, in_i[2 * l]} + {1'b0, in_i[(2 * l) + 1]};
		end
		for (_gv_m_2 = 0; _gv_m_2 < 8; _gv_m_2 = _gv_m_2 + 1) begin : genblk2
			localparam m = _gv_m_2;
			assign cnt_l2[m * 3+:3] = {1'b0, cnt_l1[(2 * m) * 2+:2]} + {1'b0, cnt_l1[((2 * m) + 1) * 2+:2]};
		end
		for (_gv_n_1 = 0; _gv_n_1 < 4; _gv_n_1 = _gv_n_1 + 1) begin : genblk3
			localparam n = _gv_n_1;
			assign cnt_l3[n * 4+:4] = {1'b0, cnt_l2[(2 * n) * 3+:3]} + {1'b0, cnt_l2[((2 * n) + 1) * 3+:3]};
		end
		for (_gv_p_1 = 0; _gv_p_1 < 2; _gv_p_1 = _gv_p_1 + 1) begin : genblk4
			localparam p = _gv_p_1;
			assign cnt_l4[p * 5+:5] = {1'b0, cnt_l3[(2 * p) * 4+:4]} + {1'b0, cnt_l3[((2 * p) + 1) * 4+:4]};
		end
	endgenerate
	assign result_o = {1'b0, cnt_l4[0+:5]} + {1'b0, cnt_l4[5+:5]};
endmodule
module riscv_pmp (
	clk,
	rst_n,
	pmp_privil_mode_i,
	pmp_addr_i,
	pmp_cfg_i,
	data_req_i,
	data_addr_i,
	data_we_i,
	data_gnt_o,
	data_req_o,
	data_gnt_i,
	data_addr_o,
	data_err_o,
	data_err_ack_i,
	instr_req_i,
	instr_addr_i,
	instr_gnt_o,
	instr_req_o,
	instr_gnt_i,
	instr_addr_o,
	instr_err_o
);
	reg _sv2v_0;
	parameter N_PMP_ENTRIES = 16;
	input wire clk;
	input wire rst_n;
	input wire [1:0] pmp_privil_mode_i;
	input wire [(N_PMP_ENTRIES * 32) - 1:0] pmp_addr_i;
	input wire [(N_PMP_ENTRIES * 8) - 1:0] pmp_cfg_i;
	input wire data_req_i;
	input wire [31:0] data_addr_i;
	input wire data_we_i;
	output reg data_gnt_o;
	output reg data_req_o;
	input wire data_gnt_i;
	output wire [31:0] data_addr_o;
	output reg data_err_o;
	input wire data_err_ack_i;
	input wire instr_req_i;
	input wire [31:0] instr_addr_i;
	output reg instr_gnt_o;
	output reg instr_req_o;
	input wire instr_gnt_i;
	output wire [31:0] instr_addr_o;
	output reg instr_err_o;
	reg [N_PMP_ENTRIES - 1:0] EN_rule;
	wire [N_PMP_ENTRIES - 1:0] R_rule;
	wire [N_PMP_ENTRIES - 1:0] W_rule;
	wire [N_PMP_ENTRIES - 1:0] X_rule;
	wire [(N_PMP_ENTRIES * 2) - 1:0] MODE_rule;
	wire [(N_PMP_ENTRIES * 2) - 1:0] WIRI_rule;
	wire [(N_PMP_ENTRIES * 2) - 1:0] LOCK_rule;
	reg [(N_PMP_ENTRIES * 32) - 1:0] mask_addr;
	reg [(N_PMP_ENTRIES * 32) - 1:0] start_addr;
	reg [(N_PMP_ENTRIES * 32) - 1:0] stop_addr;
	reg [N_PMP_ENTRIES - 1:0] data_match_region;
	reg [N_PMP_ENTRIES - 1:0] instr_match_region;
	reg data_err_int;
	genvar _gv_i_6;
	reg [31:0] j;
	reg [31:0] k;
	generate
		for (_gv_i_6 = 0; _gv_i_6 < N_PMP_ENTRIES; _gv_i_6 = _gv_i_6 + 1) begin : CFG_EXP
			localparam i = _gv_i_6;
			assign {LOCK_rule[i * 2+:2], WIRI_rule[i * 2+:2], MODE_rule[i * 2+:2], X_rule[i], W_rule[i], R_rule[i]} = pmp_cfg_i[i * 8+:8];
		end
		for (_gv_i_6 = 0; _gv_i_6 < N_PMP_ENTRIES; _gv_i_6 = _gv_i_6 + 1) begin : ADDR_EXP
			localparam i = _gv_i_6;
			always @(*) begin
				start_addr[i * 32+:32] = 1'sb0;
				stop_addr[i * 32+:32] = 1'sb0;
				mask_addr[i * 32+:32] = 32'hffffffff;
				case (MODE_rule[i * 2+:2])
					2'b00: begin : DISABLED
						EN_rule[i] = 1'b0;
					end
					2'b01: begin : TOR_MODE
						EN_rule[i] = 1'b1;
						if (i == 0)
							start_addr[i * 32+:32] = 0;
						else
							start_addr[i * 32+:32] = pmp_addr_i[(i - 1) * 32+:32];
						stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
					end
					2'b10: begin : NA4_MODE
						EN_rule[i] = 1'b1;
						stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
						start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
					end
					2'b11: begin : NAPOT_MODE
						EN_rule[i] = 1'b1;
						mask_addr[i * 32+:32] = 32'hffffffff;
						stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
						start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
						casex (pmp_addr_i[i * 32+:32])
							32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0: begin : BYTE_ALIGN_8B
								mask_addr[i * 32+:32] = 32'hfffffffe;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx01: begin : BYTE_ALIGN_16B
								mask_addr[i * 32+:32] = 32'hfffffffc;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxx011: begin : BYTE_ALIGN_32B
								mask_addr[i * 32+:32] = 32'hfffffff8;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxx0111: begin : BYTE_ALIGN_64B
								mask_addr[i * 32+:32] = 32'hfffffff0;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxxxxx01111: begin : BYTE_ALIGN_128B
								mask_addr[i * 32+:32] = 32'hffffffe0;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & 32'hffffffe0;
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxxxx011111: begin : BYTE_ALIGN_256B
								mask_addr[i * 32+:32] = 32'hffffffc0;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxxx0111111: begin : BYTE_ALIGN_512B
								mask_addr[i * 32+:32] = 32'hffffff80;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxxx01111111: begin : BYTE_ALIGN_1KB
								mask_addr[i * 32+:32] = 32'hffffff00;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxxx011111111: begin : BYTE_ALIGN_2KB
								mask_addr[i * 32+:32] = 32'hfffffe00;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxxx0111111111: begin : BYTE_ALIGN_4KB
								mask_addr[i * 32+:32] = 32'hfffffc00;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxxx01111111111: begin : BYTE_ALIGN_8KB
								mask_addr[i * 32+:32] = 32'hfffff800;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxxx011111111111: begin : BYTE_ALIGN_16KB
								mask_addr[i * 32+:32] = 32'hfffff000;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxxx0111111111111: begin : BYTE_ALIGN_32KB
								mask_addr[i * 32+:32] = 32'hffffe000;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxxx01111111111111: begin : BYTE_ALIGN_64KB
								mask_addr[i * 32+:32] = 32'hffffc000;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxxx011111111111111: begin : BYTE_ALIGN_128KB
								mask_addr[i * 32+:32] = 32'hffff8000;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							32'bxxxxxxxxxxxxxxxx0111111111111111: begin : BYTE_ALIGN_256KB
								mask_addr[i * 32+:32] = 32'hffff0000;
								start_addr[i * 32+:32] = pmp_addr_i[i * 32+:32] & mask_addr[i * 32+:32];
								stop_addr[i * 32+:32] = pmp_addr_i[i * 32+:32];
							end
							default: begin : INVALID_RULE
								EN_rule[i] = 1'b0;
								start_addr[i * 32+:32] = 1'sb0;
								stop_addr[i * 32+:32] = 1'sb0;
							end
						endcase
					end
					default: begin : DEFAULT_DISABLED
						EN_rule[i] = 1'b0;
						start_addr[i * 32+:32] = 1'sb0;
						stop_addr[i * 32+:32] = 1'sb0;
					end
				endcase
			end
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		for (j = 0; j < N_PMP_ENTRIES; j = j + 1)
			if (EN_rule[j] & ((~data_we_i & R_rule[j]) | (data_we_i & W_rule[j])))
				case (MODE_rule[j * 2+:2])
					2'b01: begin : TOR_CHECK_DATA
						if ((data_addr_i[31:2] >= start_addr[j * 32+:32]) && (data_addr_i[31:2] < stop_addr[j * 32+:32]))
							data_match_region[j] = 1'b1;
						else
							data_match_region[j] = 1'b0;
					end
					2'b10: begin : NA4_CHECK_DATA
						if (data_addr_i[31:2] == start_addr[(j * 32) + 29-:30])
							data_match_region[j] = 1'b1;
						else
							data_match_region[j] = 1'b0;
					end
					2'b11: begin : NAPOT_CHECK_DATA
						if ((data_addr_i[31:2] & mask_addr[(j * 32) + 29-:30]) == start_addr[(j * 32) + 29-:30])
							data_match_region[j] = 1'b1;
						else
							data_match_region[j] = 1'b0;
					end
					default: data_match_region[j] = 1'b0;
				endcase
			else
				data_match_region[j] = 1'b0;
	end
	assign data_addr_o = data_addr_i;
	always @(*) begin
		if (_sv2v_0)
			;
		if (pmp_privil_mode_i == 2'b11) begin
			data_req_o = data_req_i;
			data_gnt_o = data_gnt_i;
			data_err_int = 1'b0;
		end
		else if (|data_match_region == 1'b0) begin
			data_req_o = 1'b0;
			data_err_int = data_req_i;
			data_gnt_o = 1'b0;
		end
		else begin
			data_req_o = data_req_i;
			data_err_int = 1'b0;
			data_gnt_o = data_gnt_i;
		end
	end
	reg data_err_state_q;
	reg data_err_state_n;
	always @(*) begin
		if (_sv2v_0)
			;
		data_err_o = 1'b0;
		data_err_state_n = data_err_state_q;
		case (data_err_state_q)
			1'd0:
				if (data_err_int)
					data_err_state_n = 1'd1;
			1'd1: begin
				data_err_o = 1'b1;
				if (data_err_ack_i)
					data_err_state_n = 1'd0;
			end
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n)
			data_err_state_q <= 1'd0;
		else
			data_err_state_q <= data_err_state_n;
	always @(*) begin
		if (_sv2v_0)
			;
		for (k = 0; k < N_PMP_ENTRIES; k = k + 1)
			if (EN_rule[k] & X_rule[k])
				case (MODE_rule[k * 2+:2])
					2'b01: begin : TOR_CHECK
						if ((instr_addr_i[31:2] >= start_addr[k * 32+:32]) && (instr_addr_i[31:2] < stop_addr[k * 32+:32]))
							instr_match_region[k] = 1'b1;
						else
							instr_match_region[k] = 1'b0;
					end
					2'b10: begin : NA4_CHECK
						if (instr_addr_i[31:2] == start_addr[(k * 32) + 29-:30])
							instr_match_region[k] = 1'b1;
						else
							instr_match_region[k] = 1'b0;
					end
					2'b11:
						if ((instr_addr_i[31:2] & mask_addr[(k * 32) + 29-:30]) == start_addr[(k * 32) + 29-:30])
							instr_match_region[k] = 1'b1;
						else
							instr_match_region[k] = 1'b0;
					default: instr_match_region[k] = 1'b0;
				endcase
			else
				instr_match_region[k] = 1'b0;
	end
	assign instr_addr_o = instr_addr_i;
	always @(*) begin
		if (_sv2v_0)
			;
		if (pmp_privil_mode_i == 2'b11) begin
			instr_req_o = instr_req_i;
			instr_gnt_o = instr_gnt_i;
			instr_err_o = 1'b0;
		end
		else if (|instr_match_region == 1'b0) begin
			instr_req_o = 1'b0;
			instr_err_o = instr_req_i;
			instr_gnt_o = 1'b0;
		end
		else begin
			instr_req_o = instr_req_i;
			instr_err_o = 1'b0;
			instr_gnt_o = instr_gnt_i;
		end
	end
	initial _sv2v_0 = 0;
endmodule
module riscv_apu_disp (
	clk_i,
	rst_ni,
	enable_i,
	apu_lat_i,
	apu_waddr_i,
	apu_waddr_o,
	apu_multicycle_o,
	apu_singlecycle_o,
	active_o,
	stall_o,
	read_regs_i,
	read_regs_valid_i,
	read_dep_o,
	write_regs_i,
	write_regs_valid_i,
	write_dep_o,
	perf_type_o,
	perf_cont_o,
	apu_master_req_o,
	apu_master_ready_o,
	apu_master_gnt_i,
	apu_master_valid_i
);
	reg _sv2v_0;
	input wire clk_i;
	input wire rst_ni;
	input wire enable_i;
	input wire [1:0] apu_lat_i;
	input wire [5:0] apu_waddr_i;
	output reg [5:0] apu_waddr_o;
	output wire apu_multicycle_o;
	output wire apu_singlecycle_o;
	output wire active_o;
	output wire stall_o;
	input wire [17:0] read_regs_i;
	input wire [2:0] read_regs_valid_i;
	output wire read_dep_o;
	input wire [11:0] write_regs_i;
	input wire [1:0] write_regs_valid_i;
	output wire write_dep_o;
	output wire perf_type_o;
	output wire perf_cont_o;
	output wire apu_master_req_o;
	output wire apu_master_ready_o;
	input wire apu_master_gnt_i;
	input wire apu_master_valid_i;
	wire [5:0] addr_req;
	reg [5:0] addr_inflight;
	reg [5:0] addr_waiting;
	reg [5:0] addr_inflight_dn;
	reg [5:0] addr_waiting_dn;
	wire valid_req;
	reg valid_inflight;
	reg valid_waiting;
	reg valid_inflight_dn;
	reg valid_waiting_dn;
	wire returned_req;
	wire returned_inflight;
	wire returned_waiting;
	wire req_accepted;
	wire active;
	reg [1:0] apu_lat;
	wire [2:0] read_deps_req;
	wire [2:0] read_deps_inflight;
	wire [2:0] read_deps_waiting;
	wire [1:0] write_deps_req;
	wire [1:0] write_deps_inflight;
	wire [1:0] write_deps_waiting;
	wire read_dep_req;
	wire read_dep_inflight;
	wire read_dep_waiting;
	wire write_dep_req;
	wire write_dep_inflight;
	wire write_dep_waiting;
	wire stall_full;
	wire stall_type;
	wire stall_nack;
	assign valid_req = enable_i & !(stall_full | stall_type);
	assign addr_req = apu_waddr_i;
	assign req_accepted = valid_req & apu_master_gnt_i;
	assign returned_req = ((valid_req & apu_master_valid_i) & !valid_inflight) & !valid_waiting;
	assign returned_inflight = (valid_inflight & apu_master_valid_i) & !valid_waiting;
	assign returned_waiting = valid_waiting & apu_master_valid_i;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni) begin
			valid_inflight <= 1'b0;
			valid_waiting <= 1'b0;
			addr_inflight <= 1'sb0;
			addr_waiting <= 1'sb0;
		end
		else begin
			valid_inflight <= valid_inflight_dn;
			valid_waiting <= valid_waiting_dn;
			addr_inflight <= addr_inflight_dn;
			addr_waiting <= addr_waiting_dn;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		valid_inflight_dn = valid_inflight;
		valid_waiting_dn = valid_waiting;
		addr_inflight_dn = addr_inflight;
		addr_waiting_dn = addr_waiting;
		if (req_accepted & !returned_req) begin
			valid_inflight_dn = 1'b1;
			addr_inflight_dn = addr_req;
			if (valid_inflight & !returned_inflight) begin
				valid_waiting_dn = 1'b1;
				addr_waiting_dn = addr_inflight;
			end
			if (returned_waiting) begin
				valid_waiting_dn = 1'b1;
				addr_waiting_dn = addr_inflight;
			end
		end
		else if (returned_inflight) begin
			valid_inflight_dn = 1'sb0;
			valid_waiting_dn = 1'sb0;
			addr_inflight_dn = 1'sb0;
			addr_waiting_dn = 1'sb0;
		end
		else if (returned_waiting) begin
			valid_waiting_dn = 1'sb0;
			addr_waiting_dn = 1'sb0;
		end
	end
	assign active = valid_inflight | valid_waiting;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni)
			apu_lat <= 1'sb0;
		else if (valid_req)
			apu_lat <= apu_lat_i;
	genvar _gv_i_7;
	generate
		for (_gv_i_7 = 0; _gv_i_7 < 3; _gv_i_7 = _gv_i_7 + 1) begin : genblk1
			localparam i = _gv_i_7;
			assign read_deps_req[i] = (read_regs_i[i * 6+:6] == addr_req) & read_regs_valid_i[i];
			assign read_deps_inflight[i] = (read_regs_i[i * 6+:6] == addr_inflight) & read_regs_valid_i[i];
			assign read_deps_waiting[i] = (read_regs_i[i * 6+:6] == addr_waiting) & read_regs_valid_i[i];
		end
	endgenerate
	genvar _gv_i_8;
	generate
		for (_gv_i_8 = 0; _gv_i_8 < 2; _gv_i_8 = _gv_i_8 + 1) begin : genblk2
			localparam i = _gv_i_8;
			assign write_deps_req[i] = (write_regs_i[i * 6+:6] == addr_req) & write_regs_valid_i[i];
			assign write_deps_inflight[i] = (write_regs_i[i * 6+:6] == addr_inflight) & write_regs_valid_i[i];
			assign write_deps_waiting[i] = (write_regs_i[i * 6+:6] == addr_waiting) & write_regs_valid_i[i];
		end
	endgenerate
	assign read_dep_req = (|read_deps_req & valid_req) & !returned_req;
	assign read_dep_inflight = (|read_deps_inflight & valid_inflight) & !returned_inflight;
	assign read_dep_waiting = (|read_deps_waiting & valid_waiting) & !returned_waiting;
	assign write_dep_req = (|write_deps_req & valid_req) & !returned_req;
	assign write_dep_inflight = (|write_deps_inflight & valid_inflight) & !returned_inflight;
	assign write_dep_waiting = (|write_deps_waiting & valid_waiting) & !returned_waiting;
	assign read_dep_o = (read_dep_req | read_dep_inflight) | read_dep_waiting;
	assign write_dep_o = (write_dep_req | write_dep_inflight) | write_dep_waiting;
	assign stall_full = valid_inflight & valid_waiting;
	assign stall_type = (enable_i & active) & (((apu_lat_i == 2'h1) | ((apu_lat_i == 2'h2) & (apu_lat == 2'h3))) | (apu_lat_i == 2'h3));
	assign stall_nack = valid_req & !apu_master_gnt_i;
	assign stall_o = (stall_full | stall_type) | stall_nack;
	assign apu_master_req_o = valid_req;
	assign apu_master_ready_o = 1'b1;
	always @(*) begin
		if (_sv2v_0)
			;
		apu_waddr_o = 1'sb0;
		if (returned_req)
			apu_waddr_o = addr_req;
		if (returned_inflight)
			apu_waddr_o = addr_inflight;
		if (returned_waiting)
			apu_waddr_o = addr_waiting;
	end
	assign active_o = active;
	assign perf_type_o = stall_type;
	assign perf_cont_o = stall_nack;
	assign apu_multicycle_o = apu_lat == 2'h3;
	assign apu_singlecycle_o = ~(valid_inflight | valid_waiting);
	initial _sv2v_0 = 0;
endmodule
module riscv_controller (
	clk,
	rst_n,
	fetch_enable_i,
	ctrl_busy_o,
	first_fetch_o,
	is_decoding_o,
	is_fetch_failed_i,
	deassert_we_o,
	illegal_insn_i,
	ecall_insn_i,
	mret_insn_i,
	uret_insn_i,
	dret_insn_i,
	mret_dec_i,
	uret_dec_i,
	dret_dec_i,
	pipe_flush_i,
	ebrk_insn_i,
	fencei_insn_i,
	csr_status_i,
	instr_multicycle_i,
	hwloop_mask_o,
	instr_valid_i,
	instr_req_o,
	pc_set_o,
	pc_mux_o,
	exc_pc_mux_o,
	trap_addr_mux_o,
	data_req_ex_i,
	data_we_ex_i,
	data_misaligned_i,
	data_load_event_i,
	data_err_i,
	data_err_ack_o,
	mult_multicycle_i,
	apu_en_i,
	apu_read_dep_i,
	apu_write_dep_i,
	apu_stall_o,
	branch_taken_ex_i,
	jump_in_id_i,
	jump_in_dec_i,
	irq_pending_i,
	irq_req_ctrl_i,
	irq_sec_ctrl_i,
	irq_id_ctrl_i,
	m_IE_i,
	u_IE_i,
	current_priv_lvl_i,
	irq_ack_o,
	irq_id_o,
	exc_cause_o,
	exc_ack_o,
	exc_kill_o,
	debug_mode_o,
	debug_cause_o,
	debug_csr_save_o,
	debug_req_i,
	debug_single_step_i,
	debug_ebreakm_i,
	debug_ebreaku_i,
	csr_save_if_o,
	csr_save_id_o,
	csr_save_ex_o,
	csr_cause_o,
	csr_irq_sec_o,
	csr_restore_mret_id_o,
	csr_restore_uret_id_o,
	csr_restore_dret_id_o,
	csr_save_cause_o,
	regfile_we_id_i,
	regfile_alu_waddr_id_i,
	regfile_we_ex_i,
	regfile_waddr_ex_i,
	regfile_we_wb_i,
	regfile_alu_we_fw_i,
	operand_a_fw_mux_sel_o,
	operand_b_fw_mux_sel_o,
	operand_c_fw_mux_sel_o,
	reg_d_ex_is_reg_a_i,
	reg_d_ex_is_reg_b_i,
	reg_d_ex_is_reg_c_i,
	reg_d_wb_is_reg_a_i,
	reg_d_wb_is_reg_b_i,
	reg_d_wb_is_reg_c_i,
	reg_d_alu_is_reg_a_i,
	reg_d_alu_is_reg_b_i,
	reg_d_alu_is_reg_c_i,
	halt_if_o,
	halt_id_o,
	misaligned_stall_o,
	jr_stall_o,
	load_stall_o,
	id_ready_i,
	ex_valid_i,
	wb_ready_i,
	perf_jump_o,
	perf_jr_stall_o,
	perf_ld_stall_o,
	perf_pipeline_stall_o
);
	reg _sv2v_0;
	parameter FPU = 0;
	input wire clk;
	input wire rst_n;
	input wire fetch_enable_i;
	output reg ctrl_busy_o;
	output reg first_fetch_o;
	output reg is_decoding_o;
	input wire is_fetch_failed_i;
	output reg deassert_we_o;
	input wire illegal_insn_i;
	input wire ecall_insn_i;
	input wire mret_insn_i;
	input wire uret_insn_i;
	input wire dret_insn_i;
	input wire mret_dec_i;
	input wire uret_dec_i;
	input wire dret_dec_i;
	input wire pipe_flush_i;
	input wire ebrk_insn_i;
	input wire fencei_insn_i;
	input wire csr_status_i;
	input wire instr_multicycle_i;
	output reg hwloop_mask_o;
	input wire instr_valid_i;
	output reg instr_req_o;
	output reg pc_set_o;
	output reg [2:0] pc_mux_o;
	output reg [2:0] exc_pc_mux_o;
	output reg [1:0] trap_addr_mux_o;
	input wire data_req_ex_i;
	input wire data_we_ex_i;
	input wire data_misaligned_i;
	input wire data_load_event_i;
	input wire data_err_i;
	output reg data_err_ack_o;
	input wire mult_multicycle_i;
	input wire apu_en_i;
	input wire apu_read_dep_i;
	input wire apu_write_dep_i;
	output wire apu_stall_o;
	input wire branch_taken_ex_i;
	input wire [1:0] jump_in_id_i;
	input wire [1:0] jump_in_dec_i;
	input wire irq_pending_i;
	input wire irq_req_ctrl_i;
	input wire irq_sec_ctrl_i;
	input wire [5:0] irq_id_ctrl_i;
	input wire m_IE_i;
	input wire u_IE_i;
	input wire [1:0] current_priv_lvl_i;
	output reg irq_ack_o;
	output reg [4:0] irq_id_o;
	output reg [5:0] exc_cause_o;
	output reg exc_ack_o;
	output reg exc_kill_o;
	output wire debug_mode_o;
	output reg [2:0] debug_cause_o;
	output reg debug_csr_save_o;
	input wire debug_req_i;
	input wire debug_single_step_i;
	input wire debug_ebreakm_i;
	input wire debug_ebreaku_i;
	output reg csr_save_if_o;
	output reg csr_save_id_o;
	output reg csr_save_ex_o;
	output reg [6:0] csr_cause_o;
	output reg csr_irq_sec_o;
	output reg csr_restore_mret_id_o;
	output reg csr_restore_uret_id_o;
	output reg csr_restore_dret_id_o;
	output reg csr_save_cause_o;
	input wire regfile_we_id_i;
	input wire [5:0] regfile_alu_waddr_id_i;
	input wire regfile_we_ex_i;
	input wire [5:0] regfile_waddr_ex_i;
	input wire regfile_we_wb_i;
	input wire regfile_alu_we_fw_i;
	output reg [1:0] operand_a_fw_mux_sel_o;
	output reg [1:0] operand_b_fw_mux_sel_o;
	output reg [1:0] operand_c_fw_mux_sel_o;
	input wire reg_d_ex_is_reg_a_i;
	input wire reg_d_ex_is_reg_b_i;
	input wire reg_d_ex_is_reg_c_i;
	input wire reg_d_wb_is_reg_a_i;
	input wire reg_d_wb_is_reg_b_i;
	input wire reg_d_wb_is_reg_c_i;
	input wire reg_d_alu_is_reg_a_i;
	input wire reg_d_alu_is_reg_b_i;
	input wire reg_d_alu_is_reg_c_i;
	output reg halt_if_o;
	output reg halt_id_o;
	output wire misaligned_stall_o;
	output reg jr_stall_o;
	output reg load_stall_o;
	input wire id_ready_i;
	input wire ex_valid_i;
	input wire wb_ready_i;
	output wire perf_jump_o;
	output wire perf_jr_stall_o;
	output wire perf_ld_stall_o;
	output reg perf_pipeline_stall_o;
	reg [4:0] ctrl_fsm_cs;
	reg [4:0] ctrl_fsm_ns;
	reg jump_done;
	reg jump_done_q;
	reg jump_in_dec;
	reg branch_in_id;
	reg boot_done;
	reg boot_done_q;
	reg irq_enable_int;
	reg data_err_q;
	reg debug_mode_q;
	reg debug_mode_n;
	reg ebrk_force_debug_mode;
	reg illegal_insn_q;
	reg illegal_insn_n;
	reg instr_valid_irq_flush_n;
	reg instr_valid_irq_flush_q;
	localparam riscv_defines_BRANCH_COND = 2'b11;
	localparam riscv_defines_BRANCH_JAL = 2'b01;
	localparam riscv_defines_BRANCH_JALR = 2'b10;
	localparam riscv_defines_DBG_CAUSE_EBREAK = 3'h1;
	localparam riscv_defines_DBG_CAUSE_HALTREQ = 3'h3;
	localparam riscv_defines_DBG_CAUSE_STEP = 3'h4;
	localparam riscv_defines_EXC_CAUSE_BREAKPOINT = 6'h03;
	localparam riscv_defines_EXC_CAUSE_ECALL_MMODE = 6'h0b;
	localparam riscv_defines_EXC_CAUSE_ECALL_UMODE = 6'h08;
	localparam riscv_defines_EXC_CAUSE_ILLEGAL_INSN = 6'h02;
	localparam riscv_defines_EXC_CAUSE_INSTR_FAULT = 6'h01;
	localparam riscv_defines_EXC_CAUSE_LOAD_FAULT = 6'h05;
	localparam riscv_defines_EXC_CAUSE_STORE_FAULT = 6'h07;
	localparam riscv_defines_EXC_PC_DBD = 3'b010;
	localparam riscv_defines_EXC_PC_EXCEPTION = 3'b000;
	localparam riscv_defines_EXC_PC_IRQ = 3'b001;
	localparam riscv_defines_PC_BOOT = 3'b000;
	localparam riscv_defines_PC_BRANCH = 3'b011;
	localparam riscv_defines_PC_DRET = 3'b111;
	localparam riscv_defines_PC_EXCEPTION = 3'b100;
	localparam riscv_defines_PC_FENCEI = 3'b001;
	localparam riscv_defines_PC_JUMP = 3'b010;
	localparam riscv_defines_PC_MRET = 3'b101;
	localparam riscv_defines_PC_URET = 3'b110;
	localparam riscv_defines_TRAP_MACHINE = 2'b00;
	localparam riscv_defines_TRAP_MACHINEX = 2'b10;
	localparam riscv_defines_TRAP_USER = 2'b01;
	always @(*) begin
		if (_sv2v_0)
			;
		instr_req_o = 1'b1;
		exc_ack_o = 1'b0;
		exc_kill_o = 1'b0;
		data_err_ack_o = 1'b0;
		csr_save_if_o = 1'b0;
		csr_save_id_o = 1'b0;
		csr_save_ex_o = 1'b0;
		csr_restore_mret_id_o = 1'b0;
		csr_restore_uret_id_o = 1'b0;
		csr_restore_dret_id_o = 1'b0;
		csr_save_cause_o = 1'b0;
		exc_cause_o = 1'sb0;
		exc_pc_mux_o = riscv_defines_EXC_PC_IRQ;
		trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
		csr_cause_o = 1'sb0;
		csr_irq_sec_o = 1'b0;
		pc_mux_o = riscv_defines_PC_BOOT;
		pc_set_o = 1'b0;
		jump_done = jump_done_q;
		ctrl_fsm_ns = ctrl_fsm_cs;
		ctrl_busy_o = 1'b1;
		first_fetch_o = 1'b0;
		halt_if_o = 1'b0;
		halt_id_o = 1'b0;
		irq_ack_o = 1'b0;
		irq_id_o = irq_id_ctrl_i[4:0];
		boot_done = 1'b0;
		jump_in_dec = (jump_in_dec_i == riscv_defines_BRANCH_JALR) || (jump_in_dec_i == riscv_defines_BRANCH_JAL);
		branch_in_id = jump_in_id_i == riscv_defines_BRANCH_COND;
		irq_enable_int = ((u_IE_i | irq_sec_ctrl_i) & (current_priv_lvl_i == 2'b00)) | (m_IE_i & (current_priv_lvl_i == 2'b11));
		ebrk_force_debug_mode = (debug_ebreakm_i && (current_priv_lvl_i == 2'b11)) || (debug_ebreaku_i && (current_priv_lvl_i == 2'b00));
		debug_csr_save_o = 1'b0;
		debug_cause_o = riscv_defines_DBG_CAUSE_EBREAK;
		debug_mode_n = debug_mode_q;
		illegal_insn_n = illegal_insn_q;
		perf_pipeline_stall_o = 1'b0;
		instr_valid_irq_flush_n = 1'b0;
		hwloop_mask_o = 1'b0;
		case (ctrl_fsm_cs)
			5'd0: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				if (fetch_enable_i == 1'b1)
					ctrl_fsm_ns = 5'd1;
			end
			5'd1: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b1;
				pc_mux_o = riscv_defines_PC_BOOT;
				pc_set_o = 1'b1;
				boot_done = 1'b1;
				ctrl_fsm_ns = 5'd4;
			end
			5'd3: begin
				is_decoding_o = 1'b0;
				ctrl_busy_o = 1'b0;
				instr_req_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				ctrl_fsm_ns = 5'd2;
			end
			5'd2: begin
				is_decoding_o = 1'b0;
				ctrl_busy_o = 1'b0;
				instr_req_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (irq_pending_i || ((debug_req_i || debug_mode_q) || debug_single_step_i))
					ctrl_fsm_ns = 5'd4;
			end
			5'd4: begin
				is_decoding_o = 1'b0;
				first_fetch_o = 1'b1;
				if (id_ready_i == 1'b1)
					ctrl_fsm_ns = 5'd5;
				if (irq_req_ctrl_i & irq_enable_int) begin
					ctrl_fsm_ns = 5'd7;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
				end
				if (debug_req_i & ~debug_mode_q) begin
					ctrl_fsm_ns = 5'd15;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
				end
			end
			5'd5:
				if (branch_taken_ex_i) begin
					is_decoding_o = 1'b0;
					pc_mux_o = riscv_defines_PC_BRANCH;
					pc_set_o = 1'b1;
				end
				else if (data_err_i) begin
					is_decoding_o = 1'b0;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = (data_we_ex_i ? riscv_defines_EXC_CAUSE_STORE_FAULT : riscv_defines_EXC_CAUSE_LOAD_FAULT);
					ctrl_fsm_ns = 5'd12;
				end
				else if (is_fetch_failed_i) begin
					is_decoding_o = 1'b0;
					halt_id_o = 1'b1;
					halt_if_o = 1'b1;
					csr_save_if_o = 1'b1;
					csr_save_cause_o = 1'b1;
					csr_cause_o = riscv_defines_EXC_CAUSE_INSTR_FAULT;
					ctrl_fsm_ns = 5'd12;
				end
				else if (instr_valid_i || instr_valid_irq_flush_q) begin
					is_decoding_o = 1'b1;
					case (1'b1)
						((irq_req_ctrl_i & irq_enable_int) & ~debug_req_i) & ~debug_mode_q: begin
							halt_if_o = 1'b1;
							halt_id_o = 1'b1;
							ctrl_fsm_ns = 5'd8;
							hwloop_mask_o = 1'b1;
						end
						debug_req_i & ~debug_mode_q: begin
							halt_if_o = 1'b1;
							halt_id_o = 1'b1;
							ctrl_fsm_ns = 5'd16;
						end
						default: begin
							exc_kill_o = (irq_req_ctrl_i ? 1'b1 : 1'b0);
							if (illegal_insn_i) begin
								halt_if_o = 1'b1;
								halt_id_o = 1'b1;
								csr_save_id_o = 1'b1;
								csr_save_cause_o = 1'b1;
								csr_cause_o = riscv_defines_EXC_CAUSE_ILLEGAL_INSN;
								ctrl_fsm_ns = 5'd11;
								illegal_insn_n = 1'b1;
							end
							else
								case (1'b1)
									jump_in_dec: begin
										pc_mux_o = riscv_defines_PC_JUMP;
										if (~jr_stall_o && ~jump_done_q) begin
											pc_set_o = 1'b1;
											jump_done = 1'b1;
										end
									end
									ebrk_insn_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b1;
										if (debug_mode_q)
											ctrl_fsm_ns = 5'd16;
										else if (ebrk_force_debug_mode)
											ctrl_fsm_ns = 5'd16;
										else begin
											csr_save_id_o = 1'b1;
											csr_save_cause_o = 1'b1;
											ctrl_fsm_ns = 5'd11;
											csr_cause_o = riscv_defines_EXC_CAUSE_BREAKPOINT;
										end
									end
									pipe_flush_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b1;
										ctrl_fsm_ns = 5'd11;
									end
									ecall_insn_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b1;
										csr_save_id_o = 1'b1;
										csr_save_cause_o = 1'b1;
										csr_cause_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_EXC_CAUSE_ECALL_UMODE : riscv_defines_EXC_CAUSE_ECALL_MMODE);
										ctrl_fsm_ns = 5'd11;
									end
									fencei_insn_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b1;
										ctrl_fsm_ns = 5'd11;
									end
									(mret_insn_i | uret_insn_i) | dret_insn_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b1;
										ctrl_fsm_ns = 5'd11;
									end
									csr_status_i: begin
										halt_if_o = 1'b1;
										ctrl_fsm_ns = (id_ready_i ? 5'd11 : 5'd5);
									end
									data_load_event_i: begin
										ctrl_fsm_ns = (id_ready_i ? 5'd10 : 5'd5);
										halt_if_o = 1'b1;
									end
									default:
										;
								endcase
							if (debug_single_step_i & ~debug_mode_q) begin
								halt_if_o = 1'b1;
								if (id_ready_i)
									case (1'b1)
										illegal_insn_i | ecall_insn_i: ctrl_fsm_ns = 5'd11;
										~ebrk_force_debug_mode & ebrk_insn_i: ctrl_fsm_ns = 5'd11;
										mret_insn_i | uret_insn_i: ctrl_fsm_ns = 5'd11;
										branch_in_id: ctrl_fsm_ns = 5'd17;
										default: ctrl_fsm_ns = 5'd16;
									endcase
							end
						end
					endcase
				end
				else begin
					is_decoding_o = 1'b0;
					perf_pipeline_stall_o = data_load_event_i;
				end
			5'd11: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (data_err_i) begin
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = (data_we_ex_i ? riscv_defines_EXC_CAUSE_STORE_FAULT : riscv_defines_EXC_CAUSE_LOAD_FAULT);
					ctrl_fsm_ns = 5'd12;
					illegal_insn_n = 1'b0;
				end
				else if (ex_valid_i)
					ctrl_fsm_ns = 5'd12;
			end
			5'd8: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (data_err_i) begin
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = (data_we_ex_i ? riscv_defines_EXC_CAUSE_STORE_FAULT : riscv_defines_EXC_CAUSE_LOAD_FAULT);
					ctrl_fsm_ns = 5'd12;
				end
				else if (irq_pending_i & irq_enable_int)
					ctrl_fsm_ns = 5'd6;
				else begin
					exc_kill_o = 1'b1;
					instr_valid_irq_flush_n = 1'b1;
					ctrl_fsm_ns = 5'd5;
				end
			end
			5'd9: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				perf_pipeline_stall_o = data_load_event_i;
				if (irq_pending_i & irq_enable_int)
					ctrl_fsm_ns = 5'd6;
				else begin
					exc_kill_o = 1'b1;
					ctrl_fsm_ns = 5'd5;
				end
			end
			5'd10: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (id_ready_i)
					ctrl_fsm_ns = (debug_req_i & ~debug_mode_q ? 5'd16 : 5'd9);
				else
					ctrl_fsm_ns = 5'd10;
				perf_pipeline_stall_o = data_load_event_i;
			end
			5'd6: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = riscv_defines_PC_EXCEPTION;
				exc_pc_mux_o = riscv_defines_EXC_PC_IRQ;
				exc_cause_o = {1'b0, irq_id_ctrl_i[4:0]};
				csr_irq_sec_o = irq_sec_ctrl_i;
				if (irq_id_ctrl_i[5]) begin
					irq_ack_o = 1'b1;
					if (irq_sec_ctrl_i)
						trap_addr_mux_o = riscv_defines_TRAP_MACHINEX;
					else
						trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_TRAP_USER : riscv_defines_TRAP_MACHINEX);
				end
				else if (irq_sec_ctrl_i)
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
				else
					trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_TRAP_USER : riscv_defines_TRAP_MACHINE);
				csr_save_cause_o = 1'b1;
				csr_cause_o = {1'b1, irq_id_ctrl_i};
				csr_save_id_o = 1'b1;
				exc_ack_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
			end
			5'd7: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = riscv_defines_PC_EXCEPTION;
				exc_pc_mux_o = riscv_defines_EXC_PC_IRQ;
				exc_cause_o = {1'b0, irq_id_ctrl_i[4:0]};
				csr_irq_sec_o = irq_sec_ctrl_i;
				if (irq_id_ctrl_i[5]) begin
					irq_ack_o = 1'b1;
					if (irq_sec_ctrl_i)
						trap_addr_mux_o = riscv_defines_TRAP_MACHINEX;
					else
						trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_TRAP_USER : riscv_defines_TRAP_MACHINEX);
				end
				else if (irq_sec_ctrl_i)
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
				else
					trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? riscv_defines_TRAP_USER : riscv_defines_TRAP_MACHINE);
				csr_save_cause_o = 1'b1;
				csr_cause_o = {1'b1, irq_id_ctrl_i};
				csr_save_if_o = 1'b1;
				exc_ack_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
			end
			5'd12: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
				if (data_err_q) begin
					pc_mux_o = riscv_defines_PC_EXCEPTION;
					pc_set_o = 1'b1;
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
					exc_pc_mux_o = riscv_defines_EXC_PC_EXCEPTION;
					exc_cause_o = (data_we_ex_i ? riscv_defines_EXC_CAUSE_LOAD_FAULT : riscv_defines_EXC_CAUSE_STORE_FAULT);
				end
				else if (is_fetch_failed_i) begin
					pc_mux_o = riscv_defines_PC_EXCEPTION;
					pc_set_o = 1'b1;
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
					exc_pc_mux_o = riscv_defines_EXC_PC_EXCEPTION;
					exc_cause_o = riscv_defines_EXC_CAUSE_INSTR_FAULT;
				end
				else if (illegal_insn_q) begin
					pc_mux_o = riscv_defines_PC_EXCEPTION;
					pc_set_o = 1'b1;
					trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
					exc_pc_mux_o = riscv_defines_EXC_PC_EXCEPTION;
					illegal_insn_n = 1'b0;
					if (debug_single_step_i && ~debug_mode_q)
						ctrl_fsm_ns = 5'd15;
				end
				else
					case (1'b1)
						ebrk_insn_i: begin
							pc_mux_o = riscv_defines_PC_EXCEPTION;
							pc_set_o = 1'b1;
							trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
							exc_pc_mux_o = riscv_defines_EXC_PC_EXCEPTION;
							if (debug_single_step_i && ~debug_mode_q)
								ctrl_fsm_ns = 5'd15;
						end
						ecall_insn_i: begin
							pc_mux_o = riscv_defines_PC_EXCEPTION;
							pc_set_o = 1'b1;
							trap_addr_mux_o = riscv_defines_TRAP_MACHINE;
							exc_pc_mux_o = riscv_defines_EXC_PC_EXCEPTION;
							exc_cause_o = riscv_defines_EXC_CAUSE_ECALL_MMODE;
							if (debug_single_step_i && ~debug_mode_q)
								ctrl_fsm_ns = 5'd15;
						end
						mret_insn_i: begin
							csr_restore_mret_id_o = 1'b1;
							ctrl_fsm_ns = 5'd13;
						end
						uret_insn_i: begin
							csr_restore_uret_id_o = 1'b1;
							ctrl_fsm_ns = 5'd13;
						end
						dret_insn_i: begin
							csr_restore_dret_id_o = 1'b1;
							ctrl_fsm_ns = 5'd13;
						end
						csr_status_i:
							;
						pipe_flush_i: ctrl_fsm_ns = 5'd3;
						fencei_insn_i: begin
							pc_mux_o = riscv_defines_PC_FENCEI;
							pc_set_o = 1'b1;
						end
						default:
							;
					endcase
			end
			5'd13: begin
				is_decoding_o = 1'b0;
				ctrl_fsm_ns = 5'd5;
				case (1'b1)
					mret_dec_i: begin
						pc_mux_o = riscv_defines_PC_MRET;
						pc_set_o = 1'b1;
					end
					uret_dec_i: begin
						pc_mux_o = riscv_defines_PC_URET;
						pc_set_o = 1'b1;
					end
					dret_dec_i: begin
						pc_mux_o = riscv_defines_PC_DRET;
						pc_set_o = 1'b1;
						debug_mode_n = 1'b0;
					end
					default:
						;
				endcase
				if (debug_single_step_i && ~debug_mode_q)
					ctrl_fsm_ns = 5'd15;
			end
			5'd17: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				if (branch_taken_ex_i) begin
					pc_mux_o = riscv_defines_PC_BRANCH;
					pc_set_o = 1'b1;
				end
				ctrl_fsm_ns = 5'd16;
			end
			5'd14: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = riscv_defines_PC_EXCEPTION;
				exc_pc_mux_o = riscv_defines_EXC_PC_DBD;
				if ((debug_req_i && ~debug_mode_q) || ((ebrk_insn_i && ebrk_force_debug_mode) && ~debug_mode_q)) begin
					csr_save_cause_o = 1'b1;
					csr_save_id_o = 1'b1;
					debug_csr_save_o = 1'b1;
					if (debug_req_i)
						debug_cause_o = riscv_defines_DBG_CAUSE_HALTREQ;
					if (ebrk_insn_i)
						debug_cause_o = riscv_defines_DBG_CAUSE_EBREAK;
				end
				ctrl_fsm_ns = 5'd5;
				debug_mode_n = 1'b1;
			end
			5'd15: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = riscv_defines_PC_EXCEPTION;
				exc_pc_mux_o = riscv_defines_EXC_PC_DBD;
				csr_save_cause_o = 1'b1;
				debug_csr_save_o = 1'b1;
				if (debug_single_step_i)
					debug_cause_o = riscv_defines_DBG_CAUSE_STEP;
				if (debug_req_i)
					debug_cause_o = riscv_defines_DBG_CAUSE_HALTREQ;
				if (ebrk_insn_i)
					debug_cause_o = riscv_defines_DBG_CAUSE_EBREAK;
				csr_save_if_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
				debug_mode_n = 1'b1;
			end
			5'd16: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				perf_pipeline_stall_o = data_load_event_i;
				if (data_err_i) begin
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = (data_we_ex_i ? riscv_defines_EXC_CAUSE_STORE_FAULT : riscv_defines_EXC_CAUSE_LOAD_FAULT);
					ctrl_fsm_ns = 5'd12;
				end
				else if (debug_mode_q)
					ctrl_fsm_ns = 5'd14;
				else if (data_load_event_i)
					ctrl_fsm_ns = 5'd14;
				else if (debug_single_step_i)
					ctrl_fsm_ns = 5'd15;
				else
					ctrl_fsm_ns = 5'd14;
			end
			default: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				ctrl_fsm_ns = 5'd0;
			end
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		load_stall_o = 1'b0;
		jr_stall_o = 1'b0;
		deassert_we_o = 1'b0;
		if (~is_decoding_o)
			deassert_we_o = 1'b1;
		if (illegal_insn_i)
			deassert_we_o = 1'b1;
		if ((((data_req_ex_i == 1'b1) && (regfile_we_ex_i == 1'b1)) || ((wb_ready_i == 1'b0) && (regfile_we_wb_i == 1'b1))) && ((((reg_d_ex_is_reg_a_i == 1'b1) || (reg_d_ex_is_reg_b_i == 1'b1)) || (reg_d_ex_is_reg_c_i == 1'b1)) || ((is_decoding_o && regfile_we_id_i) && (regfile_waddr_ex_i == regfile_alu_waddr_id_i)))) begin
			deassert_we_o = 1'b1;
			load_stall_o = 1'b1;
		end
		if ((jump_in_dec_i == riscv_defines_BRANCH_JALR) && ((((regfile_we_wb_i == 1'b1) && (reg_d_wb_is_reg_a_i == 1'b1)) || ((regfile_we_ex_i == 1'b1) && (reg_d_ex_is_reg_a_i == 1'b1))) || ((regfile_alu_we_fw_i == 1'b1) && (reg_d_alu_is_reg_a_i == 1'b1)))) begin
			jr_stall_o = 1'b1;
			deassert_we_o = 1'b1;
		end
	end
	assign misaligned_stall_o = data_misaligned_i;
	assign apu_stall_o = apu_read_dep_i | (apu_write_dep_i & ~apu_en_i);
	localparam riscv_defines_SEL_FW_EX = 2'b01;
	localparam riscv_defines_SEL_FW_WB = 2'b10;
	localparam riscv_defines_SEL_REGFILE = 2'b00;
	always @(*) begin
		if (_sv2v_0)
			;
		operand_a_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		operand_b_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		operand_c_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		if (regfile_we_wb_i == 1'b1) begin
			if (reg_d_wb_is_reg_a_i == 1'b1)
				operand_a_fw_mux_sel_o = riscv_defines_SEL_FW_WB;
			if (reg_d_wb_is_reg_b_i == 1'b1)
				operand_b_fw_mux_sel_o = riscv_defines_SEL_FW_WB;
			if (reg_d_wb_is_reg_c_i == 1'b1)
				operand_c_fw_mux_sel_o = riscv_defines_SEL_FW_WB;
		end
		if (regfile_alu_we_fw_i == 1'b1) begin
			if (reg_d_alu_is_reg_a_i == 1'b1)
				operand_a_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
			if (reg_d_alu_is_reg_b_i == 1'b1)
				operand_b_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
			if (reg_d_alu_is_reg_c_i == 1'b1)
				operand_c_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
		end
		if (data_misaligned_i) begin
			operand_a_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
			operand_b_fw_mux_sel_o = riscv_defines_SEL_REGFILE;
		end
		else if (mult_multicycle_i)
			operand_c_fw_mux_sel_o = riscv_defines_SEL_FW_EX;
	end
	always @(posedge clk or negedge rst_n) begin : UPDATE_REGS
		if (rst_n == 1'b0) begin
			ctrl_fsm_cs <= 5'd0;
			jump_done_q <= 1'b0;
			boot_done_q <= 1'b0;
			data_err_q <= 1'b0;
			debug_mode_q <= 1'b0;
			illegal_insn_q <= 1'b0;
			instr_valid_irq_flush_q <= 1'b0;
		end
		else begin
			ctrl_fsm_cs <= ctrl_fsm_ns;
			boot_done_q <= boot_done | (~boot_done & boot_done_q);
			jump_done_q <= jump_done & ~id_ready_i;
			data_err_q <= data_err_i;
			debug_mode_q <= debug_mode_n;
			illegal_insn_q <= illegal_insn_n;
			instr_valid_irq_flush_q <= instr_valid_irq_flush_n;
		end
	end
	assign perf_jump_o = (jump_in_id_i == riscv_defines_BRANCH_JAL) || (jump_in_id_i == riscv_defines_BRANCH_JALR);
	assign perf_jr_stall_o = jr_stall_o;
	assign perf_ld_stall_o = load_stall_o;
	assign debug_mode_o = debug_mode_q;
	initial _sv2v_0 = 0;
endmodule
module riscv_core (
	clk_i,
	rst_ni,
	clock_en_i,
	test_en_i,
	fregfile_disable_i,
	boot_addr_i,
	core_id_i,
	cluster_id_i,
	instr_req_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_addr_o,
	instr_rdata_i,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_we_o,
	data_be_o,
	data_addr_o,
	data_wdata_o,
	data_rdata_i,
	data_atop_o,
	apu_master_req_o,
	apu_master_ready_o,
	apu_master_gnt_i,
	apu_master_operands_o,
	apu_master_op_o,
	apu_master_type_o,
	apu_master_flags_o,
	apu_master_valid_i,
	apu_master_result_i,
	apu_master_flags_i,
	irq_ack_o,
	irq_id_o,
	irq_sec_i,
	irq_software_i,
	irq_timer_i,
	irq_external_i,
	irq_fast_i,
	irq_nmi_i,
	irq_fastx_i,
	sec_lvl_o,
	debug_req_i,
	ivalid_o,
	iexception_o,
	interrupt_o,
	cause_o,
	tval_o,
	priv_o,
	iaddr_o,
	instr_o,
	compressed_o,
	fetch_enable_i,
	core_busy_o,
	ext_perf_counters_i
);
	parameter N_EXT_PERF_COUNTERS = 0;
	parameter INSTR_RDATA_WIDTH = 32;
	parameter PULP_SECURE = 0;
	parameter N_PMP_ENTRIES = 16;
	parameter USE_PMP = 1;
	parameter PULP_CLUSTER = 1;
	parameter A_EXTENSION = 0;
	parameter FPU = 0;
	parameter Zfinx = 0;
	parameter FP_DIVSQRT = 1;
	parameter SHARED_FP = 0;
	parameter SHARED_DSP_MULT = 0;
	parameter SHARED_INT_MULT = 0;
	parameter SHARED_INT_DIV = 0;
	parameter SHARED_FP_DIVSQRT = 0;
	parameter WAPUTYPE = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	parameter DM_HaltAddress = 32'h1a110800;
	input wire clk_i;
	input wire rst_ni;
	input wire clock_en_i;
	input wire test_en_i;
	input wire fregfile_disable_i;
	input wire [31:0] boot_addr_i;
	input wire [3:0] core_id_i;
	input wire [5:0] cluster_id_i;
	output wire instr_req_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	output wire [31:0] instr_addr_o;
	input wire [INSTR_RDATA_WIDTH - 1:0] instr_rdata_i;
	output wire data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_addr_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	output wire [5:0] data_atop_o;
	output wire apu_master_req_o;
	output wire apu_master_ready_o;
	input wire apu_master_gnt_i;
	output wire [(APU_NARGS_CPU * 32) - 1:0] apu_master_operands_o;
	output wire [APU_WOP_CPU - 1:0] apu_master_op_o;
	output wire [WAPUTYPE - 1:0] apu_master_type_o;
	output wire [APU_NDSFLAGS_CPU - 1:0] apu_master_flags_o;
	input wire apu_master_valid_i;
	input wire [31:0] apu_master_result_i;
	input wire [APU_NUSFLAGS_CPU - 1:0] apu_master_flags_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	input wire irq_sec_i;
	input wire irq_software_i;
	input wire irq_timer_i;
	input wire irq_external_i;
	input wire [14:0] irq_fast_i;
	input wire irq_nmi_i;
	input wire [31:0] irq_fastx_i;
	output wire sec_lvl_o;
	input wire debug_req_i;
	output wire ivalid_o;
	output wire iexception_o;
	output wire interrupt_o;
	output wire [4:0] cause_o;
	output wire [31:0] tval_o;
	output wire [2:0] priv_o;
	output wire [31:0] iaddr_o;
	output wire [31:0] instr_o;
	output wire compressed_o;
	input wire fetch_enable_i;
	output wire core_busy_o;
	input wire [N_EXT_PERF_COUNTERS - 1:0] ext_perf_counters_i;
	localparam N_HWLP = 2;
	localparam N_HWLP_BITS = 1;
	localparam APU = (((SHARED_DSP_MULT == 1) | (SHARED_INT_DIV == 1)) | (FPU == 1) ? 1 : 0);
	wire is_hwlp_id;
	wire [1:0] hwlp_dec_cnt_id;
	wire instr_valid_id;
	wire [31:0] instr_rdata_id;
	wire is_compressed_id;
	wire is_fetch_failed_id;
	wire illegal_c_insn_id;
	wire [31:0] pc_if;
	wire [31:0] pc_id;
	wire clear_instr_valid;
	wire pc_set;
	wire [2:0] pc_mux_id;
	wire [2:0] exc_pc_mux_id;
	wire [5:0] exc_cause;
	wire [1:0] trap_addr_mux;
	wire lsu_load_err;
	wire lsu_store_err;
	wire is_decoding;
	wire useincr_addr_ex;
	wire data_misaligned;
	wire mult_multicycle;
	wire [31:0] jump_target_id;
	wire [31:0] jump_target_ex;
	wire branch_in_ex;
	wire branch_decision;
	wire ctrl_busy;
	wire if_busy;
	wire lsu_busy;
	wire apu_busy;
	wire [31:0] pc_ex;
	wire alu_en_ex;
	localparam riscv_defines_ALU_OP_WIDTH = 7;
	wire [6:0] alu_operator_ex;
	wire [31:0] alu_operand_a_ex;
	wire [31:0] alu_operand_b_ex;
	wire [31:0] alu_operand_c_ex;
	wire [4:0] bmask_a_ex;
	wire [4:0] bmask_b_ex;
	wire [1:0] imm_vec_ext_ex;
	wire [1:0] alu_vec_mode_ex;
	wire alu_is_clpx_ex;
	wire alu_is_subrot_ex;
	wire [1:0] alu_clpx_shift_ex;
	wire [2:0] mult_operator_ex;
	wire [31:0] mult_operand_a_ex;
	wire [31:0] mult_operand_b_ex;
	wire [31:0] mult_operand_c_ex;
	wire mult_en_ex;
	wire mult_sel_subword_ex;
	wire [1:0] mult_signed_mode_ex;
	wire [4:0] mult_imm_ex;
	wire [31:0] mult_dot_op_a_ex;
	wire [31:0] mult_dot_op_b_ex;
	wire [31:0] mult_dot_op_c_ex;
	wire [1:0] mult_dot_signed_ex;
	wire mult_is_clpx_ex_o;
	wire [1:0] mult_clpx_shift_ex;
	wire mult_clpx_img_ex;
	localparam riscv_defines_C_PC = 5;
	wire [4:0] fprec_csr;
	localparam riscv_defines_C_RM = 3;
	wire [2:0] frm_csr;
	localparam riscv_defines_C_FFLAG = 5;
	wire [4:0] fflags;
	wire [4:0] fflags_csr;
	wire fflags_we;
	wire apu_en_ex;
	wire [WAPUTYPE - 1:0] apu_type_ex;
	wire [APU_NDSFLAGS_CPU - 1:0] apu_flags_ex;
	wire [APU_WOP_CPU - 1:0] apu_op_ex;
	wire [1:0] apu_lat_ex;
	wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands_ex;
	wire [5:0] apu_waddr_ex;
	wire [17:0] apu_read_regs;
	wire [2:0] apu_read_regs_valid;
	wire apu_read_dep;
	wire [11:0] apu_write_regs;
	wire [1:0] apu_write_regs_valid;
	wire apu_write_dep;
	wire perf_apu_type;
	wire perf_apu_cont;
	wire perf_apu_dep;
	wire perf_apu_wb;
	wire [5:0] regfile_waddr_ex;
	wire regfile_we_ex;
	wire [5:0] regfile_waddr_fw_wb_o;
	wire regfile_we_wb;
	wire [31:0] regfile_wdata;
	wire [5:0] regfile_alu_waddr_ex;
	wire regfile_alu_we_ex;
	wire [5:0] regfile_alu_waddr_fw;
	wire regfile_alu_we_fw;
	wire [31:0] regfile_alu_wdata_fw;
	wire csr_access_ex;
	wire [1:0] csr_op_ex;
	wire [23:0] mtvec;
	wire [23:0] mtvecx;
	wire [23:0] utvec;
	wire csr_access;
	wire [1:0] csr_op;
	wire [11:0] csr_addr;
	wire [11:0] csr_addr_int;
	wire [31:0] csr_rdata;
	wire [31:0] csr_wdata;
	wire [1:0] current_priv_lvl;
	wire data_we_ex;
	wire [5:0] data_atop_ex;
	wire [1:0] data_type_ex;
	wire [1:0] data_sign_ext_ex;
	wire [1:0] data_reg_offset_ex;
	wire data_req_ex;
	wire data_load_event_ex;
	wire data_misaligned_ex;
	wire [31:0] lsu_rdata;
	wire halt_if;
	wire id_ready;
	wire ex_ready;
	wire id_valid;
	wire ex_valid;
	wire wb_valid;
	wire lsu_ready_ex;
	wire lsu_ready_wb;
	wire apu_ready_wb;
	wire instr_req_int;
	wire m_irq_enable;
	wire u_irq_enable;
	wire csr_irq_sec;
	wire [31:0] mepc;
	wire [31:0] uepc;
	wire [31:0] depc;
	wire irq_software;
	wire irq_timer;
	wire irq_external;
	wire [14:0] irq_fast;
	wire irq_nmi;
	wire csr_save_cause;
	wire csr_save_if;
	wire csr_save_id;
	wire csr_save_ex;
	wire [6:0] csr_cause;
	wire csr_restore_mret_id;
	wire csr_restore_uret_id;
	wire csr_restore_dret_id;
	wire debug_mode;
	wire [2:0] debug_cause;
	wire debug_csr_save;
	wire debug_single_step;
	wire debug_ebreakm;
	wire debug_ebreaku;
	wire [63:0] hwlp_start;
	wire [63:0] hwlp_end;
	wire [63:0] hwlp_cnt;
	wire [0:0] csr_hwlp_regid;
	wire [2:0] csr_hwlp_we;
	wire [31:0] csr_hwlp_data;
	wire perf_imiss;
	wire perf_jump;
	wire perf_jr_stall;
	wire perf_ld_stall;
	wire perf_pipeline_stall;
	wire core_ctrl_firstfetch;
	wire core_busy_int;
	reg core_busy_q;
	wire [(N_PMP_ENTRIES * 32) - 1:0] pmp_addr;
	wire [(N_PMP_ENTRIES * 8) - 1:0] pmp_cfg;
	wire data_req_pmp;
	wire [31:0] data_addr_pmp;
	wire data_we_pmp;
	wire data_gnt_pmp;
	wire data_err_pmp;
	wire data_err_ack;
	wire instr_req_pmp;
	wire instr_gnt_pmp;
	wire [31:0] instr_addr_pmp;
	wire instr_err_pmp;
	wire irq_pending;
	wire [5:0] irq_id;
	wire illegal_insn;
	wire ebrk_insn;
	wire mret_insn;
	wire uret_insn;
	wire ecall_insn;
	wire pipe_flush;
	wire is_interrupt;
	localparam riscv_defines_EXC_PC_IRQ = 3'b001;
	localparam riscv_defines_PC_EXCEPTION = 3'b100;
	assign is_interrupt = (pc_mux_id == riscv_defines_PC_EXCEPTION) && (exc_pc_mux_id == riscv_defines_EXC_PC_IRQ);
	generate
		if (SHARED_FP) begin : genblk1
			assign apu_master_type_o = apu_type_ex;
			assign apu_master_flags_o = apu_flags_ex;
			assign fflags_csr = apu_master_flags_i;
		end
		else begin : genblk1
			assign apu_master_type_o = 1'sb0;
			assign apu_master_flags_o = 1'sb0;
			assign fflags_csr = fflags;
		end
	endgenerate
	wire clk;
	wire clock_en;
	wire sleeping;
	assign core_busy_o = (core_ctrl_firstfetch ? 1'b1 : core_busy_q);
	assign core_busy_int = ((PULP_CLUSTER & data_load_event_ex) & data_req_o ? if_busy | apu_busy : ((if_busy | ctrl_busy) | lsu_busy) | apu_busy);
	assign clock_en = (PULP_CLUSTER ? clock_en_i | core_busy_o : (irq_pending | debug_req_i) | core_busy_o);
	assign sleeping = ~core_busy_o;
	always @(posedge clk_i or negedge rst_ni)
		if (rst_ni == 1'b0)
			core_busy_q <= 1'b0;
		else
			core_busy_q <= core_busy_int;
	cv32e40p_clock_gate core_clock_gate_i(
		.clk_i(clk_i),
		.en_i(clock_en),
		.test_en_i(test_en_i),
		.clk_o(clk)
	);
	riscv_if_stage #(
		.N_HWLP(N_HWLP),
		.RDATA_WIDTH(INSTR_RDATA_WIDTH),
		.FPU(FPU),
		.DM_HaltAddress(DM_HaltAddress)
	) if_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.boot_addr_i(boot_addr_i[31:1]),
		.m_trap_base_addr_i(mtvec),
		.m_trap_base_addrx_i(mtvecx),
		.u_trap_base_addr_i(utvec),
		.trap_addr_mux_i(trap_addr_mux),
		.req_i(instr_req_int),
		.instr_req_o(instr_req_pmp),
		.instr_addr_o(instr_addr_pmp),
		.instr_gnt_i(instr_gnt_pmp),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_rdata_i(instr_rdata_i),
		.instr_err_pmp_i(instr_err_pmp),
		.hwlp_dec_cnt_id_o(hwlp_dec_cnt_id),
		.is_hwlp_id_o(is_hwlp_id),
		.instr_valid_id_o(instr_valid_id),
		.instr_rdata_id_o(instr_rdata_id),
		.is_compressed_id_o(is_compressed_id),
		.illegal_c_insn_id_o(illegal_c_insn_id),
		.pc_if_o(pc_if),
		.pc_id_o(pc_id),
		.is_fetch_failed_o(is_fetch_failed_id),
		.clear_instr_valid_i(clear_instr_valid),
		.pc_set_i(pc_set),
		.mepc_i(mepc),
		.uepc_i(uepc),
		.depc_i(depc),
		.pc_mux_i(pc_mux_id),
		.exc_pc_mux_i(exc_pc_mux_id),
		.exc_vec_pc_mux_i(exc_cause[4:0]),
		.hwlp_start_i(hwlp_start),
		.hwlp_end_i(hwlp_end),
		.hwlp_cnt_i(hwlp_cnt),
		.jump_target_id_i(jump_target_id),
		.jump_target_ex_i(jump_target_ex),
		.halt_if_i(halt_if),
		.id_ready_i(id_ready),
		.if_busy_o(if_busy),
		.perf_imiss_o(perf_imiss)
	);
	wire mult_is_clpx_ex;
	riscv_id_stage #(
		.N_HWLP(N_HWLP),
		.PULP_SECURE(PULP_SECURE),
		.A_EXTENSION(A_EXTENSION),
		.APU(APU),
		.FPU(FPU),
		.Zfinx(Zfinx),
		.FP_DIVSQRT(FP_DIVSQRT),
		.SHARED_FP(SHARED_FP),
		.SHARED_DSP_MULT(SHARED_DSP_MULT),
		.SHARED_INT_MULT(SHARED_INT_MULT),
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.SHARED_FP_DIVSQRT(SHARED_FP_DIVSQRT),
		.WAPUTYPE(WAPUTYPE),
		.APU_NARGS_CPU(APU_NARGS_CPU),
		.APU_WOP_CPU(APU_WOP_CPU),
		.APU_NDSFLAGS_CPU(APU_NDSFLAGS_CPU),
		.APU_NUSFLAGS_CPU(APU_NUSFLAGS_CPU)
	) id_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.test_en_i(test_en_i),
		.fregfile_disable_i(fregfile_disable_i),
		.fetch_enable_i(fetch_enable_i),
		.ctrl_busy_o(ctrl_busy),
		.core_ctrl_firstfetch_o(core_ctrl_firstfetch),
		.is_decoding_o(is_decoding),
		.hwlp_dec_cnt_i(hwlp_dec_cnt_id),
		.is_hwlp_i(is_hwlp_id),
		.instr_valid_i(instr_valid_id),
		.instr_rdata_i(instr_rdata_id),
		.instr_req_o(instr_req_int),
		.branch_in_ex_o(branch_in_ex),
		.branch_decision_i(branch_decision),
		.jump_target_o(jump_target_id),
		.clear_instr_valid_o(clear_instr_valid),
		.pc_set_o(pc_set),
		.pc_mux_o(pc_mux_id),
		.exc_pc_mux_o(exc_pc_mux_id),
		.exc_cause_o(exc_cause),
		.trap_addr_mux_o(trap_addr_mux),
		.illegal_c_insn_i(illegal_c_insn_id),
		.is_compressed_i(is_compressed_id),
		.is_fetch_failed_i(is_fetch_failed_id),
		.pc_if_i(pc_if),
		.pc_id_i(pc_id),
		.halt_if_o(halt_if),
		.id_ready_o(id_ready),
		.ex_ready_i(ex_ready),
		.wb_ready_i(lsu_ready_wb),
		.id_valid_o(id_valid),
		.ex_valid_i(ex_valid),
		.pc_ex_o(pc_ex),
		.alu_en_ex_o(alu_en_ex),
		.alu_operator_ex_o(alu_operator_ex),
		.alu_operand_a_ex_o(alu_operand_a_ex),
		.alu_operand_b_ex_o(alu_operand_b_ex),
		.alu_operand_c_ex_o(alu_operand_c_ex),
		.bmask_a_ex_o(bmask_a_ex),
		.bmask_b_ex_o(bmask_b_ex),
		.imm_vec_ext_ex_o(imm_vec_ext_ex),
		.alu_vec_mode_ex_o(alu_vec_mode_ex),
		.alu_is_clpx_ex_o(alu_is_clpx_ex),
		.alu_is_subrot_ex_o(alu_is_subrot_ex),
		.alu_clpx_shift_ex_o(alu_clpx_shift_ex),
		.regfile_waddr_ex_o(regfile_waddr_ex),
		.regfile_we_ex_o(regfile_we_ex),
		.regfile_alu_we_ex_o(regfile_alu_we_ex),
		.regfile_alu_waddr_ex_o(regfile_alu_waddr_ex),
		.mult_operator_ex_o(mult_operator_ex),
		.mult_en_ex_o(mult_en_ex),
		.mult_sel_subword_ex_o(mult_sel_subword_ex),
		.mult_signed_mode_ex_o(mult_signed_mode_ex),
		.mult_operand_a_ex_o(mult_operand_a_ex),
		.mult_operand_b_ex_o(mult_operand_b_ex),
		.mult_operand_c_ex_o(mult_operand_c_ex),
		.mult_imm_ex_o(mult_imm_ex),
		.mult_dot_op_a_ex_o(mult_dot_op_a_ex),
		.mult_dot_op_b_ex_o(mult_dot_op_b_ex),
		.mult_dot_op_c_ex_o(mult_dot_op_c_ex),
		.mult_dot_signed_ex_o(mult_dot_signed_ex),
		.mult_is_clpx_ex_o(mult_is_clpx_ex),
		.mult_clpx_shift_ex_o(mult_clpx_shift_ex),
		.mult_clpx_img_ex_o(mult_clpx_img_ex),
		.frm_i(frm_csr),
		.apu_en_ex_o(apu_en_ex),
		.apu_type_ex_o(apu_type_ex),
		.apu_op_ex_o(apu_op_ex),
		.apu_lat_ex_o(apu_lat_ex),
		.apu_operands_ex_o(apu_operands_ex),
		.apu_flags_ex_o(apu_flags_ex),
		.apu_waddr_ex_o(apu_waddr_ex),
		.apu_read_regs_o(apu_read_regs),
		.apu_read_regs_valid_o(apu_read_regs_valid),
		.apu_read_dep_i(apu_read_dep),
		.apu_write_regs_o(apu_write_regs),
		.apu_write_regs_valid_o(apu_write_regs_valid),
		.apu_write_dep_i(apu_write_dep),
		.apu_perf_dep_o(perf_apu_dep),
		.apu_busy_i(apu_busy),
		.csr_access_ex_o(csr_access_ex),
		.csr_op_ex_o(csr_op_ex),
		.current_priv_lvl_i(current_priv_lvl),
		.csr_irq_sec_o(csr_irq_sec),
		.csr_cause_o(csr_cause),
		.csr_save_if_o(csr_save_if),
		.csr_save_id_o(csr_save_id),
		.csr_save_ex_o(csr_save_ex),
		.csr_restore_mret_id_o(csr_restore_mret_id),
		.csr_restore_uret_id_o(csr_restore_uret_id),
		.csr_restore_dret_id_o(csr_restore_dret_id),
		.csr_save_cause_o(csr_save_cause),
		.hwlp_start_o(hwlp_start),
		.hwlp_end_o(hwlp_end),
		.hwlp_cnt_o(hwlp_cnt),
		.csr_hwlp_regid_i(csr_hwlp_regid),
		.csr_hwlp_we_i(csr_hwlp_we),
		.csr_hwlp_data_i(csr_hwlp_data),
		.data_req_ex_o(data_req_ex),
		.data_we_ex_o(data_we_ex),
		.atop_ex_o(data_atop_ex),
		.data_type_ex_o(data_type_ex),
		.data_sign_ext_ex_o(data_sign_ext_ex),
		.data_reg_offset_ex_o(data_reg_offset_ex),
		.data_load_event_ex_o(data_load_event_ex),
		.data_misaligned_ex_o(data_misaligned_ex),
		.prepost_useincr_ex_o(useincr_addr_ex),
		.data_misaligned_i(data_misaligned),
		.data_err_i(data_err_pmp),
		.data_err_ack_o(data_err_ack),
		.irq_pending_i(irq_pending),
		.irq_id_i(irq_id),
		.irq_sec_i((PULP_SECURE ? irq_sec_i : 1'b0)),
		.m_irq_enable_i(m_irq_enable),
		.u_irq_enable_i(u_irq_enable),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.debug_mode_o(debug_mode),
		.debug_cause_o(debug_cause),
		.debug_csr_save_o(debug_csr_save),
		.debug_req_i(debug_req_i),
		.debug_single_step_i(debug_single_step),
		.debug_ebreakm_i(debug_ebreakm),
		.debug_ebreaku_i(debug_ebreaku),
		.regfile_waddr_wb_i(regfile_waddr_fw_wb_o),
		.regfile_we_wb_i(regfile_we_wb),
		.regfile_wdata_wb_i(regfile_wdata),
		.regfile_alu_waddr_fw_i(regfile_alu_waddr_fw),
		.regfile_alu_we_fw_i(regfile_alu_we_fw),
		.regfile_alu_wdata_fw_i(regfile_alu_wdata_fw),
		.mult_multicycle_i(mult_multicycle),
		.perf_jump_o(perf_jump),
		.perf_jr_stall_o(perf_jr_stall),
		.perf_ld_stall_o(perf_ld_stall),
		.perf_pipeline_stall_o(perf_pipeline_stall)
	);
	riscv_ex_stage #(
		.FPU(FPU),
		.FP_DIVSQRT(FP_DIVSQRT),
		.SHARED_FP(SHARED_FP),
		.SHARED_DSP_MULT(SHARED_DSP_MULT),
		.SHARED_INT_DIV(SHARED_INT_DIV),
		.APU_NARGS_CPU(APU_NARGS_CPU),
		.APU_WOP_CPU(APU_WOP_CPU),
		.APU_NDSFLAGS_CPU(APU_NDSFLAGS_CPU),
		.APU_NUSFLAGS_CPU(APU_NUSFLAGS_CPU)
	) ex_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.alu_en_i(alu_en_ex),
		.alu_operator_i(alu_operator_ex),
		.alu_operand_a_i(alu_operand_a_ex),
		.alu_operand_b_i(alu_operand_b_ex),
		.alu_operand_c_i(alu_operand_c_ex),
		.bmask_a_i(bmask_a_ex),
		.bmask_b_i(bmask_b_ex),
		.imm_vec_ext_i(imm_vec_ext_ex),
		.alu_vec_mode_i(alu_vec_mode_ex),
		.alu_is_clpx_i(alu_is_clpx_ex),
		.alu_is_subrot_i(alu_is_subrot_ex),
		.alu_clpx_shift_i(alu_clpx_shift_ex),
		.mult_operator_i(mult_operator_ex),
		.mult_operand_a_i(mult_operand_a_ex),
		.mult_operand_b_i(mult_operand_b_ex),
		.mult_operand_c_i(mult_operand_c_ex),
		.mult_en_i(mult_en_ex),
		.mult_sel_subword_i(mult_sel_subword_ex),
		.mult_signed_mode_i(mult_signed_mode_ex),
		.mult_imm_i(mult_imm_ex),
		.mult_dot_op_a_i(mult_dot_op_a_ex),
		.mult_dot_op_b_i(mult_dot_op_b_ex),
		.mult_dot_op_c_i(mult_dot_op_c_ex),
		.mult_dot_signed_i(mult_dot_signed_ex),
		.mult_is_clpx_i(mult_is_clpx_ex),
		.mult_clpx_shift_i(mult_clpx_shift_ex),
		.mult_clpx_img_i(mult_clpx_img_ex),
		.mult_multicycle_o(mult_multicycle),
		.fpu_prec_i(fprec_csr),
		.fpu_fflags_o(fflags),
		.fpu_fflags_we_o(fflags_we),
		.apu_en_i(apu_en_ex),
		.apu_op_i(apu_op_ex),
		.apu_lat_i(apu_lat_ex),
		.apu_operands_i(apu_operands_ex),
		.apu_waddr_i(apu_waddr_ex),
		.apu_flags_i(apu_flags_ex),
		.apu_read_regs_i(apu_read_regs),
		.apu_read_regs_valid_i(apu_read_regs_valid),
		.apu_read_dep_o(apu_read_dep),
		.apu_write_regs_i(apu_write_regs),
		.apu_write_regs_valid_i(apu_write_regs_valid),
		.apu_write_dep_o(apu_write_dep),
		.apu_perf_type_o(perf_apu_type),
		.apu_perf_cont_o(perf_apu_cont),
		.apu_perf_wb_o(perf_apu_wb),
		.apu_ready_wb_o(apu_ready_wb),
		.apu_busy_o(apu_busy),
		.apu_master_req_o(apu_master_req_o),
		.apu_master_ready_o(apu_master_ready_o),
		.apu_master_gnt_i(apu_master_gnt_i),
		.apu_master_operands_o(apu_master_operands_o),
		.apu_master_op_o(apu_master_op_o),
		.apu_master_valid_i(apu_master_valid_i),
		.apu_master_result_i(apu_master_result_i),
		.lsu_en_i(data_req_ex),
		.lsu_rdata_i(lsu_rdata),
		.csr_access_i(csr_access_ex),
		.csr_rdata_i(csr_rdata),
		.branch_in_ex_i(branch_in_ex),
		.regfile_alu_waddr_i(regfile_alu_waddr_ex),
		.regfile_alu_we_i(regfile_alu_we_ex),
		.regfile_waddr_i(regfile_waddr_ex),
		.regfile_we_i(regfile_we_ex),
		.regfile_waddr_wb_o(regfile_waddr_fw_wb_o),
		.regfile_we_wb_o(regfile_we_wb),
		.regfile_wdata_wb_o(regfile_wdata),
		.jump_target_o(jump_target_ex),
		.branch_decision_o(branch_decision),
		.regfile_alu_waddr_fw_o(regfile_alu_waddr_fw),
		.regfile_alu_we_fw_o(regfile_alu_we_fw),
		.regfile_alu_wdata_fw_o(regfile_alu_wdata_fw),
		.lsu_ready_ex_i(lsu_ready_ex),
		.lsu_err_i(data_err_pmp),
		.ex_ready_o(ex_ready),
		.ex_valid_o(ex_valid),
		.wb_ready_i(lsu_ready_wb)
	);
	riscv_load_store_unit load_store_unit_i(
		.clk(clk),
		.rst_n(rst_ni),
		.data_req_o(data_req_pmp),
		.data_gnt_i(data_gnt_pmp),
		.data_rvalid_i(data_rvalid_i),
		.data_err_i(data_err_pmp),
		.data_addr_o(data_addr_pmp),
		.data_we_o(data_we_o),
		.data_atop_o(data_atop_o),
		.data_be_o(data_be_o),
		.data_wdata_o(data_wdata_o),
		.data_rdata_i(data_rdata_i),
		.data_we_ex_i(data_we_ex),
		.data_atop_ex_i(data_atop_ex),
		.data_type_ex_i(data_type_ex),
		.data_wdata_ex_i(alu_operand_c_ex),
		.data_reg_offset_ex_i(data_reg_offset_ex),
		.data_sign_ext_ex_i(data_sign_ext_ex),
		.data_rdata_ex_o(lsu_rdata),
		.data_req_ex_i(data_req_ex),
		.operand_a_ex_i(alu_operand_a_ex),
		.operand_b_ex_i(alu_operand_b_ex),
		.addr_useincr_ex_i(useincr_addr_ex),
		.data_misaligned_ex_i(data_misaligned_ex),
		.data_misaligned_o(data_misaligned),
		.lsu_ready_ex_o(lsu_ready_ex),
		.lsu_ready_wb_o(lsu_ready_wb),
		.ex_valid_i(ex_valid),
		.busy_o(lsu_busy)
	);
	assign wb_valid = lsu_ready_wb & apu_ready_wb;
	riscv_cs_registers #(
		.N_EXT_CNT(N_EXT_PERF_COUNTERS),
		.A_EXTENSION(A_EXTENSION),
		.FPU(FPU),
		.APU(APU),
		.PULP_SECURE(PULP_SECURE),
		.USE_PMP(USE_PMP),
		.N_PMP_ENTRIES(N_PMP_ENTRIES)
	) cs_registers_i(
		.clk(clk),
		.rst_n(rst_ni),
		.core_id_i(core_id_i),
		.cluster_id_i(cluster_id_i),
		.mtvec_o(mtvec),
		.mtvecx_o(mtvecx),
		.utvec_o(utvec),
		.boot_addr_i(boot_addr_i[31:1]),
		.csr_access_i(csr_access),
		.csr_addr_i(csr_addr),
		.csr_wdata_i(csr_wdata),
		.csr_op_i(csr_op),
		.csr_rdata_o(csr_rdata),
		.frm_o(frm_csr),
		.fprec_o(fprec_csr),
		.fflags_i(fflags_csr),
		.fflags_we_i(fflags_we),
		.m_irq_enable_o(m_irq_enable),
		.u_irq_enable_o(u_irq_enable),
		.csr_irq_sec_i(csr_irq_sec),
		.sec_lvl_o(sec_lvl_o),
		.mepc_o(mepc),
		.uepc_o(uepc),
		.irq_software_i(irq_software_i),
		.irq_timer_i(irq_timer_i),
		.irq_external_i(irq_external_i),
		.irq_fast_i(irq_fast_i),
		.irq_nmi_i(irq_nmi_i),
		.irq_fastx_i(irq_fastx_i),
		.irq_pending_o(irq_pending),
		.irq_id_o(irq_id),
		.debug_mode_i(debug_mode),
		.debug_cause_i(debug_cause),
		.debug_csr_save_i(debug_csr_save),
		.depc_o(depc),
		.debug_single_step_o(debug_single_step),
		.debug_ebreakm_o(debug_ebreakm),
		.debug_ebreaku_o(debug_ebreaku),
		.priv_lvl_o(current_priv_lvl),
		.pmp_addr_o(pmp_addr),
		.pmp_cfg_o(pmp_cfg),
		.pc_if_i(pc_if),
		.pc_id_i(pc_id),
		.pc_ex_i(pc_ex),
		.csr_save_if_i(csr_save_if),
		.csr_save_id_i(csr_save_id),
		.csr_save_ex_i(csr_save_ex),
		.csr_restore_mret_i(csr_restore_mret_id),
		.csr_restore_uret_i(csr_restore_uret_id),
		.csr_restore_dret_i(csr_restore_dret_id),
		.csr_cause_i(csr_cause),
		.csr_save_cause_i(csr_save_cause),
		.hwlp_start_i(hwlp_start),
		.hwlp_end_i(hwlp_end),
		.hwlp_cnt_i(hwlp_cnt),
		.hwlp_regid_o(csr_hwlp_regid),
		.hwlp_we_o(csr_hwlp_we),
		.hwlp_data_o(csr_hwlp_data),
		.id_valid_i(id_valid),
		.is_compressed_i(is_compressed_id),
		.is_decoding_i(is_decoding),
		.imiss_i(perf_imiss),
		.pc_set_i(pc_set),
		.jump_i(perf_jump),
		.branch_i(branch_in_ex),
		.branch_taken_i(branch_decision),
		.ld_stall_i(perf_ld_stall),
		.jr_stall_i(perf_jr_stall),
		.pipeline_stall_i(perf_pipeline_stall),
		.apu_typeconflict_i(perf_apu_type),
		.apu_contention_i(perf_apu_cont),
		.apu_dep_i(perf_apu_dep),
		.apu_wb_i(perf_apu_wb),
		.mem_load_i((data_req_o & data_gnt_i) & ~data_we_o),
		.mem_store_i((data_req_o & data_gnt_i) & data_we_o),
		.ext_counters_i(ext_perf_counters_i)
	);
	assign csr_access = csr_access_ex;
	assign csr_addr = csr_addr_int;
	assign csr_wdata = alu_operand_a_ex;
	assign csr_op = csr_op_ex;
	function automatic [11:0] sv2v_cast_12;
		input reg [11:0] inp;
		sv2v_cast_12 = inp;
	endfunction
	assign csr_addr_int = sv2v_cast_12((csr_access_ex ? alu_operand_b_ex[11:0] : {12 {1'sb0}}));
	generate
		if (PULP_SECURE && USE_PMP) begin : RISCY_PMP
			riscv_pmp #(.N_PMP_ENTRIES(N_PMP_ENTRIES)) pmp_unit_i(
				.clk(clk),
				.rst_n(rst_ni),
				.pmp_privil_mode_i(current_priv_lvl),
				.pmp_addr_i(pmp_addr),
				.pmp_cfg_i(pmp_cfg),
				.data_req_i(data_req_pmp),
				.data_addr_i(data_addr_pmp),
				.data_we_i(data_we_o),
				.data_gnt_o(data_gnt_pmp),
				.data_req_o(data_req_o),
				.data_gnt_i(data_gnt_i),
				.data_addr_o(data_addr_o),
				.data_err_o(data_err_pmp),
				.data_err_ack_i(data_err_ack),
				.instr_req_i(instr_req_pmp),
				.instr_addr_i(instr_addr_pmp),
				.instr_gnt_o(instr_gnt_pmp),
				.instr_req_o(instr_req_o),
				.instr_gnt_i(instr_gnt_i),
				.instr_addr_o(instr_addr_o),
				.instr_err_o(instr_err_pmp)
			);
		end
		else begin : genblk2
			assign instr_req_o = instr_req_pmp;
			assign instr_addr_o = instr_addr_pmp;
			assign instr_gnt_pmp = instr_gnt_i;
			assign instr_err_pmp = 1'b0;
			assign data_req_o = data_req_pmp;
			assign data_addr_o = data_addr_pmp;
			assign data_gnt_pmp = data_gnt_i;
			assign data_err_pmp = 1'b0;
		end
	endgenerate
	assign pipe_flush = id_stage_i.controller_i.pipe_flush_i;
	assign mret_insn = id_stage_i.controller_i.mret_insn_i;
	assign uret_insn = id_stage_i.controller_i.uret_insn_i;
	assign ecall_insn = id_stage_i.controller_i.ecall_insn_i;
	assign ebrk_insn = id_stage_i.controller_i.ebrk_insn_i;
	assign ivalid_o = ((((((id_valid || pipe_flush) || mret_insn) || uret_insn) || ecall_insn) || ebrk_insn) && is_decoding) || csr_cause[5];
	assign iexception_o = ((csr_cause[5] | ecall_insn) | ebrk_insn) | illegal_insn;
	assign interrupt_o = csr_cause[5];
	assign cause_o = exc_cause[4:0];
	assign tval_o = 1'sb0;
	assign priv_o = 1'sb1;
	assign iaddr_o = pc_id;
	assign instr_o = instr_rdata_id;
	assign compressed_o = is_compressed_id;
endmodule