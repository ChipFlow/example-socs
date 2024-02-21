module dm_csrs (
	clk_i,
	rst_ni,
	testmode_i,
	dmi_rst_ni,
	dmi_req_valid_i,
	dmi_req_ready_o,
	dmi_req_i,
	dmi_resp_valid_o,
	dmi_resp_ready_i,
	dmi_resp_o,
	ndmreset_o,
	dmactive_o,
	hartinfo_i,
	halted_i,
	unavailable_i,
	resumeack_i,
	hartsel_o,
	haltreq_o,
	resumereq_o,
	clear_resumeack_o,
	cmd_valid_o,
	cmd_o,
	cmderror_valid_i,
	cmderror_i,
	cmdbusy_i,
	progbuf_o,
	data_o,
	data_i,
	data_valid_i,
	sbaddress_o,
	sbaddress_i,
	sbaddress_write_valid_o,
	sbreadonaddr_o,
	sbautoincrement_o,
	sbaccess_o,
	sbreadondata_o,
	sbdata_o,
	sbdata_read_valid_o,
	sbdata_write_valid_o,
	sbdata_i,
	sbdata_valid_i,
	sbbusy_i,
	sberror_valid_i,
	sberror_i
);
	reg _sv2v_0;
	parameter [31:0] NrHarts = 1;
	parameter [31:0] BusWidth = 32;
	parameter [NrHarts - 1:0] SelectableHarts = {NrHarts {1'b1}};
	input wire clk_i;
	(* keep *) input wire rst_ni;
	input wire testmode_i;
	(* keep *) input wire dmi_rst_ni;
	input wire dmi_req_valid_i;
	output wire dmi_req_ready_o;
	input wire [40:0] dmi_req_i;
	output wire dmi_resp_valid_o;
	input wire dmi_resp_ready_i;
	output wire [33:0] dmi_resp_o;
	output wire ndmreset_o;
	output wire dmactive_o;
	input wire [(NrHarts * 32) - 1:0] hartinfo_i;
	input wire [NrHarts - 1:0] halted_i;
	input wire [NrHarts - 1:0] unavailable_i;
	input wire [NrHarts - 1:0] resumeack_i;
	output wire [19:0] hartsel_o;
	output reg [NrHarts - 1:0] haltreq_o;
	output reg [NrHarts - 1:0] resumereq_o;
	output reg clear_resumeack_o;
	output wire cmd_valid_o;
	output wire [31:0] cmd_o;
	input wire cmderror_valid_i;
	input wire [2:0] cmderror_i;
	input wire cmdbusy_i;
	localparam [4:0] dm_ProgBufSize = 5'h08;
	output wire [255:0] progbuf_o;
	localparam [3:0] dm_DataCount = 4'h2;
	output wire [63:0] data_o;
	input wire [63:0] data_i;
	input wire data_valid_i;
	output wire [BusWidth - 1:0] sbaddress_o;
	input wire [BusWidth - 1:0] sbaddress_i;
	output reg sbaddress_write_valid_o;
	output wire sbreadonaddr_o;
	output wire sbautoincrement_o;
	output wire [2:0] sbaccess_o;
	output wire sbreadondata_o;
	output wire [BusWidth - 1:0] sbdata_o;
	output reg sbdata_read_valid_o;
	output reg sbdata_write_valid_o;
	input wire [BusWidth - 1:0] sbdata_i;
	input wire sbdata_valid_i;
	input wire sbbusy_i;
	input wire sberror_valid_i;
	input wire [2:0] sberror_i;
	localparam [31:0] HartSelLen = (NrHarts == 1 ? 1 : $clog2(NrHarts));
	localparam [31:0] NrHartsAligned = 2 ** HartSelLen;
	wire [1:0] dtm_op;
	function automatic [1:0] sv2v_cast_2;
		input reg [1:0] inp;
		sv2v_cast_2 = inp;
	endfunction
	assign dtm_op = sv2v_cast_2(dmi_req_i[33-:2]);
	(* keep *) wire resp_queue_full;
	(* keep *) wire resp_queue_empty;
	(* keep *) wire resp_queue_push;
	(* keep *) wire resp_queue_pop;
	function automatic [7:0] sv2v_cast_8;
		input reg [7:0] inp;
		sv2v_cast_8 = inp;
	endfunction
	localparam [7:0] DataEnd = sv2v_cast_8((8'h04 + {4'h0, dm_DataCount}) - 8'h01);
	localparam [7:0] ProgBufEnd = sv2v_cast_8((8'h20 + {4'h0, dm_ProgBufSize}) - 8'h01);
	reg [31:0] haltsum0;
	reg [31:0] haltsum1;
	reg [31:0] haltsum2;
	reg [31:0] haltsum3;
	reg [((((NrHarts - 1) / 32) + 1) * 32) - 1:0] halted;
	reg [(((NrHarts - 1) / 32) >= 0 ? ((((NrHarts - 1) / 32) + 1) * 32) - 1 : ((1 - ((NrHarts - 1) / 32)) * 32) + ((((NrHarts - 1) / 32) * 32) - 1)):(((NrHarts - 1) / 32) >= 0 ? 0 : ((NrHarts - 1) / 32) * 32)] halted_reshaped0;
	reg [(((NrHarts - 1) / 1024) >= 0 ? ((((NrHarts - 1) / 1024) + 1) * 32) - 1 : ((1 - ((NrHarts - 1) / 1024)) * 32) + ((((NrHarts - 1) / 1024) * 32) - 1)):(((NrHarts - 1) / 1024) >= 0 ? 0 : ((NrHarts - 1) / 1024) * 32)] halted_reshaped1;
	reg [(((NrHarts - 1) / 32768) >= 0 ? ((((NrHarts - 1) / 32768) + 1) * 32) - 1 : ((1 - ((NrHarts - 1) / 32768)) * 32) + ((((NrHarts - 1) / 32768) * 32) - 1)):(((NrHarts - 1) / 32768) >= 0 ? 0 : ((NrHarts - 1) / 32768) * 32)] halted_reshaped2;
	reg [((((NrHarts - 1) / 1024) + 1) * 32) - 1:0] halted_flat1;
	reg [((((NrHarts - 1) / 32768) + 1) * 32) - 1:0] halted_flat2;
	reg [31:0] halted_flat3;
	reg [14:0] hartsel_idx0;
	function automatic [14:0] sv2v_cast_15;
		input reg [14:0] inp;
		sv2v_cast_15 = inp;
	endfunction
	always @(*) begin : p_haltsum0
		if (_sv2v_0)
			;
		halted = 1'sb0;
		haltsum0 = 1'sb0;
		hartsel_idx0 = hartsel_o[19:5];
		halted[NrHarts - 1:0] = halted_i;
		halted_reshaped0 = halted;
		if (hartsel_idx0 < sv2v_cast_15(((NrHarts - 1) / 32) + 1))
			haltsum0 = halted_reshaped0[(((NrHarts - 1) / 32) >= 0 ? hartsel_idx0 : ((NrHarts - 1) / 32) - hartsel_idx0) * 32+:32];
	end
	reg [9:0] hartsel_idx1;
	function automatic [9:0] sv2v_cast_10;
		input reg [9:0] inp;
		sv2v_cast_10 = inp;
	endfunction
	always @(*) begin : p_reduction1
		if (_sv2v_0)
			;
		halted_flat1 = 1'sb0;
		haltsum1 = 1'sb0;
		hartsel_idx1 = hartsel_o[19:10];
		begin : sv2v_autoblock_1
			reg [31:0] k;
			for (k = 0; k < (((NrHarts - 1) / 32) + 1); k = k + 1)
				halted_flat1[k] = |halted_reshaped0[(((NrHarts - 1) / 32) >= 0 ? k : ((NrHarts - 1) / 32) - k) * 32+:32];
		end
		halted_reshaped1 = halted_flat1;
		if (hartsel_idx1 < sv2v_cast_10(((NrHarts - 1) / 1024) + 1))
			haltsum1 = halted_reshaped1[(((NrHarts - 1) / 1024) >= 0 ? hartsel_idx1 : ((NrHarts - 1) / 1024) - hartsel_idx1) * 32+:32];
	end
	reg [4:0] hartsel_idx2;
	function automatic [4:0] sv2v_cast_5;
		input reg [4:0] inp;
		sv2v_cast_5 = inp;
	endfunction
	always @(*) begin : p_reduction2
		if (_sv2v_0)
			;
		halted_flat2 = 1'sb0;
		haltsum2 = 1'sb0;
		hartsel_idx2 = hartsel_o[19:15];
		begin : sv2v_autoblock_2
			reg [31:0] k;
			for (k = 0; k < (((NrHarts - 1) / 1024) + 1); k = k + 1)
				halted_flat2[k] = |halted_reshaped1[(((NrHarts - 1) / 1024) >= 0 ? k : ((NrHarts - 1) / 1024) - k) * 32+:32];
		end
		halted_reshaped2 = halted_flat2;
		if (hartsel_idx2 < sv2v_cast_5(((NrHarts - 1) / 32768) + 1))
			haltsum2 = halted_reshaped2[(((NrHarts - 1) / 32768) >= 0 ? hartsel_idx2 : ((NrHarts - 1) / 32768) - hartsel_idx2) * 32+:32];
	end
	always @(*) begin : p_reduction3
		if (_sv2v_0)
			;
		halted_flat3 = 1'sb0;
		begin : sv2v_autoblock_3
			reg [31:0] k;
			for (k = 0; k < ((NrHarts / 32768) + 1); k = k + 1)
				halted_flat3[k] = |halted_reshaped2[(((NrHarts - 1) / 32768) >= 0 ? k : ((NrHarts - 1) / 32768) - k) * 32+:32];
		end
		haltsum3 = halted_flat3;
	end
	reg [31:0] dmstatus;
	reg [31:0] dmcontrol_d;
	reg [31:0] dmcontrol_q;
	reg [31:0] abstractcs;
	reg [2:0] cmderr_d;
	reg [2:0] cmderr_q;
	reg [31:0] command_d;
	reg [31:0] command_q;
	reg cmd_valid_d;
	reg cmd_valid_q;
	reg [31:0] abstractauto_d;
	reg [31:0] abstractauto_q;
	reg [31:0] sbcs_d;
	reg [31:0] sbcs_q;
	reg [63:0] sbaddr_d;
	reg [63:0] sbaddr_q;
	reg [63:0] sbdata_d;
	reg [63:0] sbdata_q;
	wire [NrHarts - 1:0] havereset_d;
	reg [NrHarts - 1:0] havereset_q;
	reg [255:0] progbuf_d;
	reg [255:0] progbuf_q;
	reg [63:0] data_d;
	reg [63:0] data_q;
	reg [HartSelLen - 1:0] selected_hart;
	reg [33:0] resp_queue_inp;
	assign dmi_resp_valid_o = ~resp_queue_empty;
	assign dmi_req_ready_o = ~resp_queue_full;
	assign resp_queue_push = dmi_req_valid_i & dmi_req_ready_o;
	assign sbautoincrement_o = sbcs_q[16];
	assign sbreadonaddr_o = sbcs_q[20];
	assign sbreadondata_o = sbcs_q[15];
	assign sbaccess_o = sbcs_q[19-:3];
	assign sbdata_o = sbdata_q[BusWidth - 1:0];
	assign sbaddress_o = sbaddr_q[BusWidth - 1:0];
	assign hartsel_o = {dmcontrol_q[15-:10], dmcontrol_q[25-:10]};
	reg [NrHartsAligned - 1:0] havereset_d_aligned;
	wire [NrHartsAligned - 1:0] havereset_q_aligned;
	wire [NrHartsAligned - 1:0] resumeack_aligned;
	wire [NrHartsAligned - 1:0] unavailable_aligned;
	wire [NrHartsAligned - 1:0] halted_aligned;
	function automatic [NrHartsAligned - 1:0] sv2v_cast_24E98;
		input reg [NrHartsAligned - 1:0] inp;
		sv2v_cast_24E98 = inp;
	endfunction
	assign resumeack_aligned = sv2v_cast_24E98(resumeack_i);
	assign unavailable_aligned = sv2v_cast_24E98(unavailable_i);
	assign halted_aligned = sv2v_cast_24E98(halted_i);
	function automatic [NrHarts - 1:0] sv2v_cast_5B351;
		input reg [NrHarts - 1:0] inp;
		sv2v_cast_5B351 = inp;
	endfunction
	assign havereset_d = sv2v_cast_5B351(havereset_d_aligned);
	assign havereset_q_aligned = sv2v_cast_24E98(havereset_q);
	reg [(NrHartsAligned * 32) - 1:0] hartinfo_aligned;
	always @(*) begin : p_hartinfo_align
		if (_sv2v_0)
			;
		hartinfo_aligned = 1'sb0;
		hartinfo_aligned[32 * ((NrHarts - 1) - (NrHarts - 1))+:32 * NrHarts] = hartinfo_i;
	end
	wire [7:0] dm_csr_addr;
	reg [31:0] sbcs;
	reg [31:0] a_abstractcs;
	wire [3:0] autoexecdata_idx;
	assign dm_csr_addr = sv2v_cast_8({1'b0, dmi_req_i[40-:7]});
	function automatic [3:0] sv2v_cast_4;
		input reg [3:0] inp;
		sv2v_cast_4 = inp;
	endfunction
	assign autoexecdata_idx = sv2v_cast_4({dm_csr_addr} - 8'h04);
	localparam [3:0] dm_DbgVersion013 = 4'h2;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	function automatic [63:0] sv2v_cast_64;
		input reg [63:0] inp;
		sv2v_cast_64 = inp;
	endfunction
	function automatic [$clog2(4'h2) - 1:0] sv2v_cast_7FB6B;
		input reg [$clog2(4'h2) - 1:0] inp;
		sv2v_cast_7FB6B = inp;
	endfunction
	function automatic [2:0] sv2v_cast_3;
		input reg [2:0] inp;
		sv2v_cast_3 = inp;
	endfunction
	function automatic [11:0] sv2v_cast_12;
		input reg [11:0] inp;
		sv2v_cast_12 = inp;
	endfunction
	function automatic [15:0] sv2v_cast_16;
		input reg [15:0] inp;
		sv2v_cast_16 = inp;
	endfunction
	function automatic [6:0] sv2v_cast_D1C9A;
		input reg [6:0] inp;
		sv2v_cast_D1C9A = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		(* xprop_off *)
		begin : csr_read_write
			dmstatus = 1'sb0;
			dmstatus[3-:4] = dm_DbgVersion013;
			dmstatus[7] = 1'b1;
			dmstatus[5] = 1'b0;
			dmstatus[19] = havereset_q_aligned[selected_hart];
			dmstatus[18] = havereset_q_aligned[selected_hart];
			dmstatus[17] = resumeack_aligned[selected_hart];
			dmstatus[16] = resumeack_aligned[selected_hart];
			dmstatus[13] = unavailable_aligned[selected_hart];
			dmstatus[12] = unavailable_aligned[selected_hart];
			dmstatus[15] = sv2v_cast_32(hartsel_o) > (NrHarts - 32'sd1);
			dmstatus[14] = sv2v_cast_32(hartsel_o) > (NrHarts - 32'sd1);
			dmstatus[9] = halted_aligned[selected_hart] & ~unavailable_aligned[selected_hart];
			dmstatus[8] = halted_aligned[selected_hart] & ~unavailable_aligned[selected_hart];
			dmstatus[11] = ~halted_aligned[selected_hart] & ~unavailable_aligned[selected_hart];
			dmstatus[10] = ~halted_aligned[selected_hart] & ~unavailable_aligned[selected_hart];
			abstractcs = 1'sb0;
			abstractcs[3-:4] = dm_DataCount;
			abstractcs[28-:5] = dm_ProgBufSize;
			abstractcs[12] = cmdbusy_i;
			abstractcs[10-:3] = cmderr_q;
			abstractauto_d = abstractauto_q;
			abstractauto_d[15-:4] = 1'sb0;
			havereset_d_aligned = sv2v_cast_24E98(havereset_q);
			dmcontrol_d = dmcontrol_q;
			cmderr_d = cmderr_q;
			command_d = command_q;
			progbuf_d = progbuf_q;
			data_d = data_q;
			sbcs_d = sbcs_q;
			sbaddr_d = sv2v_cast_64(sbaddress_i);
			sbdata_d = sbdata_q;
			resp_queue_inp[33-:32] = 32'h00000000;
			resp_queue_inp[1-:2] = 2'h0;
			cmd_valid_d = 1'b0;
			sbaddress_write_valid_o = 1'b0;
			sbdata_read_valid_o = 1'b0;
			sbdata_write_valid_o = 1'b0;
			clear_resumeack_o = 1'b0;
			sbcs = 1'sb0;
			a_abstractcs = 1'sb0;
			if ((dmi_req_ready_o && dmi_req_valid_i) && (dtm_op == 2'h1)) begin
				if ((8'h04 <= dm_csr_addr) && (DataEnd >= dm_csr_addr)) begin
					resp_queue_inp[33-:32] = data_q[sv2v_cast_7FB6B(autoexecdata_idx) * 32+:32];
					if (!cmdbusy_i)
						cmd_valid_d = abstractauto_q[0 + autoexecdata_idx];
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if (dm_csr_addr == 8'h10)
					resp_queue_inp[33-:32] = dmcontrol_q;
				else if (dm_csr_addr == 8'h11)
					resp_queue_inp[33-:32] = dmstatus;
				else if (dm_csr_addr == 8'h12)
					resp_queue_inp[33-:32] = hartinfo_aligned[selected_hart * 32+:32];
				else if (dm_csr_addr == 8'h16)
					resp_queue_inp[33-:32] = abstractcs;
				else if (dm_csr_addr == 8'h18)
					resp_queue_inp[33-:32] = abstractauto_q;
				else if (dm_csr_addr == 8'h17)
					resp_queue_inp[33-:32] = 1'sb0;
				else if ((8'h20 <= dm_csr_addr) && (ProgBufEnd >= dm_csr_addr)) begin
					resp_queue_inp[33-:32] = progbuf_q[dmi_req_i[$clog2(5'h08) + 33:34] * 32+:32];
					if (!cmdbusy_i)
						cmd_valid_d = abstractauto_q[0 + {1'b1, dmi_req_i[37:34]}];
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if (dm_csr_addr == 8'h40)
					resp_queue_inp[33-:32] = haltsum0;
				else if (dm_csr_addr == 8'h13)
					resp_queue_inp[33-:32] = haltsum1;
				else if (dm_csr_addr == 8'h34)
					resp_queue_inp[33-:32] = haltsum2;
				else if (dm_csr_addr == 8'h35)
					resp_queue_inp[33-:32] = haltsum3;
				else if (dm_csr_addr == 8'h38)
					resp_queue_inp[33-:32] = sbcs_q;
				else if (dm_csr_addr == 8'h39)
					resp_queue_inp[33-:32] = sbaddr_q[31:0];
				else if (dm_csr_addr == 8'h3a)
					resp_queue_inp[33-:32] = sbaddr_q[63:32];
				else if (dm_csr_addr == 8'h3c) begin
					if (sbbusy_i || sbcs_q[22]) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else begin
						sbdata_read_valid_o = sbcs_q[14-:3] == {3 {1'sb0}};
						resp_queue_inp[33-:32] = sbdata_q[31:0];
					end
				end
				else if (dm_csr_addr == 8'h3d) begin
					if (sbbusy_i || sbcs_q[22]) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else
						resp_queue_inp[33-:32] = sbdata_q[63:32];
				end
			end
			if ((dmi_req_ready_o && dmi_req_valid_i) && (dtm_op == 2'h2)) begin
				if ((8'h04 <= dm_csr_addr) && (DataEnd >= dm_csr_addr)) begin
					if (!cmdbusy_i) begin
						data_d[dmi_req_i[$clog2(4'h2) + 33:34] * 32+:32] = dmi_req_i[31-:32];
						cmd_valid_d = abstractauto_q[0 + autoexecdata_idx];
					end
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if (dm_csr_addr == 8'h10) begin
					dmcontrol_d = dmi_req_i[31-:32];
					if (dmcontrol_d[28])
						havereset_d_aligned[selected_hart] = 1'b0;
				end
				else if (dm_csr_addr == 8'h11)
					;
				else if (dm_csr_addr == 8'h12)
					;
				else if (dm_csr_addr == 8'h16) begin
					a_abstractcs = sv2v_cast_32(dmi_req_i[31-:32]);
					if (!cmdbusy_i)
						cmderr_d = sv2v_cast_3(~a_abstractcs[10-:3] & cmderr_q);
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if (dm_csr_addr == 8'h17) begin
					if (!cmdbusy_i) begin
						cmd_valid_d = 1'b1;
						command_d = sv2v_cast_32(dmi_req_i[31-:32]);
					end
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if (dm_csr_addr == 8'h18) begin
					if (!cmdbusy_i) begin
						abstractauto_d = 32'h00000000;
						abstractauto_d[11-:12] = sv2v_cast_12(dmi_req_i[1:0]);
						abstractauto_d[31-:16] = sv2v_cast_16(dmi_req_i[23:16]);
					end
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if ((8'h20 <= dm_csr_addr) && (ProgBufEnd >= dm_csr_addr)) begin
					if (!cmdbusy_i) begin
						progbuf_d[dmi_req_i[$clog2(5'h08) + 33:34] * 32+:32] = dmi_req_i[31-:32];
						cmd_valid_d = abstractauto_q[0 + {1'b1, dmi_req_i[37:34]}];
					end
					else begin
						resp_queue_inp[1-:2] = 2'h3;
						if (cmderr_q == 3'd0)
							cmderr_d = 3'd1;
					end
				end
				else if (dm_csr_addr == 8'h38) begin
					if (sbbusy_i) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else begin
						sbcs = sv2v_cast_32(dmi_req_i[31-:32]);
						sbcs_d = sbcs;
						sbcs_d[22] = sbcs_q[22] & ~sbcs[22];
						sbcs_d[14-:3] = (|sbcs[14-:3] ? 3'b000 : sbcs_q[14-:3]);
					end
				end
				else if (dm_csr_addr == 8'h39) begin
					if (sbbusy_i || sbcs_q[22]) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else begin
						sbaddr_d[31:0] = dmi_req_i[31-:32];
						sbaddress_write_valid_o = sbcs_q[14-:3] == {3 {1'sb0}};
					end
				end
				else if (dm_csr_addr == 8'h3a) begin
					if (sbbusy_i || sbcs_q[22]) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else
						sbaddr_d[63:32] = dmi_req_i[31-:32];
				end
				else if (dm_csr_addr == 8'h3c) begin
					if (sbbusy_i || sbcs_q[22]) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else begin
						sbdata_d[31:0] = dmi_req_i[31-:32];
						sbdata_write_valid_o = sbcs_q[14-:3] == {3 {1'sb0}};
					end
				end
				else if (dm_csr_addr == 8'h3d) begin
					if (sbbusy_i || sbcs_q[22]) begin
						sbcs_d[22] = 1'b1;
						resp_queue_inp[1-:2] = 2'h3;
					end
					else
						sbdata_d[63:32] = dmi_req_i[31-:32];
				end
			end
			if (cmderror_valid_i)
				cmderr_d = cmderror_i;
			if (data_valid_i)
				data_d = data_i;
			if (ndmreset_o)
				havereset_d_aligned[NrHarts - 1:0] = 1'sb1;
			if (sberror_valid_i)
				sbcs_d[14-:3] = sberror_i;
			if (sbdata_valid_i)
				sbdata_d = sv2v_cast_64(sbdata_i);
			dmcontrol_d[26] = 1'b0;
			dmcontrol_d[29] = 1'b0;
			dmcontrol_d[3] = 1'b0;
			dmcontrol_d[2] = 1'b0;
			dmcontrol_d[27] = 1'sb0;
			dmcontrol_d[5-:2] = 1'sb0;
			dmcontrol_d[28] = 1'b0;
			if (!dmcontrol_q[30] && dmcontrol_d[30])
				clear_resumeack_o = 1'b1;
			if (dmcontrol_q[30] && resumeack_i)
				dmcontrol_d[30] = 1'b0;
			sbcs_d[31-:3] = 3'd1;
			sbcs_d[21] = sbbusy_i;
			sbcs_d[11-:7] = sv2v_cast_D1C9A(BusWidth);
			sbcs_d[4] = BusWidth >= 32'd128;
			sbcs_d[3] = BusWidth >= 32'd64;
			sbcs_d[2] = BusWidth >= 32'd32;
			sbcs_d[1] = BusWidth >= 32'd16;
			sbcs_d[0] = BusWidth >= 32'd8;
		end
	end
	function automatic [HartSelLen - 1:0] sv2v_cast_5F664;
		input reg [HartSelLen - 1:0] inp;
		sv2v_cast_5F664 = inp;
	endfunction
	always @(*) begin : p_outmux
		if (_sv2v_0)
			;
		selected_hart = hartsel_o[HartSelLen - 1:0];
		haltreq_o = 1'sb0;
		resumereq_o = 1'sb0;
		if (selected_hart <= sv2v_cast_5F664(NrHarts - 1)) begin
			haltreq_o[selected_hart] = dmcontrol_q[31];
			resumereq_o[selected_hart] = dmcontrol_q[30];
		end
	end
	assign dmactive_o = dmcontrol_q[0];
	assign cmd_o = command_q;
	assign cmd_valid_o = cmd_valid_q;
	assign progbuf_o = progbuf_q;
	assign data_o = data_q;
	assign resp_queue_pop = dmi_resp_ready_i & ~resp_queue_empty;
	assign ndmreset_o = dmcontrol_q[1];
	fifo_v2_A387C #(.DEPTH(2)) i_fifo(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.flush_i(~dmi_rst_ni),
		.testmode_i(testmode_i),
		.full_o(resp_queue_full),
		.empty_o(resp_queue_empty),
		.alm_full_o(),
		.alm_empty_o(),
		.data_i(resp_queue_inp),
		.push_i(resp_queue_push),
		.data_o(dmi_resp_o),
		.pop_i(resp_queue_pop)
	);
	always @(posedge clk_i or negedge rst_ni) begin : p_regs
		if (!rst_ni) begin
			dmcontrol_q <= 1'sb0;
			cmderr_q <= 3'd0;
			command_q <= 1'sb0;
			cmd_valid_q <= 1'sb0;
			abstractauto_q <= 1'sb0;
			progbuf_q <= 1'sb0;
			data_q <= 1'sb0;
			sbcs_q <= 32'h00040000;
			sbaddr_q <= 1'sb0;
			sbdata_q <= 1'sb0;
			havereset_q <= 1'sb1;
		end
		else begin
			havereset_q <= SelectableHarts & havereset_d;
			if (!dmcontrol_q[0]) begin
				dmcontrol_q[31] <= 1'sb0;
				dmcontrol_q[30] <= 1'sb0;
				dmcontrol_q[29] <= 1'sb0;
				dmcontrol_q[28] <= 1'sb0;
				dmcontrol_q[27] <= 1'sb0;
				dmcontrol_q[26] <= 1'sb0;
				dmcontrol_q[25-:10] <= 1'sb0;
				dmcontrol_q[15-:10] <= 1'sb0;
				dmcontrol_q[5-:2] <= 1'sb0;
				dmcontrol_q[3] <= 1'sb0;
				dmcontrol_q[2] <= 1'sb0;
				dmcontrol_q[1] <= 1'sb0;
				dmcontrol_q[0] <= dmcontrol_d[0];
				cmderr_q <= 3'd0;
				command_q <= 1'sb0;
				cmd_valid_q <= 1'sb0;
				abstractauto_q <= 1'sb0;
				progbuf_q <= 1'sb0;
				data_q <= 1'sb0;
				sbcs_q <= 32'h00040000;
				sbaddr_q <= 1'sb0;
				sbdata_q <= 1'sb0;
			end
			else begin
				dmcontrol_q <= dmcontrol_d;
				cmderr_q <= cmderr_d;
				command_q <= command_d;
				cmd_valid_q <= cmd_valid_d;
				abstractauto_q <= abstractauto_d;
				progbuf_q <= progbuf_d;
				data_q <= data_d;
				sbcs_q <= sbcs_d;
				sbaddr_q <= sbaddr_d;
				sbdata_q <= sbdata_d;
			end
		end
	end
	initial _sv2v_0 = 0;
endmodule
module dm_mem (
	clk_i,
	rst_ni,
	debug_req_o,
	ndmreset_i,
	hartsel_i,
	haltreq_i,
	resumereq_i,
	clear_resumeack_i,
	halted_o,
	resuming_o,
	progbuf_i,
	data_i,
	data_o,
	data_valid_o,
	cmd_valid_i,
	cmd_i,
	cmderror_valid_o,
	cmderror_o,
	cmdbusy_o,
	req_i,
	we_i,
	addr_i,
	wdata_i,
	be_i,
	rdata_o
);
	reg _sv2v_0;
	parameter [31:0] NrHarts = 1;
	parameter [31:0] BusWidth = 32;
	parameter [NrHarts - 1:0] SelectableHarts = {NrHarts {1'b1}};
	parameter [31:0] DmBaseAddress = 1'sb0;
	input wire clk_i;
	input wire rst_ni;
	output wire [NrHarts - 1:0] debug_req_o;
	input wire ndmreset_i;
	input wire [19:0] hartsel_i;
	input wire [NrHarts - 1:0] haltreq_i;
	input wire [NrHarts - 1:0] resumereq_i;
	input wire clear_resumeack_i;
	output wire [NrHarts - 1:0] halted_o;
	output wire [NrHarts - 1:0] resuming_o;
	localparam [4:0] dm_ProgBufSize = 5'h08;
	input wire [255:0] progbuf_i;
	localparam [3:0] dm_DataCount = 4'h2;
	input wire [63:0] data_i;
	output reg [63:0] data_o;
	output reg data_valid_o;
	input wire cmd_valid_i;
	input wire [31:0] cmd_i;
	output reg cmderror_valid_o;
	output reg [2:0] cmderror_o;
	output reg cmdbusy_o;
	input wire req_i;
	input wire we_i;
	input wire [BusWidth - 1:0] addr_i;
	input wire [BusWidth - 1:0] wdata_i;
	input wire [(BusWidth / 8) - 1:0] be_i;
	output wire [BusWidth - 1:0] rdata_o;
	localparam [31:0] DbgAddressBits = 12;
	localparam [31:0] HartSelLen = (NrHarts == 1 ? 1 : $clog2(NrHarts));
	localparam [31:0] NrHartsAligned = 2 ** HartSelLen;
	localparam [31:0] MaxAar = (BusWidth == 64 ? 4 : 3);
	localparam [0:0] HasSndScratch = DmBaseAddress != 0;
	localparam [4:0] LoadBaseAddr = (DmBaseAddress == 0 ? 5'd0 : 5'd10);
	localparam [11:0] dm_DataAddr = 12'h380;
	localparam [11:0] DataBaseAddr = dm_DataAddr;
	localparam [11:0] DataEndAddr = 903;
	localparam [11:0] ProgBufBaseAddr = 864;
	localparam [11:0] ProgBufEndAddr = 895;
	localparam [11:0] AbstractCmdBaseAddr = ProgBufBaseAddr - 40;
	localparam [11:0] AbstractCmdEndAddr = ProgBufBaseAddr - 1;
	localparam [11:0] WhereToAddr = 'h300;
	localparam [11:0] FlagsBaseAddr = 'h400;
	localparam [11:0] FlagsEndAddr = 'h7ff;
	localparam [11:0] HaltedAddr = 'h100;
	localparam [11:0] GoingAddr = 'h108;
	localparam [11:0] ResumingAddr = 'h110;
	localparam [11:0] ExceptionAddr = 'h118;
	wire [255:0] progbuf;
	reg [511:0] abstract_cmd;
	wire [NrHarts - 1:0] halted_d;
	reg [NrHarts - 1:0] halted_q;
	wire [NrHarts - 1:0] resuming_d;
	reg [NrHarts - 1:0] resuming_q;
	reg resume;
	reg go;
	reg going;
	reg exception;
	reg unsupported_command;
	wire [63:0] rom_rdata;
	reg [63:0] rdata_d;
	reg [63:0] rdata_q;
	reg word_enable32_q;
	wire [HartSelLen - 1:0] hartsel;
	wire [HartSelLen - 1:0] wdata_hartsel;
	assign hartsel = hartsel_i[HartSelLen - 1:0];
	assign wdata_hartsel = wdata_i[HartSelLen - 1:0];
	wire [NrHartsAligned - 1:0] resumereq_aligned;
	wire [NrHartsAligned - 1:0] haltreq_aligned;
	reg [NrHartsAligned - 1:0] halted_d_aligned;
	wire [NrHartsAligned - 1:0] halted_q_aligned;
	reg [NrHartsAligned - 1:0] halted_aligned;
	wire [NrHartsAligned - 1:0] resumereq_wdata_aligned;
	reg [NrHartsAligned - 1:0] resuming_d_aligned;
	wire [NrHartsAligned - 1:0] resuming_q_aligned;
	function automatic [NrHartsAligned - 1:0] sv2v_cast_24E98;
		input reg [NrHartsAligned - 1:0] inp;
		sv2v_cast_24E98 = inp;
	endfunction
	assign resumereq_aligned = sv2v_cast_24E98(resumereq_i);
	assign haltreq_aligned = sv2v_cast_24E98(haltreq_i);
	assign resumereq_wdata_aligned = sv2v_cast_24E98(resumereq_i);
	assign halted_q_aligned = sv2v_cast_24E98(halted_q);
	function automatic [NrHarts - 1:0] sv2v_cast_5B351;
		input reg [NrHarts - 1:0] inp;
		sv2v_cast_5B351 = inp;
	endfunction
	assign halted_d = sv2v_cast_5B351(halted_d_aligned);
	assign resuming_q_aligned = sv2v_cast_24E98(resuming_q);
	assign resuming_d = sv2v_cast_5B351(resuming_d_aligned);
	wire fwd_rom_d;
	reg fwd_rom_q;
	wire [23:0] ac_ar;
	function automatic [23:0] sv2v_cast_24;
		input reg [23:0] inp;
		sv2v_cast_24 = inp;
	endfunction
	assign ac_ar = sv2v_cast_24(cmd_i[23-:24]);
	assign debug_req_o = haltreq_i;
	assign halted_o = halted_q;
	assign resuming_o = resuming_q;
	assign progbuf = progbuf_i;
	reg [1:0] state_d;
	reg [1:0] state_q;
	always @(*) begin : p_hart_ctrl_queue
		if (_sv2v_0)
			;
		cmderror_valid_o = 1'b0;
		cmderror_o = 3'd0;
		state_d = state_q;
		go = 1'b0;
		resume = 1'b0;
		cmdbusy_o = 1'b1;
		case (state_q)
			2'd0: begin
				cmdbusy_o = 1'b0;
				if ((cmd_valid_i && halted_q_aligned[hartsel]) && !unsupported_command)
					state_d = 2'd1;
				else if (cmd_valid_i) begin
					cmderror_valid_o = 1'b1;
					cmderror_o = 3'd4;
				end
				if (((resumereq_aligned[hartsel] && !resuming_q_aligned[hartsel]) && !haltreq_aligned[hartsel]) && halted_q_aligned[hartsel])
					state_d = 2'd2;
			end
			2'd1: begin
				cmdbusy_o = 1'b1;
				go = 1'b1;
				if (going)
					state_d = 2'd3;
			end
			2'd2: begin
				cmdbusy_o = 1'b1;
				resume = 1'b1;
				if (resuming_q_aligned[hartsel])
					state_d = 2'd0;
			end
			2'd3: begin
				cmdbusy_o = 1'b1;
				go = 1'b0;
				if (halted_aligned[hartsel])
					state_d = 2'd0;
			end
			default:
				;
		endcase
		if (unsupported_command && cmd_valid_i) begin
			cmderror_valid_o = 1'b1;
			cmderror_o = 3'd2;
		end
		if (exception) begin
			cmderror_valid_o = 1'b1;
			cmderror_o = 3'd3;
		end
		if (ndmreset_i) begin
			state_d = 2'd0;
			go = 1'b0;
			resume = 1'b0;
		end
	end
	wire [63:0] word_mux;
	assign word_mux = (fwd_rom_q ? rom_rdata : rdata_q);
	generate
		if (BusWidth == 64) begin : gen_word_mux64
			assign rdata_o = word_mux;
		end
		else begin : gen_word_mux32
			assign rdata_o = (word_enable32_q ? word_mux[32+:32] : word_mux[0+:32]);
		end
	endgenerate
	reg [63:0] data_bits;
	reg [63:0] rdata;
	localparam [63:0] dm_HaltAddress = 64'h0000000000000800;
	localparam [63:0] dm_ResumeAddress = 2056;
	function automatic [31:0] dm_jal;
		input reg [4:0] rd;
		input reg [20:0] imm;
		dm_jal = {imm[20], imm[10:1], imm[11], imm[19:12], rd, 7'h6f};
	endfunction
	function automatic [20:0] sv2v_cast_21;
		input reg [20:0] inp;
		sv2v_cast_21 = inp;
	endfunction
	function automatic [$clog2(4'h2) - 1:0] sv2v_cast_7FB6B;
		input reg [$clog2(4'h2) - 1:0] inp;
		sv2v_cast_7FB6B = inp;
	endfunction
	function automatic [$clog2(5'h08) - 1:0] sv2v_cast_3C70D;
		input reg [$clog2(5'h08) - 1:0] inp;
		sv2v_cast_3C70D = inp;
	endfunction
	function automatic [2:0] sv2v_cast_3;
		input reg [2:0] inp;
		sv2v_cast_3 = inp;
	endfunction
	function automatic [11:0] sv2v_cast_C7572;
		input reg [11:0] inp;
		sv2v_cast_C7572 = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		(* xprop_off *)
		begin : p_rw_logic
			halted_d_aligned = sv2v_cast_24E98(halted_q);
			resuming_d_aligned = sv2v_cast_24E98(resuming_q);
			rdata_d = rdata_q;
			data_bits = data_i;
			rdata = 1'sb0;
			data_valid_o = 1'b0;
			exception = 1'b0;
			halted_aligned = 1'sb0;
			going = 1'b0;
			if (clear_resumeack_i)
				resuming_d_aligned[hartsel] = 1'b0;
			if (req_i) begin
				if (we_i) begin
					if (addr_i[11:0] == HaltedAddr) begin
						halted_aligned[wdata_hartsel] = 1'b1;
						halted_d_aligned[wdata_hartsel] = 1'b1;
					end
					else if (addr_i[11:0] == GoingAddr)
						going = 1'b1;
					else if (addr_i[11:0] == ResumingAddr) begin
						halted_d_aligned[wdata_hartsel] = 1'b0;
						resuming_d_aligned[wdata_hartsel] = 1'b1;
					end
					else if (addr_i[11:0] == ExceptionAddr)
						exception = 1'b1;
					else if ((DataBaseAddr <= addr_i[11:0]) && (DataEndAddr >= addr_i[11:0])) begin
						data_valid_o = 1'b1;
						begin : sv2v_autoblock_1
							reg signed [31:0] dc;
							for (dc = 0; dc < dm_DataCount; dc = dc + 1)
								if ((addr_i[11:2] - DataBaseAddr[11:2]) == dc) begin : sv2v_autoblock_2
									reg signed [31:0] i;
									for (i = 0; i < (BusWidth / 8); i = i + 1)
										if (be_i[i]) begin
											if (i > 3) begin
												if ((dc + 1) < dm_DataCount)
													data_bits[((dc + 1) * 32) + ((i - 4) * 8)+:8] = wdata_i[i * 8+:8];
											end
											else
												data_bits[(dc * 32) + (i * 8)+:8] = wdata_i[i * 8+:8];
										end
								end
						end
					end
				end
				else if (addr_i[11:0] == WhereToAddr) begin
					if (resumereq_wdata_aligned[wdata_hartsel])
						rdata_d = {32'b00000000000000000000000000000000, dm_jal(1'sb0, sv2v_cast_21(dm_ResumeAddress[11:0]) - sv2v_cast_21(WhereToAddr))};
					if (cmdbusy_o) begin
						if (((cmd_i[31-:8] == 8'h00) && !ac_ar[17]) && ac_ar[18])
							rdata_d = {32'b00000000000000000000000000000000, dm_jal(1'sb0, sv2v_cast_21(ProgBufBaseAddr) - sv2v_cast_21(WhereToAddr))};
						else
							rdata_d = {32'b00000000000000000000000000000000, dm_jal(1'sb0, sv2v_cast_21(AbstractCmdBaseAddr) - sv2v_cast_21(WhereToAddr))};
					end
				end
				else if ((DataBaseAddr <= addr_i[11:0]) && (DataEndAddr >= addr_i[11:0]))
					rdata_d = {data_i[sv2v_cast_7FB6B(((addr_i[11:3] - DataBaseAddr[11:3]) << 1) + 1'b1) * 32+:32], data_i[sv2v_cast_7FB6B((addr_i[11:3] - DataBaseAddr[11:3]) << 1) * 32+:32]};
				else if ((ProgBufBaseAddr <= addr_i[11:0]) && (ProgBufEndAddr >= addr_i[11:0]))
					rdata_d = progbuf[sv2v_cast_3C70D(addr_i[11:3] - ProgBufBaseAddr[11:3]) * 64+:64];
				else if ((AbstractCmdBaseAddr <= addr_i[11:0]) && (AbstractCmdEndAddr >= addr_i[11:0]))
					rdata_d = abstract_cmd[sv2v_cast_3(addr_i[11:3] - AbstractCmdBaseAddr[11:3]) * 64+:64];
				else if ((FlagsBaseAddr <= addr_i[11:0]) && (FlagsEndAddr >= addr_i[11:0])) begin
					if (({addr_i[11:3], 3'b000} - FlagsBaseAddr[11:0]) == (sv2v_cast_C7572(hartsel) & {{9 {1'b1}}, 3'b000}))
						rdata[(sv2v_cast_C7572(hartsel) & sv2v_cast_C7572(3'b111)) * 8+:8] = {6'b000000, resume, go};
					rdata_d = rdata;
				end
			end
			if (ndmreset_i) begin
				halted_d_aligned = 1'sb0;
				resuming_d_aligned = 1'sb0;
			end
			data_o = data_bits;
		end
	end
	function automatic [31:0] dm_auipc;
		input reg [4:0] rd;
		input reg [20:0] imm;
		dm_auipc = {imm[20], imm[10:1], imm[11], imm[19:12], rd, 7'h17};
	endfunction
	function automatic [31:0] dm_csrr;
		input reg [11:0] csr;
		input reg [4:0] dest;
		dm_csrr = {csr, 8'h02, dest, 7'h73};
	endfunction
	function automatic [31:0] dm_csrw;
		input reg [11:0] csr;
		input reg [4:0] rs1;
		dm_csrw = {csr, rs1, 15'h1073};
	endfunction
	function automatic [31:0] dm_ebreak;
		input reg _sv2v_unused;
		dm_ebreak = 32'h00100073;
	endfunction
	function automatic [31:0] dm_float_load;
		input reg [2:0] size;
		input reg [4:0] dest;
		input reg [4:0] base;
		input reg [11:0] offset;
		dm_float_load = {offset[11:0], base, size, dest, 7'b0000111};
	endfunction
	function automatic [31:0] dm_float_store;
		input reg [2:0] size;
		input reg [4:0] src;
		input reg [4:0] base;
		input reg [11:0] offset;
		dm_float_store = {offset[11:5], src, base, size, offset[4:0], 7'b0100111};
	endfunction
	function automatic [31:0] dm_illegal;
		input reg _sv2v_unused;
		dm_illegal = 32'h00000000;
	endfunction
	function automatic [31:0] dm_load;
		input reg [2:0] size;
		input reg [4:0] dest;
		input reg [4:0] base;
		input reg [11:0] offset;
		dm_load = {offset[11:0], base, size, dest, 7'h03};
	endfunction
	function automatic [31:0] dm_nop;
		input reg _sv2v_unused;
		dm_nop = 32'h00000013;
	endfunction
	function automatic [31:0] dm_slli;
		input reg [4:0] rd;
		input reg [4:0] rs1;
		input reg [5:0] shamt;
		dm_slli = {6'b000000, shamt[5:0], rs1, 3'h1, rd, 7'h13};
	endfunction
	function automatic [31:0] dm_srli;
		input reg [4:0] rd;
		input reg [4:0] rs1;
		input reg [5:0] shamt;
		dm_srli = {6'b000000, shamt[5:0], rs1, 3'h5, rd, 7'h13};
	endfunction
	function automatic [31:0] dm_store;
		input reg [2:0] size;
		input reg [4:0] src;
		input reg [4:0] base;
		input reg [11:0] offset;
		dm_store = {offset[11:5], src, base, size, offset[4:0], 7'h23};
	endfunction
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	always @(*) begin : p_abstract_cmd_rom
		if (_sv2v_0)
			;
		unsupported_command = 1'b0;
		abstract_cmd[31-:32] = dm_illegal(0);
		abstract_cmd[63-:32] = (HasSndScratch ? dm_auipc(5'd10, 1'sb0) : dm_nop(0));
		abstract_cmd[95-:32] = (HasSndScratch ? dm_srli(5'd10, 5'd10, 6'd12) : dm_nop(0));
		abstract_cmd[127-:32] = (HasSndScratch ? dm_slli(5'd10, 5'd10, 6'd12) : dm_nop(0));
		abstract_cmd[159-:32] = dm_nop(0);
		abstract_cmd[191-:32] = dm_nop(0);
		abstract_cmd[223-:32] = dm_nop(0);
		abstract_cmd[255-:32] = dm_nop(0);
		abstract_cmd[287-:32] = (HasSndScratch ? dm_csrr(12'h7b3, 5'd10) : dm_nop(0));
		abstract_cmd[319-:32] = dm_ebreak(0);
		abstract_cmd[320+:192] = 1'sb0;
		case (cmd_i[31-:8])
			8'h00: begin
				if (((sv2v_cast_32(ac_ar[22-:3]) < MaxAar) && ac_ar[17]) && ac_ar[16]) begin
					abstract_cmd[31-:32] = (HasSndScratch ? dm_csrw(12'h7b3, 5'd10) : dm_nop(0));
					if (ac_ar[15:14] != {2 {1'sb0}}) begin
						abstract_cmd[31-:32] = dm_ebreak(0);
						unsupported_command = 1'b1;
					end
					else if (((HasSndScratch && ac_ar[12]) && !ac_ar[5]) && (ac_ar[4:0] == 5'd10)) begin
						abstract_cmd[159-:32] = dm_csrw(12'h7b2, 5'd8);
						abstract_cmd[191-:32] = dm_load(ac_ar[22-:3], 5'd8, LoadBaseAddr, dm_DataAddr);
						abstract_cmd[223-:32] = dm_csrw(12'h7b3, 5'd8);
						abstract_cmd[255-:32] = dm_csrr(12'h7b2, 5'd8);
					end
					else if (ac_ar[12]) begin
						if (ac_ar[5])
							abstract_cmd[159-:32] = dm_float_load(ac_ar[22-:3], ac_ar[4:0], LoadBaseAddr, dm_DataAddr);
						else
							abstract_cmd[159-:32] = dm_load(ac_ar[22-:3], ac_ar[4:0], LoadBaseAddr, dm_DataAddr);
					end
					else begin
						abstract_cmd[159-:32] = dm_csrw(12'h7b2, 5'd8);
						abstract_cmd[191-:32] = dm_load(ac_ar[22-:3], 5'd8, LoadBaseAddr, dm_DataAddr);
						abstract_cmd[223-:32] = dm_csrw(ac_ar[11:0], 5'd8);
						abstract_cmd[255-:32] = dm_csrr(12'h7b2, 5'd8);
					end
				end
				else if (((sv2v_cast_32(ac_ar[22-:3]) < MaxAar) && ac_ar[17]) && !ac_ar[16]) begin
					abstract_cmd[31-:32] = (HasSndScratch ? dm_csrw(12'h7b3, LoadBaseAddr) : dm_nop(0));
					if (ac_ar[15:14] != {2 {1'sb0}}) begin
						abstract_cmd[31-:32] = dm_ebreak(0);
						unsupported_command = 1'b1;
					end
					else if (((HasSndScratch && ac_ar[12]) && !ac_ar[5]) && (ac_ar[4:0] == 5'd10)) begin
						abstract_cmd[159-:32] = dm_csrw(12'h7b2, 5'd8);
						abstract_cmd[191-:32] = dm_csrr(12'h7b3, 5'd8);
						abstract_cmd[223-:32] = dm_store(ac_ar[22-:3], 5'd8, LoadBaseAddr, dm_DataAddr);
						abstract_cmd[255-:32] = dm_csrr(12'h7b2, 5'd8);
					end
					else if (ac_ar[12]) begin
						if (ac_ar[5])
							abstract_cmd[159-:32] = dm_float_store(ac_ar[22-:3], ac_ar[4:0], LoadBaseAddr, dm_DataAddr);
						else
							abstract_cmd[159-:32] = dm_store(ac_ar[22-:3], ac_ar[4:0], LoadBaseAddr, dm_DataAddr);
					end
					else begin
						abstract_cmd[159-:32] = dm_csrw(12'h7b2, 5'd8);
						abstract_cmd[191-:32] = dm_csrr(ac_ar[11:0], 5'd8);
						abstract_cmd[223-:32] = dm_store(ac_ar[22-:3], 5'd8, LoadBaseAddr, dm_DataAddr);
						abstract_cmd[255-:32] = dm_csrr(12'h7b2, 5'd8);
					end
				end
				else if ((sv2v_cast_32(ac_ar[22-:3]) >= MaxAar) || (ac_ar[19] == 1'b1)) begin
					abstract_cmd[31-:32] = dm_ebreak(0);
					unsupported_command = 1'b1;
				end
				if (ac_ar[18] && !unsupported_command)
					abstract_cmd[319-:32] = dm_nop(0);
			end
			default: begin
				abstract_cmd[31-:32] = dm_ebreak(0);
				unsupported_command = 1'b1;
			end
		endcase
	end
	wire [63:0] rom_addr;
	function automatic [63:0] sv2v_cast_64;
		input reg [63:0] inp;
		sv2v_cast_64 = inp;
	endfunction
	assign rom_addr = sv2v_cast_64(addr_i);
	generate
		if (HasSndScratch) begin : gen_rom_snd_scratch
			debug_rom i_debug_rom(
				.clk_i(clk_i),
				.rst_ni(rst_ni),
				.req_i(req_i),
				.addr_i(rom_addr),
				.rdata_o(rom_rdata)
			);
		end
		else begin : gen_rom_one_scratch
			debug_rom_one_scratch i_debug_rom(
				.clk_i(clk_i),
				.rst_ni(rst_ni),
				.req_i(req_i),
				.addr_i(rom_addr),
				.rdata_o(rom_rdata)
			);
		end
	endgenerate
	assign fwd_rom_d = addr_i[11:0] >= dm_HaltAddress[11:0];
	always @(posedge clk_i or negedge rst_ni) begin : p_regs
		if (!rst_ni) begin
			fwd_rom_q <= 1'b0;
			rdata_q <= 1'sb0;
			state_q <= 2'd0;
			word_enable32_q <= 1'b0;
		end
		else begin
			fwd_rom_q <= fwd_rom_d;
			rdata_q <= rdata_d;
			state_q <= state_d;
			word_enable32_q <= addr_i[2];
		end
	end
	always @(posedge clk_i or negedge rst_ni)
		if (!rst_ni) begin
			halted_q <= 1'b0;
			resuming_q <= 1'b0;
		end
		else begin
			halted_q <= SelectableHarts & halted_d;
			resuming_q <= SelectableHarts & resuming_d;
		end
	initial _sv2v_0 = 0;
endmodule
module dm_obi_top (
	clk_i,
	rst_ni,
	testmode_i,
	ndmreset_o,
	dmactive_o,
	debug_req_o,
	unavailable_i,
	hartinfo_i,
	slave_req_i,
	slave_gnt_o,
	slave_we_i,
	slave_addr_i,
	slave_be_i,
	slave_wdata_i,
	slave_aid_i,
	slave_rvalid_o,
	slave_rdata_o,
	slave_rid_o,
	master_req_o,
	master_addr_o,
	master_we_o,
	master_wdata_o,
	master_be_o,
	master_gnt_i,
	master_rvalid_i,
	master_err_i,
	master_other_err_i,
	master_rdata_i,
	dmi_rst_ni,
	dmi_req_valid_i,
	dmi_req_ready_o,
	dmi_req_i,
	dmi_resp_valid_o,
	dmi_resp_ready_i,
	dmi_resp_o
);
	parameter [31:0] IdWidth = 1;
	parameter [31:0] NrHarts = 1;
	parameter [31:0] BusWidth = 32;
	parameter [31:0] DmBaseAddress = 'h1000;
	parameter [NrHarts - 1:0] SelectableHarts = {NrHarts {1'b1}};
	input wire clk_i;
	input wire rst_ni;
	input wire testmode_i;
	output wire ndmreset_o;
	output wire dmactive_o;
	output wire [NrHarts - 1:0] debug_req_o;
	input wire [NrHarts - 1:0] unavailable_i;
	input wire [(NrHarts * 32) - 1:0] hartinfo_i;
	input wire slave_req_i;
	output wire slave_gnt_o;
	input wire slave_we_i;
	input wire [BusWidth - 1:0] slave_addr_i;
	input wire [(BusWidth / 8) - 1:0] slave_be_i;
	input wire [BusWidth - 1:0] slave_wdata_i;
	input wire [IdWidth - 1:0] slave_aid_i;
	output wire slave_rvalid_o;
	output wire [BusWidth - 1:0] slave_rdata_o;
	output wire [IdWidth - 1:0] slave_rid_o;
	output wire master_req_o;
	output wire [BusWidth - 1:0] master_addr_o;
	output wire master_we_o;
	output wire [BusWidth - 1:0] master_wdata_o;
	output wire [(BusWidth / 8) - 1:0] master_be_o;
	input wire master_gnt_i;
	input wire master_rvalid_i;
	input wire master_err_i;
	input wire master_other_err_i;
	input wire [BusWidth - 1:0] master_rdata_i;
	input wire dmi_rst_ni;
	input wire dmi_req_valid_i;
	output wire dmi_req_ready_o;
	input wire [40:0] dmi_req_i;
	output wire dmi_resp_valid_o;
	input wire dmi_resp_ready_i;
	output wire [33:0] dmi_resp_o;
	reg slave_rvalid_q;
	reg [IdWidth - 1:0] slave_rid_q;
	dm_top #(
		.NrHarts(NrHarts),
		.BusWidth(BusWidth),
		.DmBaseAddress(DmBaseAddress),
		.SelectableHarts(SelectableHarts)
	) i_dm_top(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.testmode_i(testmode_i),
		.ndmreset_o(ndmreset_o),
		.dmactive_o(dmactive_o),
		.debug_req_o(debug_req_o),
		.unavailable_i(unavailable_i),
		.hartinfo_i(hartinfo_i),
		.slave_req_i(slave_req_i),
		.slave_we_i(slave_we_i),
		.slave_addr_i(slave_addr_i),
		.slave_be_i(slave_be_i),
		.slave_wdata_i(slave_wdata_i),
		.slave_rdata_o(slave_rdata_o),
		.master_req_o(master_req_o),
		.master_add_o(master_addr_o),
		.master_we_o(master_we_o),
		.master_wdata_o(master_wdata_o),
		.master_be_o(master_be_o),
		.master_gnt_i(master_gnt_i),
		.master_r_valid_i(master_rvalid_i),
		.master_r_err_i(master_err_i),
		.master_r_other_err_i(master_other_err_i),
		.master_r_rdata_i(master_rdata_i),
		.dmi_rst_ni(dmi_rst_ni),
		.dmi_req_valid_i(dmi_req_valid_i),
		.dmi_req_ready_o(dmi_req_ready_o),
		.dmi_req_i(dmi_req_i),
		.dmi_resp_valid_o(dmi_resp_valid_o),
		.dmi_resp_ready_i(dmi_resp_ready_i),
		.dmi_resp_o(dmi_resp_o)
	);
	always @(posedge clk_i or negedge rst_ni) begin : obi_regs
		if (!rst_ni) begin
			slave_rvalid_q <= 1'b0;
			slave_rid_q <= 'b0;
		end
		else if (slave_req_i && slave_gnt_o) begin
			slave_rvalid_q <= 1'b1;
			slave_rid_q <= slave_aid_i;
		end
		else
			slave_rvalid_q <= 1'b0;
	end
	assign slave_gnt_o = 1'b1;
	assign slave_rvalid_o = slave_rvalid_q;
	assign slave_rid_o = slave_rid_q;
endmodule
module dmi_jtag (
	clk_i,
	rst_ni,
	testmode_i,
	dmi_rst_no,
	dmi_req_o,
	dmi_req_valid_o,
	dmi_req_ready_i,
	dmi_resp_i,
	dmi_resp_ready_o,
	dmi_resp_valid_i,
	tck_i,
	tms_i,
	trst_ni,
	td_i,
	td_o,
	tdo_oe_o
);
	reg _sv2v_0;
	parameter [31:0] IdcodeValue = 32'h00000db3;
	input wire clk_i;
	input wire rst_ni;
	input wire testmode_i;
	(* keep *) output reg dmi_rst_no;
	(* keep *) output reg [40:0] dmi_req_o;
	(* keep *) output reg dmi_req_valid_o;
	(* keep *) input wire dmi_req_ready_i;
	(* keep *) input wire [33:0] dmi_resp_i;
	(* keep *) output wire dmi_resp_ready_o;
	(* keep *) input wire dmi_resp_valid_i;
	input wire tck_i;
	input wire tms_i;
	input wire trst_ni;
	input wire td_i;
	output wire td_o;
	output wire tdo_oe_o;
	reg [1:0] tck_last;
	wire tck_posedge;
	wire tck_negedge;
	always @(posedge clk_i) tck_last <= {tck_last[0], tck_i};
	assign tck_posedge = tck_last == 2'b01;
	assign tck_negedge = tck_last == 2'b10;
	reg [1:0] error_d;
	reg [1:0] error_q;
	wire jtag_dmi_clear;
	wire dmi_clear;
	wire update;
	wire capture;
	wire shift;
	wire tdi;
	wire dtmcs_select;
	reg [31:0] dtmcs_q;
	assign dmi_clear = jtag_dmi_clear || ((dtmcs_select && update) && dtmcs_q[17]);
	reg [31:0] dtmcs_d;
	function automatic [30:0] sv2v_cast_31;
		input reg [30:0] inp;
		sv2v_cast_31 = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		dtmcs_d = dtmcs_q;
		if (capture) begin
			if (dtmcs_select)
				dtmcs_d = {20'h00001, error_q, 10'h071};
		end
		if (shift) begin
			if (dtmcs_select)
				dtmcs_d = {tdi, sv2v_cast_31(dtmcs_q >> 1)};
		end
	end
	always @(posedge clk_i or negedge trst_ni)
		if (!trst_ni)
			dtmcs_q <= 1'sb0;
		else if (tck_posedge)
			dtmcs_q <= dtmcs_d;
	wire dmi_select;
	wire dmi_tdo;
	wire [40:0] dmi_req;
	wire dmi_req_ready;
	reg dmi_req_valid;
	wire [33:0] dmi_resp;
	wire dmi_resp_valid;
	wire dmi_resp_ready;
	reg [2:0] state_d;
	reg [2:0] state_q;
	reg [40:0] dr_d;
	reg [40:0] dr_q;
	reg [6:0] address_d;
	reg [6:0] address_q;
	reg [31:0] data_d;
	reg [31:0] data_q;
	wire [40:0] dmi;
	assign dmi = dr_q;
	assign dmi_req[40-:7] = address_q;
	assign dmi_req[31-:32] = data_q;
	assign dmi_req[33-:2] = (state_q == 3'd3 ? 2'h2 : 2'h1);
	assign dmi_resp_ready = 1'b1;
	reg error_dmi_busy;
	reg error_dmi_op_failed;
	function automatic [1:0] sv2v_cast_2;
		input reg [1:0] inp;
		sv2v_cast_2 = inp;
	endfunction
	always @(*) begin : p_fsm
		if (_sv2v_0)
			;
		error_dmi_busy = 1'b0;
		error_dmi_op_failed = 1'b0;
		state_d = state_q;
		address_d = address_q;
		data_d = data_q;
		error_d = error_q;
		dmi_req_valid = 1'b0;
		if (dmi_clear) begin
			state_d = 3'd0;
			data_d = 1'sb0;
			error_d = 2'h0;
			address_d = 1'sb0;
		end
		else begin
			case (state_q)
				3'd0:
					if ((dmi_select && update) && (error_q == 2'h0)) begin
						address_d = dmi[40-:7];
						data_d = dmi[33-:32];
						if (sv2v_cast_2(dmi[1-:2]) == 2'h1)
							state_d = 3'd1;
						else if (sv2v_cast_2(dmi[1-:2]) == 2'h2)
							state_d = 3'd3;
					end
				3'd1: begin
					dmi_req_valid = 1'b1;
					if (dmi_req_ready)
						state_d = 3'd2;
				end
				3'd2:
					if (dmi_resp_valid) begin
						case (dmi_resp[1-:2])
							2'h0: data_d = dmi_resp[33-:32];
							2'h2: begin
								data_d = 32'hdeadbeef;
								error_dmi_op_failed = 1'b1;
							end
							2'h3: begin
								data_d = 32'hb051b051;
								error_dmi_busy = 1'b1;
							end
							default: data_d = 32'hbaadc0de;
						endcase
						state_d = 3'd0;
					end
				3'd3: begin
					dmi_req_valid = 1'b1;
					if (dmi_req_ready)
						state_d = 3'd4;
				end
				3'd4:
					if (dmi_resp_valid) begin
						case (dmi_resp[1-:2])
							2'h2: error_dmi_op_failed = 1'b1;
							2'h3: error_dmi_busy = 1'b1;
							default:
								;
						endcase
						state_d = 3'd0;
					end
				default:
					if (dmi_resp_valid)
						state_d = 3'd0;
			endcase
			if (update && (state_q != 3'd0))
				error_dmi_busy = 1'b1;
			if (capture && |{state_q == 3'd1, state_q == 3'd2})
				error_dmi_busy = 1'b1;
			if (error_dmi_busy && (error_q == 2'h0))
				error_d = 2'h3;
			if (error_dmi_op_failed && (error_q == 2'h0))
				error_d = 2'h2;
			if ((update && dtmcs_q[16]) && dtmcs_select)
				error_d = 2'h0;
		end
	end
	assign dmi_tdo = dr_q[0];
	always @(*) begin : p_shift
		if (_sv2v_0)
			;
		dr_d = dr_q;
		if (dmi_clear)
			dr_d = 1'sb0;
		else begin
			if (capture) begin
				if (dmi_select) begin
					if ((error_q == 2'h0) && !error_dmi_busy)
						dr_d = {address_q, data_q, 2'h0};
					else if ((error_q == 2'h3) || error_dmi_busy)
						dr_d = {address_q, data_q, 2'h3};
				end
			end
			if (shift) begin
				if (dmi_select)
					dr_d = {tdi, dr_q[40:1]};
			end
		end
	end
	always @(posedge clk_i or negedge trst_ni)
		if (!trst_ni) begin
			dr_q <= 1'sb0;
			state_q <= 3'd0;
			address_q <= 1'sb0;
			data_q <= 1'sb0;
			error_q <= 2'h0;
		end
		else if (tck_posedge) begin
			dr_q <= dr_d;
			state_q <= state_d;
			address_q <= address_d;
			data_q <= data_d;
			error_q <= error_d;
		end
	dmi_jtag_tap #(
		.IrLength(5),
		.IdcodeValue(IdcodeValue)
	) i_dmi_jtag_tap(
		.clk_i(clk_i),
		.tck_posedge_i(tck_posedge),
		.tck_negedge_i(tck_negedge),
		.tms_i(tms_i),
		.trst_ni(trst_ni),
		.td_i(td_i),
		.td_o(td_o),
		.tdo_oe_o(tdo_oe_o),
		.testmode_i(testmode_i),
		.dmi_clear_o(jtag_dmi_clear),
		.update_o(update),
		.capture_o(capture),
		.shift_o(shift),
		.tdi_o(tdi),
		.dtmcs_select_o(dtmcs_select),
		.dtmcs_tdo_i(dtmcs_q[0]),
		.dmi_select_o(dmi_select),
		.dmi_tdo_i(dmi_tdo)
	);
	reg dmi_clear_last;
	always @(posedge clk_i) begin
		dmi_rst_no <= 1'b1;
		dmi_clear_last <= dmi_clear;
		if (tck_posedge && dmi_req_valid)
			dmi_req_valid_o <= 1'b1;
		else if (dmi_req_ready_i)
			dmi_req_valid_o <= 1'b0;
		dmi_req_o <= dmi_req;
		if (dmi_clear) begin
			if (!dmi_clear_last)
				dmi_rst_no <= 1'b0;
			dmi_req_valid_o <= 1'b0;
		end
	end
	assign dmi_resp = dmi_resp_i;
	assign dmi_resp_valid = dmi_resp_valid_i;
	assign dmi_resp_ready_o = dmi_resp_ready && tck_posedge;
	assign dmi_req_ready = dmi_req_ready_i;
	initial _sv2v_0 = 0;
endmodule
module dmi_jtag_tap (
	clk_i,
	tck_posedge_i,
	tck_negedge_i,
	tms_i,
	trst_ni,
	td_i,
	td_o,
	tdo_oe_o,
	testmode_i,
	dmi_clear_o,
	update_o,
	capture_o,
	shift_o,
	tdi_o,
	dtmcs_select_o,
	dtmcs_tdo_i,
	dmi_select_o,
	dmi_tdo_i
);
	reg _sv2v_0;
	parameter [31:0] IrLength = 5;
	parameter [31:0] IdcodeValue = 32'h00000001;
	input wire clk_i;
	input wire tck_posedge_i;
	input wire tck_negedge_i;
	input wire tms_i;
	input wire trst_ni;
	input wire td_i;
	output reg td_o;
	output reg tdo_oe_o;
	input wire testmode_i;
	output wire dmi_clear_o;
	output wire update_o;
	output wire capture_o;
	output wire shift_o;
	output wire tdi_o;
	output reg dtmcs_select_o;
	input wire dtmcs_tdo_i;
	output reg dmi_select_o;
	input wire dmi_tdo_i;
	reg [3:0] tap_state_q;
	reg [3:0] tap_state_d;
	reg update_dr;
	reg shift_dr;
	reg capture_dr;
	reg [IrLength - 1:0] jtag_ir_shift_d;
	reg [IrLength - 1:0] jtag_ir_shift_q;
	reg [IrLength - 1:0] jtag_ir_d;
	reg [IrLength - 1:0] jtag_ir_q;
	reg capture_ir;
	reg shift_ir;
	reg update_ir;
	reg test_logic_reset;
	function automatic [IrLength - 1:0] sv2v_cast_7FF5F;
		input reg [IrLength - 1:0] inp;
		sv2v_cast_7FF5F = inp;
	endfunction
	always @(*) begin : p_jtag
		if (_sv2v_0)
			;
		jtag_ir_shift_d = jtag_ir_shift_q;
		jtag_ir_d = jtag_ir_q;
		if (shift_ir)
			jtag_ir_shift_d = {td_i, jtag_ir_shift_q[IrLength - 1:1]};
		if (capture_ir)
			jtag_ir_shift_d = sv2v_cast_7FF5F(4'b0101);
		if (update_ir)
			jtag_ir_d = sv2v_cast_7FF5F(jtag_ir_shift_q);
		if (test_logic_reset) begin
			jtag_ir_shift_d = 1'sb0;
			jtag_ir_d = sv2v_cast_7FF5F('h1);
		end
	end
	always @(posedge clk_i or negedge trst_ni) begin : p_jtag_ir_reg
		if (!trst_ni) begin
			jtag_ir_shift_q <= 1'sb0;
			jtag_ir_q <= sv2v_cast_7FF5F('h1);
		end
		else if (tck_posedge_i) begin
			jtag_ir_shift_q <= jtag_ir_shift_d;
			jtag_ir_q <= jtag_ir_d;
		end
	end
	reg [31:0] idcode_d;
	reg [31:0] idcode_q;
	reg idcode_select;
	reg bypass_select;
	reg bypass_d;
	reg bypass_q;
	function automatic [30:0] sv2v_cast_31;
		input reg [30:0] inp;
		sv2v_cast_31 = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		idcode_d = idcode_q;
		bypass_d = bypass_q;
		if (capture_dr) begin
			if (idcode_select)
				idcode_d = IdcodeValue;
			if (bypass_select)
				bypass_d = 1'b0;
		end
		if (shift_dr) begin
			if (idcode_select)
				idcode_d = {td_i, sv2v_cast_31(idcode_q >> 1)};
			if (bypass_select)
				bypass_d = td_i;
		end
		if (test_logic_reset) begin
			idcode_d = IdcodeValue;
			bypass_d = 1'b0;
		end
	end
	always @(*) begin : p_data_reg_sel
		if (_sv2v_0)
			;
		dmi_select_o = 1'b0;
		dtmcs_select_o = 1'b0;
		idcode_select = 1'b0;
		bypass_select = 1'b0;
		case (jtag_ir_q)
			sv2v_cast_7FF5F('h0): bypass_select = 1'b1;
			sv2v_cast_7FF5F('h1): idcode_select = 1'b1;
			sv2v_cast_7FF5F('h10): dtmcs_select_o = 1'b1;
			sv2v_cast_7FF5F('h11): dmi_select_o = 1'b1;
			sv2v_cast_7FF5F('h1f): bypass_select = 1'b1;
			default: bypass_select = 1'b1;
		endcase
	end
	reg tdo_mux;
	always @(*) begin : p_out_sel
		if (_sv2v_0)
			;
		if (shift_ir)
			tdo_mux = jtag_ir_shift_q[0];
		else
			case (jtag_ir_q)
				sv2v_cast_7FF5F('h1): tdo_mux = idcode_q[0];
				sv2v_cast_7FF5F('h10): tdo_mux = dtmcs_tdo_i;
				sv2v_cast_7FF5F('h11): tdo_mux = dmi_tdo_i;
				default: tdo_mux = bypass_q;
			endcase
	end
	wire tck_n;
	wire tck_ni;
	always @(posedge clk_i or negedge trst_ni) begin : p_tdo_regs
		if (!trst_ni) begin
			td_o <= 1'b0;
			tdo_oe_o <= 1'b0;
		end
		else if (tck_negedge_i) begin
			td_o <= tdo_mux;
			tdo_oe_o <= shift_ir | shift_dr;
		end
	end
	always @(*) begin : p_tap_fsm
		if (_sv2v_0)
			;
		test_logic_reset = 1'b0;
		capture_dr = 1'b0;
		shift_dr = 1'b0;
		update_dr = 1'b0;
		capture_ir = 1'b0;
		shift_ir = 1'b0;
		update_ir = 1'b0;
		case (tap_state_q)
			4'd0: begin
				tap_state_d = (tms_i ? 4'd0 : 4'd1);
				test_logic_reset = 1'b1;
			end
			4'd1: tap_state_d = (tms_i ? 4'd2 : 4'd1);
			4'd2: tap_state_d = (tms_i ? 4'd9 : 4'd3);
			4'd3: begin
				capture_dr = 1'b1;
				tap_state_d = (tms_i ? 4'd5 : 4'd4);
			end
			4'd4: begin
				shift_dr = 1'b1;
				tap_state_d = (tms_i ? 4'd5 : 4'd4);
			end
			4'd5: tap_state_d = (tms_i ? 4'd8 : 4'd6);
			4'd6: tap_state_d = (tms_i ? 4'd7 : 4'd6);
			4'd7: tap_state_d = (tms_i ? 4'd8 : 4'd4);
			4'd8: begin
				update_dr = 1'b1;
				tap_state_d = (tms_i ? 4'd2 : 4'd1);
			end
			4'd9: tap_state_d = (tms_i ? 4'd0 : 4'd10);
			4'd10: begin
				capture_ir = 1'b1;
				tap_state_d = (tms_i ? 4'd12 : 4'd11);
			end
			4'd11: begin
				shift_ir = 1'b1;
				tap_state_d = (tms_i ? 4'd12 : 4'd11);
			end
			4'd12: tap_state_d = (tms_i ? 4'd15 : 4'd13);
			4'd13: tap_state_d = (tms_i ? 4'd14 : 4'd13);
			4'd14: tap_state_d = (tms_i ? 4'd15 : 4'd11);
			4'd15: begin
				update_ir = 1'b1;
				tap_state_d = (tms_i ? 4'd2 : 4'd1);
			end
			default:
				;
		endcase
	end
	always @(posedge clk_i or negedge trst_ni) begin : p_regs
		if (!trst_ni) begin
			tap_state_q <= 4'd1;
			idcode_q <= IdcodeValue;
			bypass_q <= 1'b0;
		end
		else if (tck_posedge_i) begin
			tap_state_q <= tap_state_d;
			idcode_q <= idcode_d;
			bypass_q <= bypass_d;
		end
	end
	assign tdi_o = td_i;
	assign update_o = update_dr;
	assign shift_o = shift_dr;
	assign capture_o = capture_dr;
	assign dmi_clear_o = test_logic_reset;
	initial _sv2v_0 = 0;
endmodule
module dm_sba (
	clk_i,
	rst_ni,
	dmactive_i,
	master_req_o,
	master_add_o,
	master_we_o,
	master_wdata_o,
	master_be_o,
	master_gnt_i,
	master_r_valid_i,
	master_r_err_i,
	master_r_other_err_i,
	master_r_rdata_i,
	sbaddress_i,
	sbaddress_write_valid_i,
	sbreadonaddr_i,
	sbaddress_o,
	sbautoincrement_i,
	sbaccess_i,
	sbreadondata_i,
	sbdata_i,
	sbdata_read_valid_i,
	sbdata_write_valid_i,
	sbdata_o,
	sbdata_valid_o,
	sbbusy_o,
	sberror_valid_o,
	sberror_o
);
	reg _sv2v_0;
	parameter [31:0] BusWidth = 32;
	parameter [0:0] ReadByteEnable = 1;
	input wire clk_i;
	input wire rst_ni;
	input wire dmactive_i;
	output wire master_req_o;
	output wire [BusWidth - 1:0] master_add_o;
	output wire master_we_o;
	output wire [BusWidth - 1:0] master_wdata_o;
	output wire [(BusWidth / 8) - 1:0] master_be_o;
	input wire master_gnt_i;
	input wire master_r_valid_i;
	input wire master_r_err_i;
	input wire master_r_other_err_i;
	input wire [BusWidth - 1:0] master_r_rdata_i;
	input wire [BusWidth - 1:0] sbaddress_i;
	input wire sbaddress_write_valid_i;
	input wire sbreadonaddr_i;
	output wire [BusWidth - 1:0] sbaddress_o;
	input wire sbautoincrement_i;
	input wire [2:0] sbaccess_i;
	input wire sbreadondata_i;
	input wire [BusWidth - 1:0] sbdata_i;
	input wire sbdata_read_valid_i;
	input wire sbdata_write_valid_i;
	output wire [BusWidth - 1:0] sbdata_o;
	output wire sbdata_valid_o;
	output wire sbbusy_o;
	output reg sberror_valid_o;
	output reg [2:0] sberror_o;
	localparam signed [31:0] BeIdxWidth = $clog2(BusWidth / 8);
	reg [2:0] state_d;
	reg [2:0] state_q;
	reg [BusWidth - 1:0] address;
	reg req;
	wire gnt;
	reg we;
	reg [(BusWidth / 8) - 1:0] be;
	reg [(BusWidth / 8) - 1:0] be_mask;
	reg [BeIdxWidth - 1:0] be_idx;
	assign sbbusy_o = state_q != 3'd0;
	function automatic signed [31:0] sv2v_cast_32_signed;
		input reg signed [31:0] inp;
		sv2v_cast_32_signed = inp;
	endfunction
	always @(*) begin : p_be_mask
		if (_sv2v_0)
			;
		be_mask = 1'sb0;
		case (sbaccess_i)
			3'b000: be_mask[be_idx] = 1'sb1;
			3'b001: be_mask[sv2v_cast_32_signed({be_idx[BeIdxWidth - 1:1], 1'b0})+:2] = 1'sb1;
			3'b010:
				if (BusWidth == 32'd64)
					be_mask[sv2v_cast_32_signed({be_idx[BeIdxWidth - 1], 2'h0})+:4] = 1'sb1;
				else
					be_mask = 1'sb1;
			3'b011: be_mask = 1'sb1;
			default:
				;
		endcase
	end
	wire [BusWidth - 1:0] sbaccess_mask;
	assign sbaccess_mask = {BusWidth {1'b1}} << sbaccess_i;
	reg addr_incr_en;
	wire [BusWidth - 1:0] addr_incr;
	function automatic [BusWidth - 1:0] sv2v_cast_4714E;
		input reg [BusWidth - 1:0] inp;
		sv2v_cast_4714E = inp;
	endfunction
	assign addr_incr = (addr_incr_en ? sv2v_cast_4714E(1'b1) << sbaccess_i : {BusWidth {1'sb0}});
	assign sbaddress_o = sbaddress_i + addr_incr;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	always @(*) begin : p_fsm
		if (_sv2v_0)
			;
		req = 1'b0;
		address = sbaddress_i;
		we = 1'b0;
		be = 1'sb0;
		be_idx = sbaddress_i[BeIdxWidth - 1:0];
		sberror_o = 1'sb0;
		sberror_valid_o = 1'b0;
		addr_incr_en = 1'b0;
		state_d = state_q;
		case (state_q)
			3'd0: begin
				if (sbaddress_write_valid_i && sbreadonaddr_i)
					state_d = 3'd1;
				if (sbdata_write_valid_i)
					state_d = 3'd2;
				if (sbdata_read_valid_i && sbreadondata_i)
					state_d = 3'd1;
			end
			3'd1: begin
				req = 1'b1;
				if (ReadByteEnable)
					be = be_mask;
				if (gnt)
					state_d = 3'd3;
			end
			3'd2: begin
				req = 1'b1;
				we = 1'b1;
				be = be_mask;
				if (gnt)
					state_d = 3'd4;
			end
			3'd3:
				if (sbdata_valid_o) begin
					state_d = 3'd0;
					addr_incr_en = sbautoincrement_i;
					if (master_r_other_err_i) begin
						sberror_valid_o = 1'b1;
						sberror_o = 3'd7;
					end
					else if (master_r_err_i) begin
						sberror_valid_o = 1'b1;
						sberror_o = 3'd2;
					end
				end
			3'd4:
				if (sbdata_valid_o) begin
					state_d = 3'd0;
					addr_incr_en = sbautoincrement_i;
					if (master_r_other_err_i) begin
						sberror_valid_o = 1'b1;
						sberror_o = 3'd7;
					end
					else if (master_r_err_i) begin
						sberror_valid_o = 1'b1;
						sberror_o = 3'd2;
					end
				end
			default: state_d = 3'd0;
		endcase
		if ((sv2v_cast_32(sbaccess_i) > BeIdxWidth) && (state_q != 3'd0)) begin
			req = 1'b0;
			state_d = 3'd0;
			sberror_valid_o = 1'b1;
			sberror_o = 3'd4;
		end
		if (|(sbaddress_i & ~sbaccess_mask) && (state_q != 3'd0)) begin
			req = 1'b0;
			state_d = 3'd0;
			sberror_valid_o = 1'b1;
			sberror_o = 3'd3;
		end
	end
	always @(posedge clk_i or negedge rst_ni) begin : p_regs
		if (!rst_ni)
			state_q <= 3'd0;
		else
			state_q <= state_d;
	end
	wire [BeIdxWidth - 1:0] be_idx_masked;
	function automatic [BeIdxWidth - 1:0] sv2v_cast_E32F6;
		input reg [BeIdxWidth - 1:0] inp;
		sv2v_cast_E32F6 = inp;
	endfunction
	assign be_idx_masked = be_idx & sv2v_cast_E32F6(sbaccess_mask);
	assign master_req_o = req;
	assign master_add_o = address[BusWidth - 1:0];
	assign master_we_o = we;
	assign master_wdata_o = sbdata_i[BusWidth - 1:0] << (8 * be_idx_masked);
	assign master_be_o = be[(BusWidth / 8) - 1:0];
	assign gnt = master_gnt_i;
	assign sbdata_valid_o = master_r_valid_i;
	assign sbdata_o = master_r_rdata_i[BusWidth - 1:0] >> (8 * be_idx_masked);
	initial _sv2v_0 = 0;
endmodule
module dm_top (
	clk_i,
	rst_ni,
	testmode_i,
	ndmreset_o,
	dmactive_o,
	debug_req_o,
	unavailable_i,
	hartinfo_i,
	slave_req_i,
	slave_we_i,
	slave_addr_i,
	slave_be_i,
	slave_wdata_i,
	slave_rdata_o,
	master_req_o,
	master_add_o,
	master_we_o,
	master_wdata_o,
	master_be_o,
	master_gnt_i,
	master_r_valid_i,
	master_r_err_i,
	master_r_other_err_i,
	master_r_rdata_i,
	dmi_rst_ni,
	dmi_req_valid_i,
	dmi_req_ready_o,
	dmi_req_i,
	dmi_resp_valid_o,
	dmi_resp_ready_i,
	dmi_resp_o
);
	parameter [31:0] NrHarts = 1;
	parameter [31:0] BusWidth = 32;
	parameter [31:0] DmBaseAddress = 'h1000;
	parameter [NrHarts - 1:0] SelectableHarts = {NrHarts {1'b1}};
	parameter [0:0] ReadByteEnable = 1;
	input wire clk_i;
	input wire rst_ni;
	input wire testmode_i;
	output wire ndmreset_o;
	output wire dmactive_o;
	output wire [NrHarts - 1:0] debug_req_o;
	input wire [NrHarts - 1:0] unavailable_i;
	input wire [(NrHarts * 32) - 1:0] hartinfo_i;
	input wire slave_req_i;
	input wire slave_we_i;
	input wire [BusWidth - 1:0] slave_addr_i;
	input wire [(BusWidth / 8) - 1:0] slave_be_i;
	input wire [BusWidth - 1:0] slave_wdata_i;
	output wire [BusWidth - 1:0] slave_rdata_o;
	output wire master_req_o;
	output wire [BusWidth - 1:0] master_add_o;
	output wire master_we_o;
	output wire [BusWidth - 1:0] master_wdata_o;
	output wire [(BusWidth / 8) - 1:0] master_be_o;
	input wire master_gnt_i;
	input wire master_r_valid_i;
	input wire master_r_err_i;
	input wire master_r_other_err_i;
	input wire [BusWidth - 1:0] master_r_rdata_i;
	input wire dmi_rst_ni;
	input wire dmi_req_valid_i;
	output wire dmi_req_ready_o;
	input wire [40:0] dmi_req_i;
	output wire dmi_resp_valid_o;
	input wire dmi_resp_ready_i;
	output wire [33:0] dmi_resp_o;
	wire [NrHarts - 1:0] halted;
	wire [NrHarts - 1:0] resumeack;
	wire [NrHarts - 1:0] haltreq;
	wire [NrHarts - 1:0] resumereq;
	wire clear_resumeack;
	wire cmd_valid;
	wire [31:0] cmd;
	wire cmderror_valid;
	wire [2:0] cmderror;
	wire cmdbusy;
	localparam [4:0] dm_ProgBufSize = 5'h08;
	wire [255:0] progbuf;
	localparam [3:0] dm_DataCount = 4'h2;
	wire [63:0] data_csrs_mem;
	wire [63:0] data_mem_csrs;
	wire data_valid;
	wire ndmreset;
	wire [19:0] hartsel;
	wire [BusWidth - 1:0] sbaddress_csrs_sba;
	wire [BusWidth - 1:0] sbaddress_sba_csrs;
	wire sbaddress_write_valid;
	wire sbreadonaddr;
	wire sbautoincrement;
	wire [2:0] sbaccess;
	wire sbreadondata;
	wire [BusWidth - 1:0] sbdata_write;
	wire sbdata_read_valid;
	wire sbdata_write_valid;
	wire [BusWidth - 1:0] sbdata_read;
	wire sbdata_valid;
	wire sbbusy;
	wire sberror_valid;
	wire [2:0] sberror;
	assign ndmreset_o = ndmreset;
	dm_csrs #(
		.NrHarts(NrHarts),
		.BusWidth(BusWidth),
		.SelectableHarts(SelectableHarts)
	) i_dm_csrs(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.testmode_i(testmode_i),
		.dmi_rst_ni(dmi_rst_ni),
		.dmi_req_valid_i(dmi_req_valid_i),
		.dmi_req_ready_o(dmi_req_ready_o),
		.dmi_req_i(dmi_req_i),
		.dmi_resp_valid_o(dmi_resp_valid_o),
		.dmi_resp_ready_i(dmi_resp_ready_i),
		.dmi_resp_o(dmi_resp_o),
		.ndmreset_o(ndmreset),
		.dmactive_o(dmactive_o),
		.hartsel_o(hartsel),
		.hartinfo_i(hartinfo_i),
		.halted_i(halted),
		.unavailable_i(unavailable_i),
		.resumeack_i(resumeack),
		.haltreq_o(haltreq),
		.resumereq_o(resumereq),
		.clear_resumeack_o(clear_resumeack),
		.cmd_valid_o(cmd_valid),
		.cmd_o(cmd),
		.cmderror_valid_i(cmderror_valid),
		.cmderror_i(cmderror),
		.cmdbusy_i(cmdbusy),
		.progbuf_o(progbuf),
		.data_i(data_mem_csrs),
		.data_valid_i(data_valid),
		.data_o(data_csrs_mem),
		.sbaddress_o(sbaddress_csrs_sba),
		.sbaddress_i(sbaddress_sba_csrs),
		.sbaddress_write_valid_o(sbaddress_write_valid),
		.sbreadonaddr_o(sbreadonaddr),
		.sbautoincrement_o(sbautoincrement),
		.sbaccess_o(sbaccess),
		.sbreadondata_o(sbreadondata),
		.sbdata_o(sbdata_write),
		.sbdata_read_valid_o(sbdata_read_valid),
		.sbdata_write_valid_o(sbdata_write_valid),
		.sbdata_i(sbdata_read),
		.sbdata_valid_i(sbdata_valid),
		.sbbusy_i(sbbusy),
		.sberror_valid_i(sberror_valid),
		.sberror_i(sberror)
	);
	dm_sba #(
		.BusWidth(BusWidth),
		.ReadByteEnable(ReadByteEnable)
	) i_dm_sba(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.dmactive_i(dmactive_o),
		.master_req_o(master_req_o),
		.master_add_o(master_add_o),
		.master_we_o(master_we_o),
		.master_wdata_o(master_wdata_o),
		.master_be_o(master_be_o),
		.master_gnt_i(master_gnt_i),
		.master_r_valid_i(master_r_valid_i),
		.master_r_err_i(master_r_err_i),
		.master_r_other_err_i(master_r_other_err_i),
		.master_r_rdata_i(master_r_rdata_i),
		.sbaddress_i(sbaddress_csrs_sba),
		.sbaddress_o(sbaddress_sba_csrs),
		.sbaddress_write_valid_i(sbaddress_write_valid),
		.sbreadonaddr_i(sbreadonaddr),
		.sbautoincrement_i(sbautoincrement),
		.sbaccess_i(sbaccess),
		.sbreadondata_i(sbreadondata),
		.sbdata_i(sbdata_write),
		.sbdata_read_valid_i(sbdata_read_valid),
		.sbdata_write_valid_i(sbdata_write_valid),
		.sbdata_o(sbdata_read),
		.sbdata_valid_o(sbdata_valid),
		.sbbusy_o(sbbusy),
		.sberror_valid_o(sberror_valid),
		.sberror_o(sberror)
	);
	dm_mem #(
		.NrHarts(NrHarts),
		.BusWidth(BusWidth),
		.SelectableHarts(SelectableHarts),
		.DmBaseAddress(DmBaseAddress)
	) i_dm_mem(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.debug_req_o(debug_req_o),
		.ndmreset_i(ndmreset),
		.hartsel_i(hartsel),
		.haltreq_i(haltreq),
		.resumereq_i(resumereq),
		.clear_resumeack_i(clear_resumeack),
		.halted_o(halted),
		.resuming_o(resumeack),
		.cmd_valid_i(cmd_valid),
		.cmd_i(cmd),
		.cmderror_valid_o(cmderror_valid),
		.cmderror_o(cmderror),
		.cmdbusy_o(cmdbusy),
		.progbuf_i(progbuf),
		.data_i(data_csrs_mem),
		.data_o(data_mem_csrs),
		.data_valid_o(data_valid),
		.req_i(slave_req_i),
		.we_i(slave_we_i),
		.addr_i(slave_addr_i),
		.wdata_i(slave_wdata_i),
		.be_i(slave_be_i),
		.rdata_o(slave_rdata_o)
	);
endmodule
module debug_rom (
	clk_i,
	rst_ni,
	req_i,
	addr_i,
	rdata_o
);
	reg _sv2v_0;
	input wire clk_i;
	input wire rst_ni;
	input wire req_i;
	input wire [63:0] addr_i;
	output reg [63:0] rdata_o;
	localparam [31:0] RomSize = 20;
	wire [1279:0] mem;
	assign mem = 1280'h7b2000737b2024737b30257310852823f1402473a79ff06f7b2024737b30257310052423001000737b2024737b30257310052c2300c5151300c5551300000517fd5ff06ffa0418e3002474134004440300a40433f140247302041c63001474134004440300a4043310852023f140247300c5151300c55513000005177b3510737b2410730ff0000f000000130500006f000000130840006f000000130180006f;
	wire [4:0] addr_d;
	reg [4:0] addr_q;
	assign addr_d = (req_i ? addr_i[7:3] : addr_q);
	always @(posedge clk_i or negedge rst_ni)
		if (!rst_ni)
			addr_q <= 1'sb0;
		else
			addr_q <= addr_d;
	function automatic [4:0] sv2v_cast_20B01;
		input reg [4:0] inp;
		sv2v_cast_20B01 = inp;
	endfunction
	always @(*) begin : p_outmux
		if (_sv2v_0)
			;
		rdata_o = 1'sb0;
		if (addr_q < sv2v_cast_20B01(RomSize))
			rdata_o = mem[addr_q * 64+:64];
	end
	initial _sv2v_0 = 0;
endmodule
module fifo_v2_A387C (
	clk_i,
	rst_ni,
	flush_i,
	testmode_i,
	full_o,
	empty_o,
	alm_full_o,
	alm_empty_o,
	data_i,
	push_i,
	data_o,
	pop_i
);
	parameter [0:0] FALL_THROUGH = 1'b0;
	parameter [31:0] DATA_WIDTH = 32;
	parameter [31:0] DEPTH = 8;
	parameter [31:0] ALM_EMPTY_TH = 1;
	parameter [31:0] ALM_FULL_TH = 1;
	parameter [31:0] ADDR_DEPTH = (DEPTH > 1 ? $clog2(DEPTH) : 1);
	input wire clk_i;
	input wire rst_ni;
	input wire flush_i;
	input wire testmode_i;
	output wire full_o;
	output wire empty_o;
	output wire alm_full_o;
	output wire alm_empty_o;
	input wire [33:0] data_i;
	input wire push_i;
	output wire [33:0] data_o;
	input wire pop_i;
	wire [ADDR_DEPTH - 1:0] usage;
	generate
		if (DEPTH == 0) begin : genblk1
			assign alm_full_o = 1'b0;
			assign alm_empty_o = 1'b0;
		end
		else begin : genblk1
			assign alm_full_o = usage >= ALM_FULL_TH[ADDR_DEPTH - 1:0];
			assign alm_empty_o = usage <= ALM_EMPTY_TH[ADDR_DEPTH - 1:0];
		end
	endgenerate
	fifo_v3_B84BC #(
		.FALL_THROUGH(FALL_THROUGH),
		.DATA_WIDTH(DATA_WIDTH),
		.DEPTH(DEPTH)
	) i_fifo_v3(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.flush_i(flush_i),
		.testmode_i(testmode_i),
		.full_o(full_o),
		.empty_o(empty_o),
		.usage_o(usage),
		.data_i(data_i),
		.push_i(push_i),
		.data_o(data_o),
		.pop_i(pop_i)
	);
endmodule
module fifo_v3_B84BC (
	clk_i,
	rst_ni,
	flush_i,
	testmode_i,
	full_o,
	empty_o,
	usage_o,
	data_i,
	push_i,
	data_o,
	pop_i
);
	reg _sv2v_0;
	parameter [0:0] FALL_THROUGH = 1'b0;
	parameter [31:0] DATA_WIDTH = 32;
	parameter [31:0] DEPTH = 8;
	parameter [31:0] ADDR_DEPTH = (DEPTH > 1 ? $clog2(DEPTH) : 1);
	input wire clk_i;
	input wire rst_ni;
	input wire flush_i;
	input wire testmode_i;
	output wire full_o;
	output wire empty_o;
	output wire [ADDR_DEPTH - 1:0] usage_o;
	input wire [33:0] data_i;
	input wire push_i;
	output reg [33:0] data_o;
	input wire pop_i;
	localparam [31:0] FIFO_DEPTH = (DEPTH > 0 ? DEPTH : 1);
	reg gate_clock;
	reg [ADDR_DEPTH - 1:0] read_pointer_n;
	reg [ADDR_DEPTH - 1:0] read_pointer_q;
	reg [ADDR_DEPTH - 1:0] write_pointer_n;
	reg [ADDR_DEPTH - 1:0] write_pointer_q;
	reg [ADDR_DEPTH:0] status_cnt_n;
	reg [ADDR_DEPTH:0] status_cnt_q;
	reg [(FIFO_DEPTH * 34) - 1:0] mem_n;
	reg [(FIFO_DEPTH * 34) - 1:0] mem_q;
	assign usage_o = status_cnt_q[ADDR_DEPTH - 1:0];
	generate
		if (DEPTH == 0) begin : genblk1
			assign empty_o = ~push_i;
			assign full_o = ~pop_i;
		end
		else begin : genblk1
			assign full_o = status_cnt_q == FIFO_DEPTH[ADDR_DEPTH:0];
			assign empty_o = (status_cnt_q == 0) & ~(FALL_THROUGH & push_i);
		end
	endgenerate
	always @(*) begin : read_write_comb
		if (_sv2v_0)
			;
		read_pointer_n = read_pointer_q;
		write_pointer_n = write_pointer_q;
		status_cnt_n = status_cnt_q;
		data_o = (DEPTH == 0 ? data_i : mem_q[read_pointer_q * 34+:34]);
		mem_n = mem_q;
		gate_clock = 1'b1;
		if (push_i && ~full_o) begin
			mem_n[write_pointer_q * 34+:34] = data_i;
			gate_clock = 1'b0;
			if (write_pointer_q == (FIFO_DEPTH[ADDR_DEPTH - 1:0] - 1))
				write_pointer_n = 1'sb0;
			else
				write_pointer_n = write_pointer_q + 1;
			status_cnt_n = status_cnt_q + 1;
		end
		if (pop_i && ~empty_o) begin
			if (read_pointer_n == (FIFO_DEPTH[ADDR_DEPTH - 1:0] - 1))
				read_pointer_n = 1'sb0;
			else
				read_pointer_n = read_pointer_q + 1;
			status_cnt_n = status_cnt_q - 1;
		end
		if (((push_i && pop_i) && ~full_o) && ~empty_o)
			status_cnt_n = status_cnt_q;
		if ((FALL_THROUGH && (status_cnt_q == 0)) && push_i) begin
			data_o = data_i;
			if (pop_i) begin
				status_cnt_n = status_cnt_q;
				read_pointer_n = read_pointer_q;
				write_pointer_n = write_pointer_q;
			end
		end
	end
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni) begin
			read_pointer_q <= 1'sb0;
			write_pointer_q <= 1'sb0;
			status_cnt_q <= 1'sb0;
		end
		else if (flush_i) begin
			read_pointer_q <= 1'sb0;
			write_pointer_q <= 1'sb0;
			status_cnt_q <= 1'sb0;
		end
		else begin
			read_pointer_q <= read_pointer_n;
			write_pointer_q <= write_pointer_n;
			status_cnt_q <= status_cnt_n;
		end
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni)
			mem_q <= 1'sb0;
		else if (!gate_clock)
			mem_q <= mem_n;
	initial _sv2v_0 = 0;
endmodule
module dm_wrap (
	clk_i,
	rst_ni,
	ndmreset_o,
	debug_req_o,
	dm_req_i,
	dm_gnt_o,
	dm_we_i,
	dm_addr_i,
	dm_be_i,
	dm_wdata_i,
	dm_rdata_o,
	dm_rvalid_o,
	sb_req_o,
	sb_addr_o,
	sb_we_o,
	sb_wdata_o,
	sb_be_o,
	sb_gnt_i,
	sb_rvalid_i,
	sb_rdata_i,
	tck_i,
	tms_i,
	trst_ni,
	tdi_i,
	tdo_o,
	tdo_oe
);
	input wire clk_i;
	input wire rst_ni;
	output wire ndmreset_o;
	output wire debug_req_o;
	input wire dm_req_i;
	output wire dm_gnt_o;
	input wire dm_we_i;
	input wire [31:0] dm_addr_i;
	input wire [3:0] dm_be_i;
	input wire [31:0] dm_wdata_i;
	output wire [31:0] dm_rdata_o;
	output wire [31:0] dm_rvalid_o;
	output wire sb_req_o;
	output wire [31:0] sb_addr_o;
	output wire sb_we_o;
	output wire [31:0] sb_wdata_o;
	output wire [3:0] sb_be_o;
	input wire sb_gnt_i;
	input wire sb_rvalid_i;
	input wire [31:0] sb_rdata_i;
	input wire tck_i;
	input wire tms_i;
	input wire trst_ni;
	input wire tdi_i;
	output wire tdo_o;
	output wire tdo_oe;
	wire debug_req_ready;
	wire [33:0] debug_resp;
	wire jtag_req_valid;
	wire [40:0] jtag_dmi_req;
	wire jtag_resp_ready;
	wire jtag_resp_valid;
	localparam [31:0] sv2v_uu_dm_top_i_NrHarts = 1;
	localparam [31:0] sv2v_uu_dm_top_i_ext_hartinfo_i_0 = 1'sb0;
	dm_obi_top #(
		.NrHarts(1),
		.BusWidth(32),
		.SelectableHarts(1)
	) dm_top_i(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.testmode_i(0),
		.ndmreset_o(ndmreset_o),
		.debug_req_o(debug_req_o),
		.unavailable_i(0),
		.hartinfo_i(sv2v_uu_dm_top_i_ext_hartinfo_i_0),
		.slave_req_i(dm_req_i),
		.slave_gnt_o(dm_gnt_o),
		.slave_we_i(dm_we_i),
		.slave_addr_i(dm_addr_i),
		.slave_be_i(dm_be_i),
		.slave_wdata_i(dm_wdata_i),
		.slave_rdata_o(dm_rdata_o),
		.slave_rvalid_o(dm_rvalid_o),
		.master_req_o(sb_req_o),
		.master_addr_o(sb_addr_o),
		.master_we_o(sb_we_o),
		.master_wdata_o(sb_wdata_o),
		.master_be_o(sb_be_o),
		.master_gnt_i(sb_gnt_i),
		.master_rvalid_i(sb_rvalid_i),
		.master_rdata_i(sb_rdata_i),
		.dmi_rst_ni(rst_ni),
		.dmi_req_valid_i(jtag_req_valid),
		.dmi_req_ready_o(debug_req_ready),
		.dmi_req_i(jtag_dmi_req),
		.dmi_resp_valid_o(jtag_resp_valid),
		.dmi_resp_ready_i(jtag_resp_ready),
		.dmi_resp_o(debug_resp)
	);
	dmi_jtag #(.IdcodeValue(32'h249511c3)) dmi_jtag_i(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.testmode_i(0),
		.dmi_req_o(jtag_dmi_req),
		.dmi_req_valid_o(jtag_req_valid),
		.dmi_req_ready_i(debug_req_ready),
		.dmi_resp_i(debug_resp),
		.dmi_resp_ready_o(jtag_resp_ready),
		.dmi_resp_valid_i(jtag_resp_valid),
		.tck_i(tck_i),
		.tms_i(tms_i),
		.trst_ni(trst_ni),
		.td_i(tdi_i),
		.td_o(tdo_o),
		.tdo_oe_o(tdo_oe)
	);
endmodule