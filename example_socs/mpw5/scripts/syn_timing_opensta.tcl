set CALIB $::env(HOME)/openrcx-calibration
set PDK sky130B
set BUILD_DIR build/sky130
set PDK_DIR $::env(HOME)/mpw4/thirdparty/open_pdk/C4M.Sky130

read_liberty $PDK_DIR/libs.ref/StdCellLib/liberty/StdCellLib_slow.lib
read_verilog $BUILD_DIR/user_project_core_mpw5_syn.v
link_design user_project_core_mpw5

create_clock -name sys_clk -period 10000 -waveform {0 5000} [get_ports {io_in[0]}]
set_propagated_clock [get_clocks {sys_clk}]

report_checks
report_tns
report_wns

exit
