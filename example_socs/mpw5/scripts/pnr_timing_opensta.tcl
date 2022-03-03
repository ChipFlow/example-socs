set CALIB $::env(HOME)/openrcx-calibration
set PDK sky130B
set BUILD_DIR build/sky130
set PDK_DIR $::env(HOME)/mpw4/thirdparty/open_pdk/C4M.Sky130

read_liberty $PDK_DIR/libs.ref/StdCellLib/liberty/StdCellLib_slow.lib
read_verilog $BUILD_DIR/export/corona_cts_export.v

link_design corona_cts_export
read_spef $BUILD_DIR/export/corona_cts_export.max.spef

create_clock -name sys_clk -period 10000 -waveform {0 5000} [get_pins {corona_cts_I_spare_buffer_0/i}]
set_propagated_clock [get_clocks {sys_clk}]

report_checks
report_tns
report_wns

exit
