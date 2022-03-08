set CALIB $::env(HOME)/openrcx-calibration
set PDK sky130B
set BUILD_DIR build/sky130

read_lef $BUILD_DIR/export/StdCellLib.lef
read_def $BUILD_DIR/export/corona_cts_export.def

define_process_corner -ext_model_index 0 X

foreach corner {min nom max} {
	extract_parasitics -ext_model_file "$CALIB/openrcx-magic/$PDK/rules.openrcx.$PDK.$corner.magic"
	write_spef $BUILD_DIR/export/corona_cts_export.$corner.spef
}

write_verilog $BUILD_DIR/export/corona_cts_export.v
exit
