set CALIB $::env(HOME)/openrcx-calibration
set PDK sky130B
set BUILD_DIR build/sky130

if [info exists ::env(CORNER)] {
	set corner $::env(CORNER)
} else {
	set corner max
}

read_lef -tech -library $BUILD_DIR/export/StdCellLib.patched.$corner.lef
read_def $BUILD_DIR/export/corona_cts_export.def

define_process_corner -ext_model_index 0 X
extract_parasitics -ext_model_file "$CALIB/openrcx-magic/$PDK/rules.openrcx.$PDK.$corner.magic" -lef_res -cc_model 12 -context_depth 10
write_spef $BUILD_DIR/export/corona_cts_export.$corner.spef

write_verilog $BUILD_DIR/export/corona_cts_export.v
exit
