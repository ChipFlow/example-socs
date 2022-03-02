exec mkdir -p build/sky130/tas
cd build/sky130/tas

avt_config simToolModel hspice
avt_config avtVddName "vdd"
avt_config avtVssName "vss"
# avt_config tasBefig yes
# avt_config tmaDriveCapaout yes
avt_config avtParasiticCacheSize 0
avt_config avtPowerCalculation yes
avt_config simSlope 20e-12

# avt_config tasHierarchicalMode yes

avt_config simPowerSupply 1.62
avt_config simTemperature 85

avt_LoadFile $::env(HOME)/mpw4/thirdparty/open_pdk/C4M.Sky130/libs.tech/ngspice/C4M.Sky130_ss_model.spice spice
avt_LoadFile $::env(HOME)/mpw4/thirdparty/open_pdk/C4M.Sky130/libs.ref/StdCellLib/spice/StdCellLib.spi spice

foreach cell {sff1_x4 sff1r_x4} {
    inf_SetFigureName $cell
    inf_MarkSignal sff_m "MASTER"
    inf_MarkSignal sff_s "FLIPFLOP+SLAVE"
}

if [info exists ::env(CORNER)] {
	set corner $::env(CORNER)
} else {
	set corner max
}

avt_LoadFile ../export/corona_cts_export.v verilog
avt_LoadFile ../export/corona_cts_export.$corner.spef spef
set fig [hitas corona_cts_export]

inf_SetFigureName corona_cts_export
create_clock -period 10000 -waveform {5000 0} {io_in_from_pad_net(0)}

set fig [ttv_LoadSpecifiedTimingFigure corona_cts_export]
set stbfig [stb $fig]
stb_DisplaySlackReport [fopen slack_pnr_max.rep.$corner w] $stbfig * * ?? 10  all 10000

