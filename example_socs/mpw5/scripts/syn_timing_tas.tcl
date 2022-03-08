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


if [info exists ::env(CORNER)] {
	set corner $::env(CORNER)
} else {
	set corner max
}

set tmodel ss
if {$corner == "nom"} {
	set tmodel tt
}
if {$corner == "min"} {
	set tmodel ff
}

avt_LoadFile $::env(HOME)/mpw4/thirdparty/open_pdk/C4M.Sky130/libs.tech/ngspice/C4M.Sky130_${tmodel}_model.spice spice
avt_LoadFile $::env(HOME)/mpw4/thirdparty/open_pdk/C4M.Sky130/libs.ref/StdCellLib/spice/StdCellLib.spi spice

foreach cell {sff1_x4 sff1r_x4} {
    inf_SetFigureName $cell
    inf_MarkSignal sff_m "MASTER"
    inf_MarkSignal sff_s "FLIPFLOP+SLAVE"
}

avt_LoadFile ../user_project_core_mpw5_syn.v verilog
set fig [hitas user_project_core_mpw5]

inf_SetFigureName user_project_core_mpw5
create_clock -period 10000 -waveform {5000 0} {io_in[0]}

set fig [ttv_LoadSpecifiedTimingFigure user_project_core_mpw5]
set stbfig [stb $fig]
stb_DisplaySlackReport [fopen slack_syn.rep.$corner w] $stbfig * * ?? 10  all 10000

