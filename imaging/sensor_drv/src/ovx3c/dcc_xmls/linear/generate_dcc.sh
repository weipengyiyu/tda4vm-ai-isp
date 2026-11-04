DCC_TOOL_PATH=../../../../../tools/dcc_tools/
OUT_PATH=../../../../include

rm -f *.bin
bin_folder=../../dcc_bins/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/ovx3c-dcc/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/ovx3c-dcc/linear/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

rm -f $OUT_PATH/dcc_viss_ovx3c.h
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_rgb2rgb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_h3a_aewb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_viss_nsf4.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_viss_blc.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_cfa_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_linear_decompand_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_h3a_mux_luts_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_viss.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss.bin $OUT_PATH/dcc_viss_ovx3c.h dcc_viss_ovx3c
echo; echo
rm -f *.bin
rm -f $OUT_PATH/dcc_2a_ovx3c.h
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_awb_alg_ti3_tuning.xml
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_h3a_aewb_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_2a.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a.bin $OUT_PATH/dcc_2a_ovx3c.h dcc_2a_ovx3c
echo; echo
rm -f *.bin
rm -f $OUT_PATH/dcc_ldc_ovx3c.h
$DCC_TOOL_PATH/dcc_gen_linux ovx3c_mesh_ldc_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_ldc.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc.bin $OUT_PATH/dcc_ldc_ovx3c.h dcc_ldc_ovx3c
echo; echo
rm -f *.bin
