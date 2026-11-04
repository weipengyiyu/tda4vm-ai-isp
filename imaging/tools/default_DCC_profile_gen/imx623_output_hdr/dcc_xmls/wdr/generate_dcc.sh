DCC_TOOL_PATH=../../../../../tools/dcc_tools/
OUT_PATH=../../../../include

rm -f *.bin
bin_folder=../../dcc_bins/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/imx623-dcc-hdr/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/imx623-dcc-hdr/wdr/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

rm -f $OUT_PATH/dcc_viss_imx623_wdr.h
$DCC_TOOL_PATH/dcc_gen_linux imx623_rgb2rgb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_h3a_aewb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_viss_nsf4.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_viss_blc.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_cfa_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_wdr_decompand_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_wdr_glbce_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_h3a_mux_luts_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_viss_wdr.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss_wdr.bin $OUT_PATH/dcc_viss_imx623_wdr.h dcc_viss_imx623_wdr
echo; echo
rm -f *.bin
rm -f $OUT_PATH/dcc_2a_imx623_wdr.h
$DCC_TOOL_PATH/dcc_gen_linux imx623_awb_alg_ti3_tuning.xml
$DCC_TOOL_PATH/dcc_gen_linux imx623_h3a_aewb_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_2a_wdr.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a_wdr.bin $OUT_PATH/dcc_2a_imx623_wdr.h dcc_2a_imx623_wdr
echo; echo
rm -f *.bin
rm -f $OUT_PATH/dcc_ldc_imx623_wdr.h
$DCC_TOOL_PATH/dcc_gen_linux imx623_mesh_ldc_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_ldc_wdr.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc_wdr.bin $OUT_PATH/dcc_ldc_imx623_wdr.h dcc_ldc_imx623_wdr
echo; echo
rm -f *.bin
