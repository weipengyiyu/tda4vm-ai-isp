# mkdir tools/include 

cd scripts

python3 ctt_def_xml_gen.py ../configs/imx623_properties.txt
cd ../imx623_output_hdr/dcc_xmls/wdr


#python3 ctt_def_xml_gen.py ../configs/sc2210_properties.txt
#cd ../sc2210_output/dcc_xmls/linear

sleep 1 

sh generate_dcc.sh 


cd ../../../../
ls -al include/*


cp  include/*.h ../sensor_drv/include/


ls -al ../sensor_drv/include/*imx623*
