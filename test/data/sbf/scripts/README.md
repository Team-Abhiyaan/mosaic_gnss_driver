sbf_table.html was extracted from the original document

copy paste the sbf_table.html table to a text file. Manually remove header lines.
cat sbf_name_id_table | sed -r 's/([0-9])([a-zA-Z])/\1 \2/g; s/([a-zA-Z])([0-9])/\1 \2/g'  | awk '{$2=$2};1'| tee sbf_name_id_clean
run save_block_names.py to generate the sbf_name_id_desc.pkl


add_compile_options(-DMOSAIC_SBF_PRINT_ID)

rosrun mosaic_gnss_driver mosaic_gnss_driver_node _conn:="pcap" _parser:="sbf" _device:="./mosaic_gnss_driver/test/data/sbf/capture_003.pcap" 
    | sort -r | uniq -c | sort -r | id_to_name.py