#!/bin/bash
read -p "flight number? " num
echo "moving bearing files"
mv bearing_calc_eor.csv logsW/bearing_calc_eor_$num.csv
#mv bearing_calc_mle.csv logsW/bearing_calc_eor_$num.csv
mv wifly.csv logsW/wifly_$num.csv
#mv wifly2.csv logsT/wifly2_$num.csv
mv outputlog.log logsW/outputlog_$num.csv
#echo "bearing_calc_$num_take2"
