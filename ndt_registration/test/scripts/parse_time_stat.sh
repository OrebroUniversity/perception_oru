#!/bin/bash
FILES=./*_gt.txt
txt_end='_timestat.txt'

echo "# Parsed time stat data using parse_time_stat.sh..." > data_time.txt

for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    time_txt=$base_name$txt_end

    # Parse the motion model
    # xxxxxDd0.4Dt1Cdxxxxx -> 0.4Dt1C -> 0.4
    # Split on d, take the 2:nd substring. Split on D take the first sting.
    Dd=$(echo $base_name | cut -d 'd' -f 2)
    #echo $Dd
    Dd=$(echo $Dd | cut -d 'D' -f 1)
    #echo $Dd

    # From the file name extract the resolution and the sensor cut off
    arr=$(echo $base_name | tr "_" "\n")
    num_value=0
    i=0
    # Get the second 'string' -> contains
    for x in $arr
    do

	# Parse res -> skip the 3 first chars
	if [ $i -eq 1 ]
	then 	    
	    res=${x:3}
	fi
	# Parse SC flag
	if [ $i -eq 2 ]
	then
	    SC=${x:2}
	fi
	# Parse the sensorcutoff
	if [ $i -eq 4 ]
	then
	    sensorcutoff=${x:12}
	fi
	i=$((i+1)) 
    done

    echo "Res $res SC $SC sensorcutoff $sensorcutoff"
    echo -n "$SC $res $sensorcutoff " >> data_time.txt

    while read p; do

	echo -n $p >> data_time.txt
	echo -n " " >> data_time.txt
	
	
    done < $time_txt
    echo -n $Dd >> data_time.txt
    echo "" >> data_time.txt

done
