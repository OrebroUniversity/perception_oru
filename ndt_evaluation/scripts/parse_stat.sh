#!/bin/bash
FILES=./*_gt.txt
est_end='_est.txt'
png_end='.png'
txt_end='.txt'
rot='rot'
pos='pos'
rpe='rpe'

echo "# Parsed ate data using parse_stat.sh..." > data.txt

for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    ate_txt=$base_name$txt_end

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
	# Parse mindist
	if [ $i -eq 3 ]
	then
	    mindist=${x:7}
	fi
	# Parse the sensorcutoff
	if [ $i -eq 4 ]
	then
	    sensorcutoff=${x:12}
	fi
	# Parse rlf
	if [ $i -eq 7 ]
	then
	    rlf=${x:3}
	fi
	i=$((i+1)) 
    done

    echo "Res $res SC $SC sensorcutoff $sensorcutoff mindist $mindist rlf $rlf"
    echo -n "$SC $res $sensorcutoff $mindist $rlf " >> data.txt

    # Need to open the files (ate file) and extract mean and stddev...
    # ----- the file content ----
    # absolute_translational_error.rmse 0.942410 m
    # absolute_translational_error.mean 0.823487 m
    # absolute_translational_error.median 0.673875 m
    # absolute_translational_error.std 0.458264 m
    # absolute_translational_error.min 0.129373 m
    # absolute_translational_error.max 1.859175 m
    while read p; do

	arr=$(echo $p | tr " " "\n")
	num_value=0
	i=0
	# Get the second 'string' -> contains
	for x in $arr
	do
	    if [ $i -eq 1 ]
		then 
		#echo "\"$x\""
		#echo $i
		num_value=$x
	    fi
	    i=$((i+1)) 
	done
	echo -n $num_value >> data.txt
	echo -n " " >> data.txt
	
	
    done < $ate_txt
    echo -n $Dd >> data.txt
    echo "" >> data.txt

done
