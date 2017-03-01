#!/bin/bash
FILES=./*.ate
ate_end='.ate'

echo "# Parsed ate data using parse_stat_d2d_eval.sh..." > data.txt

for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    ate_txt=$base_name

    # Parse the motion model
    # xxxxxDd0.4Dt1Cdxxxxx -> 0.4Dt1C -> 0.4
    # Split on d, take the 2:nd substring. Split on D take the first sting.
    Dd=$(echo $base_name | cut -d 'd' -f 2)
#    echo $Dd
    Dd=$(echo $Dd | cut -d 'D' -f 1)
#    echo $Dd

    Dt=$(echo $base_name | cut -d 'T' -f 1)
    #echo $Dt
    Dt=$(echo $Dt | cut -d 't' -f 2)
    #echo $Dt
    # Remove final dot
    Dt=${Dt::-1}
    

    # Parse the resolution (assumes a fixed offset)
    res=${base_name:12}
#    echo $res
    res=$(echo $res | cut -d 'D' -f 1)
#    echo $res
    
    # Get the type of objective used (d2d, d2d_sc, filter)...
    # Get the type name (after Ts.) but before ate (...Ts.XXXXX.ate)
    type=$(echo $base_name | cut -d 'T' -f 2)
    type=${type:2}
    type=$(echo $type | cut -d '.' -f 1)
    
    echo "Res $res Dd $Dd Dt $Dt objective $type"
    echo -n "$res $Dd $Dt $type " >> data.txt

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
