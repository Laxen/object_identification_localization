#!/bin/bash          
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

for file in "$DIR"/../data/results/segmentation_results/*/; do
	if [ -d "$file" ]; then
		echo "Removing directory $file"
		rm -r "$file"
	fi
done

for file in "$DIR"/../data/results/segmentation_results/*/; do
	if [ -d "$file" ]; then
		echo "Removing directory $file"
		rm -r "$file"
	fi
done

for file in "$DIR"/../data/results/identification_results/*/; do
	if [ -d "$file" ]; then
		echo "Removing directory $file"
		rm -r "$file"
	fi
done

for file in "$DIR"/../data/results/pose_estimation_results/*/; do
	if [ -d "$file" ]; then
		echo "Removing directory $file"
		rm -r "$file"
	fi
done

for file in "$DIR"/../data/results/hint_system_results/*/; do
	if [ -d "$file" ]; then
		echo "Removing directory $file"
		rm -r "$file"
	fi
done

l_p="$DIR"/../data/results/segmentation_results/latest_path
if [ -f "$l_p" ]; then
	echo "Clearing text in $l_p"
	> "$l_p"
fi
