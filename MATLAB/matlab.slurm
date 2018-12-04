#!/bin/bash
#SBATCH --job-name=pp_comparisons
#SBATCH --output=pp_runs.%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1                   # Run a single task	
#SBATCH --cpus-per-task=12           # Number of CPU cores per task
#SBATCH --mail-user=syonan@ucsc.edu
#SBATCH --mail-type=ALL
#SBATCH --mem=64G

MATLAB=matlab
#/hb/software/apps/matlab/2017b/bin/matlab

#usage: sbatch this_file

cd field_exploration/MATLAB

declare -a sigma_field_dividers=("100" "4" "2" "1")
declare -a percentage_scans=("10" "20" "30")
field_size_width=100
random_seed=31

for perc in "${percentage_scans[@]}"
do
	for div in "${sigma_field_dividers[@]}"
	do
		module load matlab
		$MATLAB -nodisplay -nodesktop -r pp_run\([$random_seed],[$field_size_width],[[$perc]], $div\)
		module unload matlab
	done
done
