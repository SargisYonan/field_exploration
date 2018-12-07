#!/bin/bash

#SBATCH --job-name=pp_comparisons
#SBATCH --output=pp_runs.%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1                   # Run a single task	
#SBATCH --cpus-per-task=24           # Number of CPU cores per task
#SBATCH --mail-user=syonan@ucsc.edu
#SBATCH --mail-type=ALL
#SBATCH --mem=128G

#usage: sbatch this_file

MATLAB=matlab

cd field_exploration/MATLAB

declare -a sigma_field_dividers=(100 50 25 1)
declare -a percentage_scans=(10 20 30)
field_size_width=100
random_seed=2

module purge
module load hb hb-gnu

for div in "${sigma_field_dividers[@]}"
do
	for perc in "${percentage_scans[@]}"
	do
		module load matlab/2017b
		$MATLAB -nodisplay -nodesktop -r pp_run_method\(\'zz\',$random_seed,$field_size_width,"$perc","$div"\)
		module unload matlab/2017b
	done

	module load matlab/2017b
	$MATLAB -nodisplay -nodesktop -r pp_run_method\(\'nhv\',$random_seed,$field_size_width,30,"$div"\)
	module unload matlab/2017b

	module load matlab/2017b
	$MATLAB -nodisplay -nodesktop -r pp_run_method\(\'nnhv\',$random_seed,$field_size_width,30,"$div"\)
	module unload matlab/2017b

	module load matlab/2017b
	$MATLAB -nodisplay -nodesktop -r pp_run_method\(\'mc\',$random_seed,$field_size_width,30,"$div"\)
	module unload matlab/2017b

	module load matlab/2017b
	$MATLAB -nodisplay -nodesktop -r pp_run_method\(\'greedy\',$random_seed,$field_size_width,30,"$div"\)
	module unload matlab/2017b
done
