#!/bin/bash
#SBATCH --job-name=pp_comparisons
#SBATCH --output=pp_runs.%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1                   # Run a single task	
#SBATCH --cpus-per-task=12           # Number of CPU cores per task
#SBATCH --mail-user=syonan@ucsc.edu
#SBATCH --mail-type=ALL
#SBATCH --mem=64G

# This sucks, but this is the only way we can get around the license timeout for MATLAB 2017B on the Hummingbird Compute Cluster...

MATLAB=matlab
#/hb/software/apps/matlab/2017b/bin/matlab

cd field_exploration/MATLAB

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.1]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.1]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.1]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.1]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.2]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.2]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.2]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.2]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.3]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.3]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.3]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.3]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.4]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.4]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.4]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.4]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.5]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.5]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.5]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([2],[100],[[.5]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.1]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.1]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.1]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.1]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.2]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.2]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.2]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.2]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.3]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.3]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.3]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.3]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.4]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.4]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.4]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.4]],1\)
module unload matlab

# module load matlab
# $MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.5]],100\)
# module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.5]],4\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.5]],2\)
module unload matlab
module load matlab
$MATLAB -nodisplay -nodesktop -r pp_run\([3],[100],[[.5]],1\)
module unload matlab