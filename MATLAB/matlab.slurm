#!/bin/bash
#SBATCH --job-name=pp_comparisons
#SBATCH --output=pp_run_output.%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1                   # Run a single task	
#SBATCH --cpus-per-task=8            # Number of CPU cores per task
#SBATCH --mail-user=syonan@ucsc.edu
#SBATCH --mail-type=ALL
#SBATCH --mem=128G

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.1]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.1]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.1]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.1]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.2]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.2]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.2]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.2]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.3]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.3]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.3]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.3]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.4]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.4]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.4]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.4]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.5]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.5]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.5]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([2],[200],[[.5]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.1]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.1]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.1]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.1]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.2]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.2]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.2]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.2]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.3]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.3]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.3]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.3]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.4]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.4]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.4]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.4]],1\)

module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.5]],200\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.5]],4\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.5]],2\)
module load matlab
/hb/software/apps/matlab/2017b/bin/matlab -nodisplay -r -nodisplay -nodesktop -r pp_run\([3],[200],[[.5]],1\)