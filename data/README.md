# Data

This directory contains the raw data files from our experiments using the CyberDiver. The CSV files contain aggregated tables of the peak acceleration corresponding to figures 3(D), 4(C), and 5(C).

## Validation

This directory contains the validation tests performed to confirm that the CyberDiver can behave as an undamped linear elastic cyber-physical system with programmable stifffess, corresponding to figures 3(A) and 3(B). The CSV files contain the quasi-static compression test results using a force test machine with displacements reported in mm and force in Newtons. The ".dat" files correspond to the dyamic ring down tests presented in figure 3(B). These are the binary data files produced by the CyberDiver that can be parsed using the provided MATLAB scripts. 

Additional force test machine results are provided for the nonlinear programmed  structural response corresponding to figure 5(A). Each of the compression test data files is labeled with either the programmed stiffness for the linear cases or the programmed force threshold in the nonlinear cases. 

Finally, ring down test data files corresponding to the damped cyber-physical structure in figure 4(A) are provided.

## Linear undamped

This directory contains the water entry experimental data files corresponding to figures 3(C) and 3(D). Each file is labeled with the linear programmed stiffness, the drop speed, and the trial number. These are the binary data files produced by the CyberDiver that can be parsed using the provided MATLAB functions in the scripts directory of this repository.

## Linear damped

This directory contains the water entry experimental data files corresponding to figures 4(B) and 4(C). Each file is labeled with the linear programmed stiffness, the drop speed, the programmed damping ratio, and the trial number. These are the binary data files produced by the CyberDiver that can be parsed using the provided MATLAB functions in the scripts directory of this repository.

## Nonlinear

This directory contains the water entry experimental data files corresponding to figures 5(B) and 5(C). Each file is labeled with the programmed force threshold, the drop speed, and the trial number. The stiffness of the linear region is 15 N/mm for all of the files in this folder. These are the binary data files produced by the CyberDiver that can be parsed using the provided MATLAB functions in the scripts directory of this repository.

## Splash manipulation

This directory contains the water entry data files corresponding to the active maneuver experiments in figure 6. Each file is labeled with the maneuver type and the trial number. These are the binary data files produced by the CyberDiver that can be parsed using the provided MATLAB functions in the scripts directory of this repository.
