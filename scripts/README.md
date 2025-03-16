# Scripts

This directory contains the MATLAB scripts used to process the experimental data and run the model for water impact.

## Data processing

Scripts for interpreting the raw binary ".dat" data files produced by the CyberDiver. Each data file contains a header with all of the configuration parameters for that experiment that will be read into the MATLAB workspace as a struct, followed by a table of the experimental data points with time stamps. To configure an experiment, the MATLAB confguration struct can be edited and then the write configuration script can be used to put in on the SD card in the correct binary format.

## Added mass model

Includes the scripts to run the two-way coupled added mass model for impact as well as the digitized added mass curve due to Shiffman and Spencer. 

## Tuning

Scripts for tuning the control loops of the CyberDiver by iteratively updating the control loop gains via the serial interface and measuring and plotting the step response of the controlled system.

