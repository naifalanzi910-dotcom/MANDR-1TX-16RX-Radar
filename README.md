# MANDR 1TX/16RX FMCW Radar Project

This repository contains a MATLAB-based FMCW radar simulation and processing project using a **1TX/16RX** antenna configuration. The project simulates complex FMCW beat signals across a 16-channel receive array and processes the received radar data using range-Doppler processing, CFAR detection, MUSIC angle estimation, and EKF-based target tracking.

## Project Overview

The system is designed around a **single transmit antenna (1TX)** and a **sixteen-channel receive array (16RX)** arranged as a **4 × 4 RX antenna array**. The radar operates at **77 GHz** and is intended for simulation, signal processing, and target tracking studies.

The project includes two main MATLAB files:

* `fmcw\_16rx\_generator\_1tx.m`  
Generates simulated FMCW radar signals for the 1TX/16RX radar configuration and streams the received channel data.
* `fmcw\_16rx\_processor\_1tx.m`  
Receives the radar data, performs signal processing, detects targets, estimates angles, and tracks detected targets.

It also includes:

* `GraduationProject.grc`  
GNU Radio flowgraph file used as part of the project workflow.

## Main Features

* 1 transmit antenna and 16 receive antennas
* 4 × 4 receive antenna array
* FMCW radar operation at 77 GHz
* Complex baseband beat-signal simulation
* Range FFT processing
* Doppler FFT processing
* Range-Doppler map generation
* CA-CFAR target detection
* MUSIC-based azimuth and elevation estimation
* 3D target localization
* Extended Kalman Filter (EKF) target tracking
* ZeroMQ-based streaming between generator and processor
* GNU Radio flowgraph support

## Radar Configuration

|Parameter|Value|
|-|-|
|Carrier frequency|77 GHz|
|Transmit antennas|1|
|Receive antennas|16|
|RX array layout|4 × 4|
|ADC samples per chirp|1024|
|Chirps per frame|512|
|Sampling rate|32 MHz|
|FMCW slope|15 THz/s|

## Processing Chain

The processor follows this general signal-processing chain:

1. Receive synchronized RX channel data
2. Remove ADC DC offset
3. Apply range FFT
4. Remove static clutter
5. Apply Doppler FFT
6. Generate range-Doppler power map
7. Apply CA-CFAR detection
8. Estimate azimuth and elevation using MUSIC
9. Validate and filter detections
10. Track targets using an Extended Kalman Filter
11. Display range-Doppler, angle, 3D position, and tracking results

## How to Run

1. Open MATLAB.
2. Make sure Python and ZeroMQ dependencies are available for MATLAB.
3. Run the processor or GNU Radio relay as required by the project setup.
4. Run:

```matlab
fmcw\_16rx\_generator\_1tx
```

5. In another MATLAB session, run:

```matlab
fmcw\_16rx\_processor\_1tx
```

6. The processor will receive the radar streams, process the data, and display detection and tracking results.

## Notes

This version is a **1TX/16RX FMCW radar configuration**. It is **not a TDM-MIMO version**, because only one transmitter is used. Therefore, the array has **16 physical receive elements**, not a larger virtual MIMO array.

For TDM-MIMO versions, multiple transmit antennas would be required to create additional virtual array elements.

## Applications

This project can support studies related to:

* FMCW radar signal processing
* Range-Doppler analysis
* Radar target detection
* Direction-of-arrival estimation
* 3D radar tracking
* MATLAB and GNU Radio radar simulation workflows

## Author

Eng.Naif Abdulkarim Ayyash Alanazi (Electrical Engineer)

