

# Windows Folder – REMARO Evaluation and SDP-GD Processing

This folder contains MATLAB scripts and tools used for:

* Evaluating the **SDP-GD algorithm**
* Performing **system identification** from experimental data
* Running a **Sonar Based SLAM** using sonar data
* Comparing results with ORB-SLAM2 and Sonar-based SLAM algorithms

These evaluations correspond to the output map generation and robustness testing described in the thesis.

---

## Folder Structure Overview

| File/Script            | Description                                                                 |
| ---------------------- | --------------------------------------------------------------------------- |
| `main4.m`              | Main evaluation script used for final results presented in the thesis       |
| `main5.m`, `main6.m`   | Variants used for testing different combinations of training/testing maps   |
| `identification2.m`    | Identifies the system using a black-box model from experimental `.mat` data |
| `Kalman_class.m`       | Contains the Kalman filter SLAM implementation using sonar measurements     |
| `/identified_systems/` | Folder to store system models identified at different sampling frequencies  |

---

## 1. System Identification

The file `identification2.m` performs system identification using either:

* The **linear reference input**, or
* The **measured thruster input**

```matlab
identificationDatapath = dataFolder_Chirp + ChirpFiles(3);
obj.downSampleFreq = 6;

trusterInput = false; % Set true to use thruster input instead

obj.computeNED_AddNoise(false);
obj.DownsampleData();


```

The resulting system is saved and later used by the evaluation scripts.

---

## 2. Main Evaluation (main4.m)

`main4.m` evaluates the SLAM performance using the previously identified system. It uses a Kalman-based estimation pipeline to reconstruct the seabed map and compare against ground truth.

### Key Parameters:

```matlab
Ts = 6;           % Sampling interval
cutIdx = 60;      % Index where evaluation starts (to remove startup effects)

plot1 = true;     % Enable position/error plots
plot2 = true;     % Enable seabed comparison plots

MapIsTest = false;  % Set true if using a test map 

identifiedSystemName = 'identifiedSystem_Ts6.mat';
```

### Training/Test Files:

```matlab
trainFiles = Straight_differentBottomsTest;
testFiles  = Straight_differentBottomsTest;
TRAINING=TESTING!!!

folderDataPath = dataFolder_Straight_differentBottoms;
```

Set `trainModel = false` to use an already identified system.

---

## 3. Kalman Filter (Sonar SLAM)

The `Kalman_class.m` file implements the sonar-based SLAM algorithm that fuses IMU, sonar, and DVL data to estimate AUV position and seabed height. It is invoked inside the main scripts during evaluation.

---

## Notes

* All `.mat` data files used as input (e.g., `S0.22_D10_R0.5_D12.mat`) must be placed in the expected `dataFolder_*` directories.
* Identified systems are stored in `/identified_systems/` for reuse across experiments.
* The code is structured to support various levels of map complexity and noise conditions, as described in the thesis.

---

## Running a Full Experiment

1. Identify the system (if needed) using `identification2.m`.
2. Set your data paths and model file in `main4.m`.
3. Run `main4.m` to generate RMSE, R², and depth estimation results.
4. Compare with ORB-SLAM2 and Sonar SLAM outputs.
