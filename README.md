# System Identification and Control of a Mobile Robot

This repository presents a MATLAB and Simulink-based workflow for designing a data-driven control system for a mobile robot. The approach involves identifying a linear approximation of the robot's nonlinear dynamics using the N4SID (Numerical Subspace State Space System IDentification) algorithm. The identified model is subsequently used for PID controller design and closed-loop simulation.

## Overview

The objective is to design a linear controller that performs well on a nonlinear robotic platform by leveraging real input-output data and system identification techniques. The PID controller is tuned based on the identified dynamics and validated through simulation.

## Features

* Discrete-time state-space model estimation using the N4SID algorithm
* Data-driven modeling from acceleration and control input signals
* PID controller tuning using the Ziegler-Nichols method
* Closed-loop validation in Simulink
* Demonstrates practical control of a nonlinear system using linear techniques

## Methodology

### 1. Data Collection

Acceleration data and control inputs from a mobile robot are recorded and used as the basis for system modeling.

### 2. System Identification

* The N4SID algorithm is applied to derive a discrete-time state-space model.
* System matrices $A, B, C, D$ are extracted to characterize the linearized system dynamics.

### 3. Controller Design

* A PID controller is designed and tuned using the Ziegler-Nichols method.
* Controller parameters are selected to ensure fast response and minimal steady-state error.

### 4. Simulation and Validation

* The control architecture is implemented in Simulink (`Final_prob.slx`).
* Closed-loop performance is evaluated via step response, transient behavior, and tracking error metrics.

## Repository Structure

```
System-Identification-for-Mobile-Robot/
│
├── Algorithm_systemmodelestimation.m   % N4SID model estimation script
├── DatacurationforN4SID.slx           % Data preprocessing/curation model
├── Overall_ControlArchitecture.slx    % High-level control architecture
├── README.md                          % Project documentation
```

## Getting Started

### Prerequisites

* MATLAB R2020a or newer
* System Identification Toolbox
* Control System Toolbox
* Simulink

### Steps to Run

1. Clone the repository:

   ```bash
   git clone https://github.com/shreehank22/System-Identification-for-Mobile-Robot.git
   cd System-Identification-for-Mobile-Robot
   ```

2. Open MATLAB and run the model estimation script:

   ```matlab
   Algorithm_systemmodelestimation
   ```

3. Launch and simulate the control architecture:

   ```matlab
   open('Final_prob.slx')
   ```

## Results

* Accurate estimation of a discrete-time linear model from experimental data
* PID controller achieves desirable performance (fast settling, low overshoot)
* Simulink simulations confirm closed-loop stability and control effectiveness

## Author

**Shreehan Kate**
For academic collaboration, feedback, or inquiries, feel free to reach out via GitHub or email.
