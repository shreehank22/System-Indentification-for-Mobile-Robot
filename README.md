# System Identification for Mobile Robot

This repository contains MATLAB and Simulink-based implementation of a **data-driven control strategy** for a mobile robot. A linear state-space model of a nonlinear robotic system is identified using **Subspace State-Space System Identification (N4SID)** from acceleration and control input data. The identified model is then used for designing and tuning a PID controller, ensuring stable and optimal closed-loop performance.

---

## 📌 Key Features

* ✅ **System Identification** using the N4SID algorithm
* ✅ **Data-driven modeling** from acceleration and control input signals
* ✅ **PID Controller Design** using the Ziegler-Nichols method
* ✅ **Validation through Simulation** using MATLAB/Simulink
* ✅ Demonstrates effective **linear control of nonlinear system dynamics**

---

## 🧠 Methodology

1. **Data Collection:** Acceleration and control input data from a mobile robot are used as the basis for identification.

2. **System Identification:**

   * Employed the **N4SID algorithm** to estimate a discrete-time linear state-space model.
   * Extracted system matrices (A, B, C, D) to represent the dynamics.

3. **Controller Design:**

   * Designed and tuned a **PID controller** using the **Ziegler-Nichols** tuning method.
   * Ensured closed-loop stability and performance based on time-domain metrics.

4. **Simulation and Validation:**

   * Simulated the control strategy in **Simulink** (`Final_prob.slx`).
   * Demonstrated the efficacy of model-based control through step response and error tracking.

---

## 📁 Repository Contents

```
System-Indentification-for-Mobile-Robot/
│
├── Algorithm_systemmodelestimation.m   # MATLAB script for model estimation using N4SID
├── Final_prob.slx                      # Simulink model for closed-loop simulation
├── README.md                           # Project documentation
```

---

## 🚀 How to Run

### Prerequisites

* MATLAB R2020a or later
* System Identification Toolbox
* Control System Toolbox
* Simulink

### Steps

1. **Clone the repository:**

   ```bash
   git clone https://github.com/shreehank22/System-Indentification-for-Mobile-Robot.git
   cd System-Indentification-for-Mobile-Robot
   ```

2. **Open MATLAB and run:**

   ```matlab
   Algorithm_systemmodelestimation
   ```

3. **Open and run Simulink model:**

   ```matlab
   open('Final_prob.slx')
   ```

---

## 📊 Results

* Accurate identification of the system dynamics using N4SID
* PID controller tuned for fast response and minimal overshoot
* Simulink validation confirms the effectiveness of the control strategy

---


## 👨‍💻 Author

**Shreehan Kate**
For academic or collaborative inquiries, feel free to connect.
