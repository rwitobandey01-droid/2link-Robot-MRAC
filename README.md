# 2-Link Robot Manipulator Adaptive Control

A MATLAB-based 3D simulation platform for investigating advanced adaptive control techniques on a planar two-link robotic manipulator.

The system tracks dynamic and unpredictable targets in real time using two distinct adaptive control strategies:

* **Direct Model Reference Adaptive Control (MRAC)**
* **Indirect Self-Tuning Regulator (STR)** with online parameter identification

The target position is continuously linked to the user's desktop mouse cursor, creating an interactive real-time tracking environment for evaluating controller performance under changing operating conditions and disturbances.

---

## Features

* Real-time 3D robot visualization
* Interactive mouse-driven target tracking
* Direct MRAC implementation
* Indirect STR implementation
* Online mass parameter estimation
* Analytical inverse kinematics solver
* Nonlinear robot dynamics simulation
* Comparative MRAC vs STR benchmark experiments
* Disturbance and payload-change testing

---

## System Architecture

```text
Mouse Cursor Target
         │
         ▼
  Reference Model
         │
         ▼
 Adaptive Controller
 (MRAC or STR)
         │
         ▼
  Robot Dynamics
         │
         ▼
   Joint States
         │
         └───────────── Feedback
```

---

## Repository Structure

```text
2-Link-Robot-MRAC/
│
├── src/
│   ├── mrac_wrapper.m
│   ├── str_wrapper.m
│   ├── parameter_estimator.m
│   ├── robot_ik.m
│   └── two_link_dynamics.m
│
├── examples/
│   ├── test_MRAC_3D.m
│   ├── test_STR.m
│   └── test_STR_vs_MRAC_3D.m
│
├── docs/
└── system_architecture.png
```

---

# Control Approaches

## Direct Model Reference Adaptive Control (MRAC)

The MRAC controller adapts the control law directly from tracking error.

Instead of estimating physical parameters such as link masses, the controller continuously adjusts its gains so that the robot behaves like a predefined reference model.

### Principle

```text
Reference Model
       │
       ▼
 Tracking Error
       │
       ▼
 Adaptive Law
       │
       ▼
 Controller Gains
       │
       ▼
    Robot
```

### Key Idea

MRAC learns **how to control the robot**, rather than learning the robot itself.

### Advantages

* Simple adaptive structure
* Fast adaptation
* No explicit parameter estimation
* Suitable when plant parameters are unknown

---

## Indirect Self-Tuning Regulator (STR)

The STR controller separates adaptation into two stages:

1. Online parameter estimation
2. Control law computation

The estimator identifies unknown robot parameters using a regressor formulation and continuously updates the internal dynamic model.

### Principle

```text
Robot States
      │
      ▼
 Regressor Matrix
      │
      ▼
 Parameter Estimator
      │
      ▼
 Estimated Parameters
      │
      ▼
 Inverse Dynamics Controller
      │
      ▼
       Robot
```

### Key Idea

STR learns **the robot model first**, then computes the control law using the identified parameters.

### Advantages

* Physical parameter estimation
* Improved interpretability
* Better handling of significant model changes
* Direct insight into plant dynamics

---

# MRAC vs STR

| Feature                  | MRAC    | STR        |
| ------------------------ | ------- | ---------- |
| Learns Controller        | ✓       | Indirectly |
| Learns Plant Parameters  | ✗       | ✓          |
| Online Mass Estimation   | ✗       | ✓          |
| Computational Complexity | Lower   | Higher     |
| Model Knowledge Required | Minimal | Moderate   |
| Interpretability         | Lower   | Higher     |

<img width="1909" height="957" alt="image" src="https://github.com/user-attachments/assets/dff2781d-dda9-4192-90d0-623c03b4348c" />


---

# Example Experiments

## MRAC Tracking

```matlab
test_MRAC_3D
```

Evaluates direct adaptive tracking of a moving cursor target.

---

## STR Tracking

```matlab
test_STR
```

Evaluates online mass estimation and adaptive inverse-dynamics control.

---

## MRAC vs STR Benchmark

```matlab
test_STR_vs_MRAC_3D
```

Compares both controllers under:

* Sudden payload changes
* Abrupt reference motion
* Dynamic disturbances
* Tracking transients

---

# Educational Objectives

This project demonstrates:

* Adaptive control theory
* Model reference control
* Self-tuning regulators
* Online parameter identification
* Robot inverse dynamics
* Real-time nonlinear simulation
* Comparative adaptive control analysis

---

# License

This project is released under the MIT License.


 
