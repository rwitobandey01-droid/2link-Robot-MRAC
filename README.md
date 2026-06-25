2-Link Robot Manipulator Adaptive Control 
This repository features a 3D simulation platform for a 2-Link Planar Robot Manipulator tracking dynamic, unpredictable spatial targets in real time. It implements two major classes of advanced control theory: Direct Model Reference Adaptive Control (MRAC) and an Indirect Self-Tuning Regulator (STR) featuring online parameter identification via a kinematic Regressor Matrix.

The tracking target is bound dynamically to the user's desktop mouse cursor, transforming the workspace into an interactive, real-time haptic teleoperation environment.

2-Link-Robot-MRAC/
├── .gitignore
├── LICENSE
├── README.md
├── src/
│   ├── mrac_wrapper.m          # Direct MRAC adaptation laws & state integration
│   ├── str_wrapper.m           # Indirect STR control loop formulation
│   ├── parameter_estimator.m    # Online regressor-based mass parameter estimator
│   ├── robot_ik.m              # Analytical Inverse Kinematics solver
│   └── two_link_dynamics.m     # True non-linear physical robot plant dynamics
├── examples/
│   ├── test_MRAC_3D.m          # Interactive 3D mouse tracking via MRAC
│   ├── test_str.m              # Interactive 3D mouse tracking via STR (with real-time printouts)
│   └── test_STR_vs_MRAC_3D.m   # Abrupt transient & payload disturbance benchmark script
└── docs/
    └── system_architecture.png

1. Direct Model Reference Adaptive Control (MRAC)
In Direct MRAC, tracking errors (the variation between the actual robot states and an ideal reference model) drive the adaptation mechanism directly. The controller adjusts the feedback gains ($K_p, K_d$) online to minimize tracking error without attempting to identify or model explicit physical parameters like link masses.
+-----------------------------------+
               |      Reference Model (Ideal)      | <====== Target (r)
               +-----------------------------------+
                                 |
                                 v State (xm)
                                 |
                                 v  (-)
  Target (r) ===> [  Controller  ] ----> (Error) ===> [ Adaptation Law ]
                       ^                      |                 ||
                       |                      v                 || (Updates Gains Directly)
                       |               +------------+           ||
                       +-------------- |   Robot    | <==========+
                                       |   Plant    |
                                       +------------+
                                              |
                                              v State (x)

2. Indirect Self-Tuning Regulator (STR)
The Indirect Self-Tuning Regulator breaks the control architecture into two distinct elements: an Online Parameter Estimator and a Control Law Calculator. It observes the commanded joint torques ($\tau$) and resulting accelerations ($\ddot{q}$) through a kinematic Regressor Matrix ($Y$) to determine the true mass of the links ($\theta = [m_1; m_2]$) in real time. The calculated mass parameters are then used to update the inverse dynamics model.
+-----------------------------------+
               |      Reference Model (Ideal)      | <====== Target (r)
               +-----------------------------------+
                                 |
                                 v State (xm)
                                 |
                                 v  (-)
  Target (r) ===> [  Controller  ] ----> (Error)
                       ^       ^
                       |       | (Computes Inverse Dynamics)
                       |    +-----------------------------+
                       |    |   Control Law Calculation   |
                       |    +-----------------------------+
                       |                   ^
                       |                   | Estimated Parameters (θ_est)
                       |    +-----------------------------+
                       |    | Online Parameter Estimator  | <--- Accelerations (ddq)
                       |    +-----------------------------+
                       |                   ^
                       |                   | Prediction Error (τ - Yθ)
                       |                   |
                       |             [ Y Regressor ] <---------- States (q, dq)
                       |                   ^
                       |                   |
                Torque |                   | Torque (τ)
                (τ)    v                   |
               +-----------------------------------+
               |            Robot Plant            |
               +-----------------------------------+
                                 |
                                 v State (x)

 