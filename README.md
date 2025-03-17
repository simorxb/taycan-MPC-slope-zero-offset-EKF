# Model Predictive Control with Kalman Filter - Porsche Taycan

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/taycan-MPC-slope-zero-offset-EKF)

## Summary
This project demonstrates the integration of a Model Predictive Control (MPC) algorithm with an Extended Kalman Filter (EKF) to achieve zero-offset speed control for a Porsche Taycan under slope disturbances. The approach solves the inherent limitation of MPC in handling unmeasured disturbances by estimating them with an EKF.

## Project Overview
### Problem Statement
Model Predictive Control (MPC) inherently lacks visibility of external disturbances, such as slope variations, which can compromise its predictive accuracy. In this project, an Extended Kalman Filter (EKF) is introduced to estimate these disturbances, improving the performance of the control algorithm.

### Solution Architecture
The control architecture consists of:
- **Model Predictive Control (MPC)**: Used for speed control with minimum overshoot and zero steady-state error.
- **Extended Kalman Filter (EKF)**: Estimates force disturbances caused by slope and other unmeasured factors, providing real-time feedback to MPC.

### Plant Model - Porsche Taycan
The dynamic model of the Porsche Taycan includes:
- Mass $m$
- Force $F$
- Aerodynamic drag $-0.5 \cdot C_d \cdot A \cdot \rho \cdot v^2$
- Road slope $\theta$

The system dynamics are described by:

$m \cdot \frac{dv}{dt} = F - 0.5 \cdot C_d \cdot A \cdot \rho \cdot v^2 - m \cdot g \cdot \sin(\theta)$

### MPC Augmented Model
To handle disturbances, the model is augmented with:
- State $x_1 = v$ (velocity)
- State $x_2 = F_d$ (force disturbance)
- Input $u_1 = F$ (applied force)
- Input $u_2 = \dot{F}_d$ (derivative of force disturbance)

Dynamic equations:

$\dot{x}_1 = \frac{\text{lim}(u_1) - 0.5 \cdot C_d \cdot A \cdot \rho \cdot x_1^2 - x_2}{m}$

$\dot{x}_2 = u_2$

Here, $\text{lim}(u_1)$ represents the force-limiting function.

### EKF Discrete-Time Implementation
The EKF estimates $x_1$ and $x_2$ using the discretized model:

$x_{1,k+1} = x_{1,k} + T_s \cdot \frac{\text{lim}(u_{1,k}) - 0.5 \cdot C_d \cdot A \cdot \rho \cdot x_{1,k}^2 - x_{2,k}}{m}$

$x_{2,k+1} = x_{2,k} + T_s \cdot u_{2,k}$

This allows real-time disturbance estimation and compensation.

### Simulation Results
The integrated MPC + EKF control architecture ensures smooth speed control with minimal overshoot and zero steady-state error, even under varying slope conditions.

### Tuning
We explore the behaviour of the control algorithm when the weight related to the control error is kept constant (1) and the weight related to the rate of change of the manipulated variables varies.

We use 3 different configurations, from a very small penalty (we are not interested in how fast we vary the manipulated variables) to a high penalty (we don't want to vary the manipulated variables too fast):

- **Small penalty**: 0.0001

- **Medium penalty**: 0.002

- **High penalty**: 0.007

Use the script 'run_and_plot' to run the 3 scenarios.

### Robustness Test

The test_robustness MATLAB script performs robustness analysis for a vehicle speed control system with Extended Kalman Filter (EKF) state estimation. The script evaluates the controller's performance across different vehicle parameters to assess system robustness:

- Vehicle mass (±25% variation from nominal)
- Aerodynamic drag coefficient (±25% variation from nominal)

Generates comparative plots showing:
1. Speed tracking performance
2. Control effort (applied force)
3. Force disturbance estimation via EKF

#### How It Works
The script runs multiple simulations using a Simulink model (model.slx) with different combinations of mass and drag coefficients. For each simulation:

- Vehicle mass varies between 75% and 125% of nominal value
- Drag coefficient varies between 75% and 125% of nominal value
- The vehicle follows a predefined speed trajectory (10 → 50 → 60 → 10 m/s)

#### Output

The script generates a figure with three subplots showing:

1. Speed Tracking: Compares actual vehicle speed with the reference trajectory across all parameter combinations
2. Control Force: Shows the control input required for each parameter combination
3. Disturbance Estimation: Displays the EKF's estimation of force disturbances

## Code and Model
The MATLAB and Simulink models for this project are available in the GitHub repository: [taycan-MPC-slope-zero-offset-EKF](https://github.com/simorxb/taycan-MPC-slope-zero-offset-EKF).

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).

