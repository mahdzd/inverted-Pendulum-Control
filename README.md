# Inverted Pendulum Control (Furuta Pendulum)

This repository contains the full modeling, analysis, simulation, and control implementation of a **Furuta Pendulum (Rotary Inverted Pendulum)** developed as part of the **ELE 443 – Control Systems Lab** at the Lebanese American University (LAU).

The project covers first-principle modeling using Lagrangian mechanics, linearization around multiple equilibria, state-space formulation, simulation in MATLAB/Simulink, and controller design including **PD control** and **Full State Feedback**.

---

## 📌 Project Overview

The Furuta pendulum is a nonlinear, underactuated, and inherently unstable system around its upright equilibrium.  
This project aims to:

- Derive **nonlinear equations of motion** using Lagrangian formulation  
- Perform **linearization** around:
  - Upright equilibrium (α = 0)
  - Downward equilibrium (α = π)
- Construct **state-space models**
- Convert **motor torque input to voltage input**
- Simulate open-loop and closed-loop behavior
- Design and evaluate:
  - PD controllers
  - Full state feedback controller
- Validate results through MATLAB and Simulink simulations

---

## 🧠 Modeling Summary

- **Generalized coordinates**:
  - θ : rotary arm angle
  - α : pendulum angle
- **Modeling method**: Lagrangian mechanics
- **Effects included**:
  - Coupled dynamics
  - Gravity
  - Viscous damping
  - DC motor dynamics
- **Assumptions**:
  - Rigid bodies
  - Ideal revolute joints
  - Small-angle approximations for linearization

---

## ⚙️ Control Strategies Implemented

### 1. Open-Loop Simulation
- Validation of linearized models
- Comparison between upright and downward equilibria

### 2. PD Control
- Separate PD controllers for:
  - Pendulum angle α
  - Rotary arm angle θ
- Integral action intentionally removed due to instability
- Successfully stabilized the pendulum near α = 0

### 3. Full State Feedback
- Pole placement approach
- Compared with PD behavior
- Demonstrated sensitivity to noise and modeling inaccuracies
- Highlighted practical limitations without observers

---

## 🧪 Simulation & Tools

- **MATLAB** for analytical modeling and state-space derivation
- **Simulink** for:
  - Open-loop simulations
  - PD controller implementation
  - Full state feedback testing
- **Voltage-driven model** using DC motor equations

---

## 📄 Report

The full theoretical derivations, linearization steps, state-space matrices, and simulation results are documented in:

📘 **ControlLab Final Project Report.pdf**

This includes:
- Complete Lagrangian derivations
- Explicit state-space matrices
- Simulation plots
- Controller tuning discussion
- Experimental deployment analysis

---

## 👥 Authors

- **Ali Awada**  
- **Mahdi T. Zein Al Dine**

Lebanese American University  
ELE 443 – Control Systems Lab  
Fall 2025

---

## 📜 License

This project is intended for **educational and academic use**.  
Reuse with proper attribution.

---

## ⭐ Notes

- The upright equilibrium (α = 0) is **open-loop unstable**
- PD control proved more reliable than full state feedback in practice
- Results emphasize the gap between theory and hardware implementation

---

If you are studying **nonlinear control, inverted pendulums, or state-space modeling**, this repository provides a complete end-to-end reference.


