# ✈️ Longitudinal Dynamics Simulation

&#x20;

## 📖 Table of Contents

1. [Overview](#overview)
2. [Background: 6-DOF vs. Longitudinal-Only](#background-6-dof-vs-longitudinal-only)
3. [Contents](#contents)
4. [Requirements](#requirements)
5. [Source of Parameters](#source-of-parameters)
6. [Key Concepts & Equations](#key-concepts--equations)
   - [State Variables](#state-variables)
   - [Main Equations](#main-equations)
   - [Aerodynamic Forces & Moments](#aerodynamic-forces--moments)
7. [Code-to-Equation Mapping](#code-to-equation-mapping)
8. [Parameters & Values](#parameters--values)
9. [Simulation Results](#simulation-results)
10. [Usage](#usage)
11. [Applications & Extensions](#applications--extensions)
12. [License & Author](#license--author)

---

## 🔍 Overview

This MATLAB repository simulates and analyzes **longitudinal** (pitch-plane) dynamics of a Small UAV. It computes how elevator deflection and thrust influence:

- Pitch angle & pitch rate
- Forward & vertical velocities
- Altitude (vertical position)

All aerodynamic parameters are sourced from *Kristiansen, M., Small Unmanned Aircraft: Theory and Practice* (Section E.2, Aerosonde UAV).

Reference Courses:
   - **AE 372 - Flight Mechanics** (Prof. Ilkay Yavrucuk): [Go to Course](https://www.youtube.com/playlist?list=PLuiPz6iU5SQ-vPNTm_j0Jr4f9AWzCyoXk)

   - **MAE 175 - Flight Dynamics** (Prof Haithem Ezzat): [Go to Course](https://www.youtube.com/playlist?list=PLCheZLRn7G_yXDL3b4zCF5GLbH59q7-CN) 

   - **AE 514 - Undergraduate Flight Dynamics and Control**: [Go to Course](https://www.youtube.com/@AE-zc5sc/videos)

## 📚 Background: 6-DOF vs. Longitudinal-Only

A **full 6-DOF** (six degrees of freedom) aircraft model includes:

- 3 translational motions (surge, sway, heave)
- 3 rotational motions (roll, pitch, yaw)

> **Surge**: translation along the aircraft’s longitudinal (X) axis (forward/backward) **Sway**: translation along the lateral (Y) axis (side-to-side) **Heave**: translation along the vertical (Z) axis (up/down)

In this project, we focus solely on **longitudinal (pitch-plane)** motion:

- **Retained**: Surge (u), Heave (w), Pitch (θ, q), X-Z kinematics
- **Discarded**: Sway (v), Roll (ϕ, p), Yaw (ψ, r), lateral/directional coupling

This simplifies the model to 4 dynamic states + 2 kinematic states, ideal for studying pitch behavior without lateral complexity.

---

## 📂 Contents

- \`\`: Main script to set initial conditions, call `ode45`, and plot.
- \`\`: Defines 6-state longitudinal equations of motion (u, w, θ, q, x, z).
- **Simulation PNGs**:
  - `Longitudinal_Dynamics_Velocities.png`
  - `Longitudinal_Dynamics_Angles.png`

---

## 🛠 Requirements

- **MATLAB** R2018a or newer
- No extra toolboxes (uses **ode45**)

---

## 📑 Source of Parameters

Parameters & coefficients (mass, inertia, wing area, aerodynamic slopes) come from:

> Kristiansen, M., *Small Unmanned Aircraft: Theory and Practice*, Section E.2: Aerosonde UAV

---

## 🧮 Key Concepts & Equations

### State Variables

| Symbol | Description                      |
| ------ | -------------------------------- |
| u      | Surge (forward) velocity [m/s]   |
| w      | Heave (vertical) velocity [m/s]  |
| θ      | Pitch attitude [rad]             |
| q      | Pitch rate [rad/s]               |
| x      | Horizontal position [m]          |
| z      | Vertical position (altitude) [m] |

### Main Equations

**Translational Dynamics:**

```matlab
du = X_b/m - g*sin(theta) - q*w;
dw = Z_b/m + g*cos(theta) + q*u;
```

**Rotational Dynamics & Kinematics:**

```matlab
dtheta = q;
dq     = M/I_yy;
dx     = u*cos(theta) + w*sin(theta);
dz     = u*sin(theta) - w*cos(theta);
```

### Aerodynamic Forces & Moments

```matlab
alpha = atan2(w,u);                   % Angle-of-attack [rad]
C_L   = C_L0 + C_L_alpha*alpha + C_L_delta_e*delta_e;
C_D   = C_D0 + C_D_alpha*alpha;
C_M   = C_M0 + C_M_alpha*alpha + C_M_delta_e*delta_e;
L     = 0.5*rho*V^2*S*C_L;
D     = 0.5*rho*V^2*S*C_D;
M     = 0.5*rho*V^2*S*c*C_M;
T     = T_max*delta_t;
X_b   = L*sin(alpha) - D*cos(alpha) + T;
Z_b   = -L*cos(alpha) - D*sin(alpha);
```

---

## 🔗 Code-to-Equation Mapping

- \`\`: Implements all derivatives in one `States = [du; dw; dtheta; dq; dx; dz]` block.
- \`\`: Calls `[t,States] = ode45(@EOM_Long, tspan, X0)` and then extracts/plots `u,w,theta,q,x,z`.

---

## ⚙️ Parameters & Values

| Parameter     | Symbol         | Code Value        | Description                        |
| ------------- | -------------- | ----------------- | ---------------------------------- |
| Mass          | m              | 13.5              | UAV mass [kg]                      |
| Gravity       | g              | 9.81              | Gravitational accel [m/s²]         |
| Inertia       | I\_yy          | 1.135             | Pitch-axis inertia [kg·m²]         |
| Wing area     | S              | 0.55              | Reference area [m²]                |
| Air density   | rho            | 1.225             | [kg/m³]                            |
| Aero chord    | c              | 0.19              | Mean aerodynamic chord [m]         |
| Thrust limit  | T\_max         | 0.2*g*m           | Max thrust (0.2×weight) [N]        |
| Elevator trim | delta\_e       | -4.3791\*(pi/180) | Constant deflection [rad]          |
| C\_L0         | C\_L0          | 0.28              | Zero-α lift coeff                  |
| C\_Lα         | C\_L\_alpha    | 3.45              | Lift slope [1/rad]                 |
| C\_D0         | C\_D0          | 0.03              | Zero-α drag coeff                  |
| C\_Dα         | C\_D\_alpha    | 0.3               | Drag slope [1/rad]                 |
| C\_M0         | C\_M0          | -0.024            | Zero-α moment coeff                |
| C\_Mα         | C\_M\_alpha    | -0.38             | Moment slope [1/rad]               |
| C\_Mδ\_e      | C\_M\_delta\_e | -0.5              | Moment per rad elevator deflection |
| Time span     | —              | [0 250]           | Simulation window [s]              |

---

## 📊 Simulation Results

### Velocities & Altitude (Longitudinal\_Dynamics\_Velocities.png)



- **Vertical Position (z)**: Gradual climb from 500 m to \~550 m due to thrust excess.
- **Forward Velocity (u)**: Initial oscillations (\~30 m/s) that damp out to a trimmed speed.
- **Vertical Velocity (w)**: Damped oscillations around steady climb rate (\~1.12 m/s).

### Angles & Rates (Longitudinal\_Dynamics\_Angles.png)



- **Angle of Attack (α)**: Small oscillations around trimmed α ≈ 2.14°.
- **Pitch Angle (θ)**: Damped oscillations converging to \~2.5°.
- **Pitch Rate (q)**: Transient pitch-rate response decays to zero.

These plots illustrate the **phugoid** and **short-period** modes: a slow altitude oscillation (phugoid) and a faster pitch oscillation (short-period).

---

## ✈️ Usage

```bash
git clone https://github.com/yourusername/longitudinal-dynamics.git
cd longitudinal-dynamics
```

1. Open \`\` in MATLAB.
2. Ensure \`\` is in path.
3. Run:
   ```matlab
   run_simulation
   ```
4. View generated PNGs in repo root.

---

## 💡 Applications & Extensions

- **Educational**: Visualize basic flight dynamics modes.
- **Preliminary Design**: Test sensitivity to aerodynamic parameters.
- **Control Development**: Foundation for closed-loop pitch control.

> **Extensions**:
>
> - Add trim solver for level flight.
> - Design PID/LQR controllers for pitch.
> - Expand to **full 6-DOF**: include roll, yaw, and lateral dynamics.

---

## 📝 License & Author

- **License**: MIT (see [LICENSE](LICENSE))
- **Author**: [Nurullah Özkan](https://github.com/nrlhozkan)

