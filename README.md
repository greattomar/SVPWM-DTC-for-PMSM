# Visual Guide: Classical DTC with SVPWM for PMSM

This repository provides a visual breakdown of a Simulink model for a **Direct Torque Control (DTC) scheme using Space Vector Pulse Width Modulation (SVPWM)** for a Permanent Magnet Synchronous Motor (PMSM). It serves as both an educational guide and a reference for reconstructing or extending the model.

The control strategy follows classical DTC principles with an SVPWM modulator for constant switching frequency, and includes a **sensorless estimation technique** based on an **Adaptive Disturbance Observer–based Sliding Mode Observer (ADO-SMO)**.

---

## Overall System Diagram

The complete control scheme integrates feedback loops for speed, torque, and flux to regulate the PMSM.

![Overall System Diagram](Images/01_overall_system.png)

---


## How It Works

The control system operates by continuously estimating the motor state and adjusting inverter voltage vectors.

1. **Sensing & Transformation**
   The three-phase stator currents and voltages are transformed into the two-phase stationary (α-β) frame using the **Clarke Transformation**. For a balanced system ((i_a + i_b + i_c = 0)), the power-invariant form is:

   $$
   \begin{aligned}
   i_{\alpha} &= i_{a},[4pt]
   i_{\beta}  &= \tfrac{1}{\sqrt{3}},(i_{a} + 2,i_{b}).
   \end{aligned}
   $$

   (A full 3×3 Clarke matrix is used if zero-sequence needs to be preserved.)

---

### Flux Estimator

Calculates the stator flux linkage:

$$
\Psi_{d} = L_{d} i_{d} + \Psi_{m}, \qquad
\Psi_{q} = L_{q} i_{q}, \qquad
\Psi_{T} = \sqrt{\Psi_{d}^{2} + \Psi_{q}^{2}}
$$

---

### Torque Estimator

Computes the electromagnetic torque:

$$
T_{e} = p \big(\Psi_{m} i_{q} + (L_{d} - L_{q}) i_{d} i_{q}\big)
$$

---

### ADO-SMO (Speed & Angle Observer)

We model the stator currents (α-β frame) as:

$$
\begin{aligned}
L_s \frac{d i_{\alpha}}{dt} &= -R_s i_{\alpha} + v_{\alpha} - e_{\alpha},[4pt]
L_s \frac{d i_{\beta}}{dt}  &= -R_s i_{\beta} + v_{\beta} - e_{\beta},
\end{aligned}
$$

where (e_{\alpha}, e_{\beta}) are the back-EMF components (treated as disturbances).

The **Sliding Mode Observer (SMO)** with adaptive disturbance compensation (ADO) is written:

$$
\begin{aligned}
L_s \frac{d\hat{i}*{\alpha}}{dt} &= -R_s \hat{i}*{\alpha} + v_{\alpha} - \hat{e}*{\alpha} + g*{\alpha}(\tilde{i}*{\alpha}),[4pt]
L_s \frac{d\hat{i}*{\beta}}{dt}  &= -R_s \hat{i}*{\beta}  + v*{\beta} - \hat{e}*{\beta}  + g*{\beta}(\tilde{i}_{\beta}),
\end{aligned}
$$

with estimation error (\tilde{i} = i - \hat{i}).

The back-EMFs in the stationary frame are related to rotor angle and speed:

$$
\begin{aligned}
e_{\alpha} &= -\omega_e \Psi_m \sin(\theta_e),[4pt]
e_{\beta}  &=  \omega_e \Psi_m \cos(\theta_e).
\end{aligned}
$$

From estimated back-EMFs:

$$
\hat{\theta}*e = \operatorname{atan2}(\hat{e}*{\beta},,\hat{e}_{\alpha}),
\qquad
\hat{\omega}_e = \frac{d\hat{\theta}_e}{dt}\ \text{(filtered)}.
$$

The adaptive ADO terms improve robustness against parameter mismatch and load disturbance.

---

## Governing Equations

The PMSM model (d-q frame):

**Voltage equations**

$$
V_{d} = R_{s}i_{d} + L_{d}\tfrac{di_{d}}{dt} - \omega_e L_{q} i_{q},
$$

$$
V_{q} = R_{s}i_{q} + L_{q}\tfrac{di_{q}}{dt} + \omega_e L_{d} i_{d} + \omega_e \Psi_{m}.
$$

**Mechanical equation**

$$
T_{e} = J\tfrac{d\omega_{m}}{dt} + B\omega_{m} + T_{L}.
$$

---

## Results

Simulation results validate the DTC-SVPWM with ADO-SMO observer.

![Results\_1a](Images/7b.png)
*Figure 1: Motor speed response. The estimated speed closely tracks the reference speed step.*

![Results\_1b](Images/8b.png)
*Figure 2: Electromagnetic torque waveform showing dynamic torque response under load change.*

![Results\_1c](Images/9b.png)
*Figure 3: Stator flux magnitude estimation confirming stability and smooth flux trajectory.*

![Results\_2a](Images/10b.png)
*Figure 4: Stator phase current waveforms under SVPWM switching — nearly sinusoidal with low ripple.*

![Results\_2b](Images/11b.png)
*Figure 5: Speed estimation performance of ADO-SMO observer under transient conditions.*

![Results\_2c](Images/12b.png)
*Figure 6: Rotor position estimation by ADO-SMO, demonstrating accurate angle tracking without mechanical sensor.*
