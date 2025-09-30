# Visual Guide: Classical DTC with SVPWM for PMSM

This repository provides a visual breakdown of a Simulink model for a **Direct Torque Control (DTC) scheme using Space Vector Pulse Width Modulation (SVPWM)** for a Permanent Magnet Synchronous Motor (PMSM). It serves as both an educational guide and a reference for reconstructing or extending the model.

The control strategy follows classical DTC principles with an SVPWM modulator for constant switching frequency, and includes a **sensorless estimation technique** based on an **Adaptive Disturbance Observer–based Sliding Mode Observer (ADO-SMO)**.

---

## Overall System Diagram

The complete control scheme integrates feedback loops for speed, torque, and flux to regulate the PMSM.

![Overall System Diagram](Images/01_overall_system.png)

---

## How It Works

The control system operates by continuously estimating the motor state and adjusting the inverter voltage vectors. The major steps are:

1. **Sensing & Transformation**
   The three-phase stator currents and voltages are transformed into the two-phase stationary (`α-β`) frame using the **Clarke Transformation**:

   [
   \begin{aligned}
   i_{\alpha} &= i_{a} \
   i_{\beta} &= \frac{1}{\sqrt{3}}(i_{a} + 2i_{b})
   \end{aligned}
   ]

   (with ( i_{a} + i_{b} + i_{c} = 0 ) for a balanced system).
   Similar transformations apply for voltages.

2. **Estimation**

   * The **flux estimator** and **torque estimator** compute real-time values of stator flux and electromagnetic torque.
   * The **ADO-SMO** provides sensorless estimation of **rotor speed** and **electrical angle**, eliminating the need for a mechanical encoder.

3. **Control Loops**

   * The **speed error** (reference − estimated) passes through a PI controller to generate reference torque.
   * The **torque and flux errors** are processed through PI regulators to generate reference voltages in the d-q frame.

4. **Pulse Generation**
   An SVPWM block converts the reference voltages into optimized gate pulses for the inverter, ensuring a constant switching frequency and efficient operation.

---

## System Components Breakdown

### Flux Estimator

Calculates the stator flux linkage:

[
\Psi_{d} = L_{d} i_{d} + \Psi_{m}, \quad \Psi_{q} = L_{q} i_{q}, \quad
\Psi_{T} = \sqrt{\Psi_{d}^{2} + \Psi_{q}^{2}}
]

![Flux Estimator](Images/02_flux_estimator.png)

---

### Torque Estimator

Computes the electromagnetic torque:

[
T_{e} = p \big(\Psi_{m} i_{q} + (L_{d} - L_{q}) i_{d} i_{q}\big)
]

![Torque Estimator](Images/03_torque_estimator.png)

---

### ADO-SMO: Speed and Angle Observer

The **Adaptive Disturbance Observer–based Sliding Mode Observer (ADO-SMO)** is used for sensorless estimation of rotor position and speed.
It reconstructs back-EMF and adapts to parameter variations or disturbances.

**Observer equations:**

[
\begin{aligned}
\frac{di_{\alpha}}{dt} &= -\frac{R_{s}}{L_{s}} i_{\alpha} + \frac{1}{L_{s}} v_{\alpha} - \frac{1}{L_{s}} e_{\alpha} \
\frac{di_{\beta}}{dt} &= -\frac{R_{s}}{L_{s}} i_{\beta} + \frac{1}{L_{s}} v_{\beta} - \frac{1}{L_{s}} e_{\beta}
\end{aligned}
]

where (e_{\alpha}, e_{\beta}) are the estimated back-EMFs.

The rotor electrical angle (\theta_{e}) is obtained as:

[
\theta_{e} = \arctan\left(\frac{e_{\beta}}{e_{\alpha}}\right)
]

The rotor speed (\omega_{e}) is derived by differentiating (\theta_{e}) and filtered for noise reduction.
Adaptive gains in the observer compensate for system uncertainties, enhancing robustness compared to a conventional SMO.

![Speed & Angle Observer](Images/ADO_SMO.png)

---

## Governing Equations

The PMSM model is based on the following **d-q reference frame equations**:

* **Voltage Equations:**

[
V_{d} = R_{s}i_{d} + L_{d}\frac{di_{d}}{dt} - \omega L_{q} i_{q}
]

[
V_{q} = R_{s}i_{q} + L_{q}\frac{di_{q}}{dt} + \omega L_{d} i_{d} + \omega \Psi_{m}
]

* **Mechanical Equation:**

[
T_{e} = J\frac{d\omega_{m}}{dt} + B\omega_{m} + T_{L}
]

---

## Results

The following plots illustrate the performance of the implemented scheme:

* Motor speed tracking
* Electromagnetic torque response
* Flux linkage estimation
* Stator currents under dynamic load conditions

![Results\_1a](Images/7b.png)
![Results\_1b](Images/8b.png)
![Results\_1c](Images/9b.png)
![Results\_2a](Images/10b.png)
![Results\_2b](Images/11b.png)
![Results\_2c](Images/12b.png)

These results confirm stable performance, accurate speed/position estimation by the **ADO-SMO**, and smooth torque response thanks to **SVPWM modulation**.

---

## References

1. Research papers on DTC-SVPWM and SMO-based sensorless control of PMSM.
2. "Advanced Model Predictive Direct Torque Control with Space Vector PWM for Permanent Magnet Synchronous Motors."
3. Standard textbooks on electrical machine control for background on Clarke/Park transformations.

---

This guide provides a clear blueprint for implementing and understanding **SVPWM-DTC of PMSM with ADO-SMO observer** in Simulink.
