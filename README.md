Simulink Model: Sensorless DTC-SVPWM for PMSM
This repository provides a detailed visual breakdown of a Simulink model for a sensorless Direct Torque Control (DTC) scheme using Space Vector Pulse Width Modulation (SVPWM) for a Permanent Magnet Synchronous Motor (PMSM) ⚙️. This guide helps in understanding the control architecture and reconstructing the model.

The core of the sensorless operation is an advanced observer technique that combines a Sliding Mode Observer (SMO) with an Adaptive Disturbance Observer (ADO) for superior accuracy and robustness. This implementation integrates this powerful observer into a classical DTC-SVPWM framework.

Overall System Diagram
The complete control scheme is shown below. It integrates the ADO-SMO observer with feedback loops for torque and flux to regulate the PMSM without a physical sensor.

How It Works 💡
The control system operates by estimating the motor's rotor position and speed and then adjusting the stator voltage via a three-phase inverter.

Sensing & Transformation: The three-phase stator currents (i 
abc
​
 ) and voltages (V 
abc
​
 ) are measured. They are converted to a two-phase stationary (α-β) frame using a Clarke Transformation.

Sensorless Estimation: An ADO-SMO observer block uses the α-β voltages and currents to estimate the rotor's electrical angle (θ 
est
​
 ) and speed (ω 
est
​
 ).

Torque and Flux Calculation: The actual torque and flux of the motor are calculated in real-time by dedicated estimator blocks using the transformed currents.

Control Loops:

The speed error (reference vs. estimated) is fed to a PI controller to generate a reference torque (T 
ref
​
 ).

The torque error (reference vs. estimated) is fed to a PI controller to generate a reference d-axis voltage (V 
d_ref
​
 ).

The flux error (reference vs. estimated) is fed to a PI controller to generate a reference q-axis voltage (V 
q_ref
​
 ).

Pulse Generation: An SVPWM block uses the reference d-q voltages to generate the precise gate pulses for the inverter, ensuring optimal voltage application and a constant switching frequency.

System Components Breakdown
Clarke Transformation
This is the first step in the control loop, converting the three-phase system variables into a two-phase stationary reference frame.

Currents:


i 
α
​
 = 
3
2
​
 (i 
a
​
 − 
2
1
​
 i 
b
​
 − 
2
1
​
 i 
c
​
 )
i 
β
​
 = 
3
​
 
1
​
 (i 
b
​
 −i 
c
​
 )
Voltages:


V 
α
​
 = 
3
2
​
 (V 
a
​
 − 
2
1
​
 V 
b
​
 − 
2
1
​
 V 
c
​
 )
V 
β
​
 = 
3
​
 
1
​
 (V 
b
​
 −V 
c
​
 )
Torque and Flux Estimators
These blocks provide the essential feedback values for the control loops.

Flux Estimator: This subsystem calculates the total stator flux linkage (Ψ 
T
​
 ) from the d-q axis currents. The calculation is based on the equations Ψ 
d
​
 =L 
d
​
 i 
d
​
 +Ψ 
m
​
  and Ψ 
q
​
 =L 
q
​
 i 
q
​
 , followed by Ψ 
T
​
 = 
Ψ 
d
2
​
 +Ψ 
q
2
​
 
​
 .

Torque Estimator: This subsystem calculates the motor's electromagnetic torque (T 
e
​
 ). It implements the equation T 
e
​
 = 
2
3
​
 p(Ψ 
m
​
 i 
q
​
 +(L 
d
​
 −L 
q
​
 )i 
d
​
 i 
q
​
 ).

Speed & Angle Observer (ADO-SMO)
This block is the core of the sensorless control scheme. It uses a Sliding Mode Observer (SMO) for initial estimation and an Adaptive Disturbance Observer (ADO) for enhanced accuracy and robustness.

1. Sliding Mode Observer (SMO) Estimation

The SMO first estimates the flux linkage in the α-β frame:


Ψ 
α
​
 =∫(V 
α
​
 −R 
s
​
 i 
α
​
 )dt
Ψ 
β
​
 =∫(V 
β
​
 −R 
s
​
 i 
β
​
 )dt
From the flux, the mechanical angle (θ 
m
​
 ) and initial speed estimate (ω 
o
​
 ) are calculated:


θ 
m
​
 =tan 
−1
 ( 
Ψ 
β
​
 
Ψ 
α
​
 
​
 )
ω 
o
​
 = 
(Ψ 
α
2
​
 +Ψ 
β
2
​
 )
(Ψ 
α
​
  
dt
dΨ 
β
​
 
​
 −Ψ 
β
​
  
dt
dΨ 
α
​
 
​
 )
​
 
2. Adaptive Disturbance Observer (ADO) Correction

The conventional SMO suffers from issues like chattering and parameter sensitivity. The ADO is introduced to correct the speed estimation, improving accuracy and reducing these issues. The final estimated speed is given by:


ω 
est
​
 =(( 
J
T 
est
​
 
​
 )−( 
J
B
​
 ))ω 
o
​
 

where J is inertia and B is the friction coefficient.

Governing Equations
The PMSM model itself is based on the following fundamental d-q frame equations.

Voltage Equations:

V 
d
​
 =R 
s
​
 i 
d
​
 +L 
d
​
  
dt
di 
d
​
 
​
 −ωL 
q
​
 i 
q
​
 

V 
q
​
 =R 
s
​
 i 
q
​
 +L 
q
​
  
dt
di 
q
​
 
​
 +ωL 
d
​
 i 
d
​
 +ωΨ 
m
​
 

Mechanical Equation:

T 
e
​
 =J 
dt
dω
​
 +Bω+T 
L
​
 

Simulation Results 📈
The following graphs demonstrate the excellent performance of the ADO-SMO observer and control system under various dynamic operating conditions.

A. Constant Torque with Linear Increase in Speed
The motor maintains a steady speed of 1200 rpm and then increases linearly, while the external load torque remains constant at 8 Nm.

B. Constant Torque with Step Change in Speed
The motor speed is steady at 1200 rpm and then abruptly jumps to 1800 rpm. The system maintains a constant torque of 8 Nm and stable flux linkage throughout the transition.

C. Constant Speed with Linear Increase in Torque
The motor speed is held constant at 1200 rpm. The external load torque starts at 5 Nm and then increases linearly.

D. Constant Speed with Step Change in Torque
The motor speed operates steadily at 1200 rpm. The external load torque is initially 5 Nm and experiences a step increase to 8 Nm. The controller maintains stable speed and flux.
