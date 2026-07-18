# Inverted Pendulum With Flywheel
Single DOF Controls Project for AE 4610 (Dynamics and Controls Lab) for Spring 2026.

In AE 4610, I was tasked with constructing a single degree-of-freedom (DOF) system that could be controlled with a single controller. The purpose of this project was to expose the class to designing and implementing. This was a group projected, where I was partnered with Siddharth Balraj, Anushka Dharmasanam, Jonah Isaza, and Bowen Lewis. We were limited to a certain amount of hardware that the lab could provide (microcontrollers, sensors, and actuators) to us and $50 to spend on commerical materials. The project had four components:
1. System dynamics modeling - physics based modeling, system ID, etc.
2. Simulation
3. Controller design and evaluation
4. Physical hardware implementation

## Project Ideation and Proposal
After some brainstorming, the group decided on an inverted pendulum that spins in one plane (SDOF) that is to be held upright, inspired by the following YouTube video:\
[![Self balancing with reaction wheels (open source)](https://img.youtube.com/vi/wacuNeEO2zE/0.jpg)](https://www.youtube.com/watch?v=wacuNeEO2zE)

The goals of the project were devised as follows:
- Create an active motor control to keep the stand within 3 degrees of its upright position.
- The stand will need to angle left and right, using a motor to rotate it towards the vertical. The stand will stay upright, even as mass is added to one side of the system to change the balance.
- The stand should be able to take impacts and return to an upright position. The force applied to the stand will vary (stand will be pushed by hand).

We intended to begin with a proportional-derivative (PD) controller due to its simplicity, then if needed extend to a proportional-integral-derivative (PID) controller. PID controllers are typically more expensive and complicated than a PD, so we chose a simplified starting point. We chose the following controller specifications for the project:
- Set Point: 0 degrees in vertical/upright position 
- Stability duration: Maintain balance for at least 30 seconds 
- Rise time: 1 second 
- Settling Time: 2 seconds - the device should stabilize and stop major oscillations from a leaning start or small external force 
- Steady state error band: +/- 3 degrees from the upright position 
- Maximum overshoot: Less than 15% - the device should not swing past the vertical point excessively

Using the video above as a loose model, we came up with a list of parts that would be required. We did not need many parts, as B.L. elected to 3D print the pendulum on his own 3D printer. We did need, however, a motor, an arduino computer, an IMU to read the pendulum position, a power supply unit, and stock electric parts like a breadboard and wires.\
<img width="826" height="192" alt="image" src="https://github.com/user-attachments/assets/a83cec5a-1797-471f-9bfa-7c9eeabaefde" />

All of this information was included in a project proposal that was submitted and approved by the class TAs. This proposal is also attached in the repository.

## System Dynamics Modeling
This section was a collaborative effort between myself and J.I., with a little input from B.L. The following image shows a simplified model of the system used for the math behind the system:\
<img width="265" height="397" alt="image" src="https://github.com/user-attachments/assets/bd1fc88e-7df9-4824-a168-ccb28c190498" />

The assumptions made to simplify the mathematics are as follows:
1. Motion is restricted to a single plane.
2. The system can be modeled as a single degree of freedom system.
3. The inverted pendulum is rigid.
4. The reaction wheel is rigid and spins about a fixed axis on its attachment to the inverted pendulum.
5. "Motor torque" refers to the system input.
6. Balancing occurs around the vertical position, so the small angle approximation can be used.
7. All other forces and considerations (drag, friction, etc.) are ignored.

The full derivation of the equations of motion (EOMs) can be found in the report attached to this repository. The final EOMs in state-space form are as follows:\
<img width="255" height="218" alt="image" src="https://github.com/user-attachments/assets/5e07ba46-cc81-4078-9aa5-f8677d899f73" />

**Note:** $x_1$ represents the angle of the arm, $x_2$ represents the angular speed of the arm, $x_3$ represents the speed of the system as a whole, and $u$ represents the system input.

After B.L. printed the hardware, each piece was measured and the parameters were found to be the following values:
- $m=0.23746 \ kg$
- $l=0.153 \ m$
- $J_p=0.004514 \ kgm^2$
- $J_w=4.73315∗10^{−4} \ kgm^2$

Thus, the numerical EOMs are:\
<img width="297" height="86" alt="image" src="https://github.com/user-attachments/assets/345a6a7d-06f3-4334-819b-dd94249ba0d5" />

## Simulation
This section was primarily completed by S.B. He developed the following SIMULINK model of the system:\
<img width="810" height="330" alt="image" src="https://github.com/user-attachments/assets/fe6e7051-e6b8-4d5e-af77-ad0ff61b9fd0" />

This model simulates the system's reaction to a disturbance via a pulse of magnitude $-0.1 \ rad$ applied for one second. The model runs this input through the state space model generated in the previous section. The simulation pulls plots of arm angle (theta), wheel angle (omega), and motor torque (tau). Bryson's rule was applied with $\theta_{max}=0.052 \ rad$, $\dot\theta_{max}=0.1 \ rad/s$, $\omega_{max}=2\pi \ rad/s$, and $U_{max}=5$ to generate values for the system parameters K. These were tweaked until K produced desirable results. The final chosen K values were:
$K=[-56.607 \ -11.658 \ -0.159]$.
These gain values yielded the following graphs through the SIMULINK model:\
<img width="593" height="388" alt="image" src="https://github.com/user-attachments/assets/89c62838-2265-42e0-9ff3-67db4723f69c" />
<img width="597" height="400" alt="image" src="https://github.com/user-attachments/assets/e838685c-a463-4dbd-8cf3-6b6fca352be3" />
<img width="613" height="382" alt="image" src="https://github.com/user-attachments/assets/26bea24b-67ba-4eb5-ad35-2ec4c91f1485" />

Investigating these plots in MATLAB yields the results:
- $t_r<1 \ s$, meeting the one second maximum rise time goal.
- $t_s=8 \ s$, well exceeding the two second settling time goal.
- $e_{ss}=0$, well within the three degree goal.
- $4.7^\circ$ overshoot, within the thirteen degree goal.
- $5.7 \ Nm$ torque.

Although the settling time goal was not met, the simulation yielded valuable values of the controller gains K and gave us insight into how the system might behave when applied to real hardware.

## Controller Design and Evaluation
## Hardware Implementation
## Final Remarks
