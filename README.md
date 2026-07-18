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
## Simulation
## Controller Design and Evaluation
## Hardware Implementation
## Final Remarks
