# Digital-PID-controller-for-HOTPLATE
This repository is for the case study project done in DSC-14 Control Systems for B.Tech degree at Faculty of Technology, University of Delhi. It involves creating a PID controller but combing the principles of Verilog for making the plant and tuning our model for accuracy and stabilization.

#Digital PID Temperature Controller in Verilog

This project is a complete case study of a digital Proportional-Integral-Derivative (PID) controller implemented entirely in Verilog. The system is designed to control a "virtual hotplate," bringing it to a specific temperature and holding it there, even as it constantly loses heat to the environment.

The project consists of two main Verilog modules:

#pid_controller.v: The "brain" that implements the PID algorithm.

#hotplate.v: The "plant" that simulates the physics of the heater.

The entire system is tested in a Verilog testbench (tb_pid.v), and the results are logged to a file, which can be plotted in MATLAB.

#System Architecture

The project is a closed-loop feedback system. The controller and plant are in a constant "conversation" to achieve their goal.

            [Setpoint]
                |
                v
 [tb_pid.v] -> [Sum] --(Error)--> [pid_controller.v] --(Heater Control Signal)--> [hotplate.v]
                ^                                                                 |
                |                                                                 |
                +------------------(Current Temp)---------------------------------+
                          (Feedback Loop)


Setpoint: The target temperature we want to achieve (e.g., 150°).

Sum: The current_temp is subtracted from the setpoint to get the Error.

Controller: The pid_controller.v module calculates the Error and outputs a heater_power signal (a digital value from 0-255).

Plant: The hotplate.v module takes this heater_power and calculates its new current_temp for the next clock cycle.

Feedback: The new current_temp is fed back to the "Sum" block, and the loop repeats.

#Project Files

pid_controller.v: The core controller logic.

hotplate.v: The virtual plant/heater model.

tb_pid.v: The testbench that connects the controller and plant, generates the clock, and sets the target.

plot_pid_results.m: A MATLAB script to parse the results.txt log file and generate graphs.

#Module Deep Dive

hotplate.v (The Plant)

This module simulates the physics of a real heater using a first-order linear difference equation. It models two key concepts:

Thermal Mass (Heat In): (heater_power >> HEATING_SHIFT)

This bit-shift (dividing by 8) simulates the fact that a real object heats up slowly. It takes a lot of energy to raise the temperature.

Heat Loss (Heat Out): COOLING_RATE

This is a discrete-time model of Newton's Law of Cooling. It simulates the constant heat loss from the hotplate to the 25° room air. This is the "disturbance" the PID controller must fight.

pid_controller.v (The Brain)

This module's job is to eliminate the Error. It calculates three separate terms and adds them together:

P (Proportional) = Kp * error

"The Reaction." Reacts to the present error. It provides the main "push" to get the temperature to the target.

I (Integral) = Ki * (sum of all past errors)

"The Memory." Reacts to the past. This term slowly builds up to eliminate any small, stubborn, long-term error (steady-state error) and holds the temperature steady.

D (Derivative) = Kd * (current_error - last_error)

"The Brake." Reacts to the future. It measures how fast the temperature is changing. If it's rising too quickly (about to overshoot), this term applies a strong "braking" force.

How to Run

Verilog Simulation:

Add all three .v files to a Verilog simulator (e.g., Vivado).

Set tb_pid.v as the top-level simulation module.

Run the simulation. The testbench is configured to create a results.txt file.

Note: The tb_pid.v file is set to run for 150,000 ns (150 µs), which is long enough to see the system stabilize.

MATLAB Plotting:

Copy the results.txt file from the simulation directory.

Place it in the same folder as plot_pid_results.m.

Run the MATLAB script to generate the graphs.

#Case Study: Tuning the PID Gains

The "magic" of this project is finding the right balance for the Kp, Ki, and Kd gains.

Attempt 1: Unstable (High Ki, Low Kp/Kd)

The controller was "all memory, no brakes." The Integral term "wound up" and kept applying power even after passing the setpoint, causing a massive, unstable overshoot.
(Insert your "Unstable" graph here)

Attempt 2: Sluggish (High Kd)

The controller was "all brake, no gas." The Derivative term was so sensitive that it "panicked" and cut the power the instant the temperature started to rise, causing the system to be over-damped and never reach the target.
(Insert your "Sluggish" graph here)

Attempt 3: Stable (Balanced Gains: 20/2/20)

After tuning, we found a balance (Kp=20, Ki=2, Kd=20). The graph shows a classic under-damped response:

The Rise: The P and I terms provide power.

The Overshoot: The temperature passes 150°.

The Response: The D-term "catches" the overshoot and cuts the power.

The Settling: The system "bounces" (oscillates) a few times before the I-term finds the exact power level (e.g., ~24) needed to balance the COOLING_RATE and hold the temperature stable at 150°.
(Insert your "Stable" graph here)

References

GitHub: kra-dean/Verilog-PID-Temp-Controller: A similar practical Verilog implementation of a PID controller and plant model.

Padmanabhan, S., & Banu, R. S. D. W. (2013). "Design and Implementation of an FPGA-Based PID Controller for Temperature Control of a Water Bath.": This paper discusses the formal physics and discrete-time modeling of thermal systems for implementation on FPGAs.
