# Mars Climate Orbiter

Design and analysis of different basic control schemes for the linearized model of the Mars Climate Orbiter [(MCO)](https://it.wikipedia.org/wiki/Mars_Climate_Orbiter).

<p align="center">
  <img src="Images/MCO.jpg">
 </p>

The control schemes implemented in "**MCO_control_scheme.slx**" are:
- LQR-Based Controller with a full state space observer
- LQR-Based Controller with a reduced state space observer
- LQR-Based Controller with a dynamic compensator
- LQR-Based Controller with a disturbance observer in case of:
  - constant disturbance
  - sinusoidal disturbance
  
## Prerequisites
- MATLAB R2018b
- Simulink

## How to run it
1. Run **MCO_library.m** MATLAB file to obtain all the necessary matrices
2. Open **MCO_control_scheme.slx** Simulink scheme to simulate the behaviour of the MCO model with different types of controller


## Design of LQR-Based Controller
For a better understanding of MATLAB computations and Simulink connections please take a look of [MCO_control_scheme_analysis.pdf](https://github.com/francesca-cantoni/Mars_Climate_Orbiter/blob/master/MCO_control_scheme_analysis.pdf)

## Autor
**Francesca Cantoni:** 	francescacantoni95@gmail.com
