# Mars Climate Orbiter

Design and analysis of different basic control schemes for the linearized model of the Mars Climate Orbiter [(MCO)](https://it.wikipedia.org/wiki/Mars_Climate_Orbiter)

<p align="center">
  <img src="MCO.jpg">
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
1. Run the **MCO_library.m** MATLAB file to obtain all the required matrices for Simulink schemes
2. Open the **MCO_control_scheme.slx** Simulink file to simulate/analyse/compare the performance of the different controller schemes


## Design of LQR-Based Controller
For a better understanding of MATLAB computations and Simulink connections please take a look of [MCO_control_scheme_analysis.pdf](https://github.com/francesca-cantoni/Mars_Climate_Orbiter/blob/master/MCO_control_scheme_analysis.pdf)

## Autor
**Francesca Cantoni:** 	francescacantoni95@gmail.com
