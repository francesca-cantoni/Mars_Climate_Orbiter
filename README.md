# Mars Climate Orbiter

Design and analysis of different basic control schemes for the linearized model of the Mars Climate Orbiter [(MCO)](https://it.wikipedia.org/wiki/Mars_Climate_Orbiter)

<p align="center">
  <img src="MCO.jpg">
 </p>

  
## Prerequisites
- MATLAB R2018b
- Simulink

## Content
- **MCO_library.m**: MATLAB file for generating all the necessary matrices for all the controllers
- "**MCO_control_scheme.slx**": Simulink file which implements all the following controllers:
  - LQR-Based Controller with a full state space observer
  - LQR-Based Controller with a reduced state space observer
  - LQR-Based Controller with a dynamic compensator
  - LQR-Based Controller with a disturbance observer in case of constantand sinusoidal disturbances
  
 - **MCO_control_scheme_analysis.pdf**: PDF file that contains a detailed explanation of the design stages and requirements of each controller
  
## How to run it
1. Run **MCO_library.m**
2. Open **MCO_control_scheme.slx** to simulate/analyze/compare the performances of the different controllers


## Design of LQR-Based Controller
For a better understanding of MATLAB computations and Simulink connections please take a look of [MCO_control_scheme_analysis.pdf](https://github.com/francesca-cantoni/Mars_Climate_Orbiter/blob/master/MCO_control_scheme_analysis.pdf)

## Autor
**Francesca Cantoni:** 	francescacantoni95@gmail.com
