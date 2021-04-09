The content of this repository comes from the masters thesis "Practical comparison of MPC Toolboxes" by Emma Nilsson.

Toolboxes using Model Predictive Control is set up to run simulations in Simulink, Matlab.

All controllers are set up to control a simulation of the Quadruple tank, frequently used by the department of Automatic Control at Lund University. To learn more about the Quadruple tank, how it's set up and how it is modeled, read Quadruple_tank.pdf.

The toolboxes used to set up controllers are Mathworks' Model Predictice Control Toolbox, Multi-Parametric toolbox and MATMPC.

All toolboxes have a basic MPC controller and at least two controllers with different typs of integral action. Mathworks' toolbox and Multi-Parametric toolbox have controllers with gain scheduling while MATMPC have a controller with NMPC. For mor information about the different implementations, look in the folders for the different toolboxes.
