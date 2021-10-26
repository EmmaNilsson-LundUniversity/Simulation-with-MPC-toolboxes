Mathworks' Model Predictive Control Toolbox can use states as input to the controller by choosing "Use custom estimated states instead of measured outputs" in the MPC block. The other option is to use the process' outputs as input to the controller block. In the "Basic controller" and the "Integral action" folders, both ways are used, while in the "Gain Scheduling" folder only states as inputs for the controller are used.

In Mathworks' toolbox, constraints are set for the outputs and can not be set for the states. To be able to set constraints on all four tanks the C-matrix is changed to C= [kc1 0 0 0; 
                           0 kc2 0 0; 
                           0 0 kc3 0; 
                           0 0 0 kc4] so that all the water levels is considered outputs.

The weights are set for the outputs, inputs, and slew rate. Even if the controller takes states as input the toolbox still dont accept weights on the states.

As the goal is to control the water level in the lower tanks, the water level in the upper tanks is not relevant as long as they dont overflow. With the new C-matrix, all the water levels are outputs, and in this toolbox, all outputs require reference signals. To solve this, the upper tanks were given reference signals but the weights to these outputs were set to zero.
