function [ mpcobj ] = makeMPC(h1, h2)

% This function will:  -find the stationary operating point for
%                       h1 and h2
%                      -make a liniarized model for the
%                       quadruple tank.
%                      -create an MPC-object with Mathworks 
%                       Model Predictive Control Toolbox
%                      -return the created MPC-object

%               Process parameters

% Cross-section of the tanks
A1=4.9; % [cm^2]
A2=4.9;
A3=4.9;
A4=4.9;

% Cross-section of the outlet holes
a1=0.06; % [cm^2]
a2=0.06;
a3=0.06;
a4=0.06;

% Relation between the actual water level and the sensor measurment
kc1=0.5; % [V cm^{-1}]
kc2=0.5;
kc3=0.5;
kc4=0.5;
kc=[kc1 kc2 kc3 kc4];

% Gravity
g=981;  % [cm s^{-2}]

%Relation between the voltage to the pumps and the water flow
k1=1.6; % [cm^3 V^{-1} s^{-1}]
k2=1.6;

g1=0.3; % ratio of allocated pump capacity between upper
g2=0.3; % and lower tank [1]



%                   Linearization point

u1 = sqrt(2*g)*((g2-1)*a2*sqrt(h2)+g2*a1*sqrt(h1))/(k1*(g1+g2-1)); % [V]
u2 = sqrt(2*g)*((g1-1)*a1*sqrt(h1)+g1*a2*sqrt(h2))/(k2*(g1+g2-1)); % [V]
h3 = (k2*u2*(1-g2)/a3)^2/(2*g); % [cm]
h4 = (k1*u1*(1-g1)/a4)^2/(2*g); % [cm]
 
%                  Build state space model

T1=A1/a1*sqrt(2*h1/g);
T2=A2/a2*sqrt(2*h2/g);
T3=A3/a3*sqrt(2*h3/g);
T4=A4/a4*sqrt(2*h4/g);

A=[-1/T1  0   A3/(A1*T3)   0;
    0   -1/T2    0      A4/(A2*T4);
    0     0    -1/T3       0;
    0     0      0       -1/T4];

B=[g1*k1/A1       0;
     0        g2*k2/A2;   
     0      (1-g2)*k2/A3;
  (1-g1)*k1/A4    0];

C=[kc1  0   0   0; 
    0  kc2  0   0;
    0   0  kc3  0;
    0   0   0  kc4];

D=zeros(4,2);

Ts = 0.5; % sample time


% Discretize model
G = c2d(ss(A,B,C,D),Ts);

% sets no. of input signals to two and output signals to four
G = setmpcsignals(G,'MV',2,'MO',4);



% Prediction parameters
Hp = 60;  % Prediction horizon
Hu = 20;  % Horizon for varying input signal

%                  Constraints

% limit delta u
du_max = {inf inf};
du_min = {-inf -inf};

% limit absolute value of u
u_max = {10-u1 10-u2};
u_min = {-u1 -u2};

% limit constrained outputs
y_max = {(15-h1)*kc1 (15-h2)*kc2 (15-h3)*kc3 (15-h4)*kc4};
y_min = {-h1*kc1 -h2*kc2 -h3*kc3 -h4*kc4};


MV = struct('Min',u_min,'Max',u_max,'RateMin',du_min,'RateMax', du_max);
OV = struct('Min',y_min,'Max',y_max);

%                   Weigths
Weights.OV = [10 10 0 0] ; % Weights on y (Output signals)
Weights.MV = [1 1]; % Weigths on u (Input signals)
Weights.ManipulatedVariablesRate = [3 3]; % Weigths on delta u (slew rate)

%                   Creating MPC-object
mpcobj = mpc(G,Ts,Hp,Hu, Weights,MV,OV);

%              Setting noise- and disturbance models to zero
setoutdist(mpcobj,'model',tf(zeros(4,1)));
mpcobj.Model.Noise = tf(zeros(4,1));

% When the user wants to provide all states for the controller
setEstimator(mpcobj,'custom'); %In MPC-block, make sure that "Use custom estimated states instead of measured outputs" option is set.

end


