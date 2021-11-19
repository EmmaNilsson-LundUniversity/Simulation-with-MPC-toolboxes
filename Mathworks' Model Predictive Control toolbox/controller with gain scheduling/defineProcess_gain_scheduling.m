% % % Model Predictiv Control for Quadruple tank % % %

%This script will:  -initialize all parameters needed for 
%                   running the simulation block.
%                   -create the MPC-objects with the 
%                   function makeMPC


clear all

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

Ts=0.5; % sample time


%                   Creating MPC-objects
mpcobj1 = makeMPC(4,4);
mpcobj2 = makeMPC(4,8);
mpcobj3 = makeMPC(4,12);
mpcobj4 = makeMPC(8,4);
mpcobj5 = makeMPC(8,8);
mpcobj6 = makeMPC(8,12);
mpcobj7 = makeMPC(12,4);
mpcobj8 = makeMPC(12,8);
mpcobj9 = makeMPC(12,12);

%                   Setting starting switch value

i=1; % Assuming a low water level in both tanks when starting

%                   Setting starting stationary operating point

h1 = 8;  % [cm] % using switch value i=1, the mpc-object when
h2 = 8;  % [cm] % simulation starts uses this stationary operating point
u1 = sqrt(2*g)*((g2-1)*a2*sqrt(h2)+g2*a1*sqrt(h1))/(k1*(g1+g2-1)); % [V]
u2 = sqrt(2*g)*((g1-1)*a1*sqrt(h1)+g1*a2*sqrt(h2))/(k2*(g1+g2-1)); % [V]
h3 = (k2*u2*(1-g2)/a3)^2/(2*g); % [cm]
h4 = (k1*u1*(1-g1)/a4)^2/(2*g); % [cm]