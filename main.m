%% 2.1 
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1609.34];
o0 = [0; 0; 0];
vB0 = [21; 0; 0];
omegaB0 = [0; 0; 0];
x0 = [pE0; o0; vB0; omegaB0];
u0 = [0; 0; 0; 0];
wE = [0; 0; 0];

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOM(t,x,u0,wE,aircraft_parameters), [0 10], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

fig = 1:6;
col = {"b-","","2.1",1};
PlotAircraftSim(tout, xout', u0, fig, col);

%% 2.2
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1800];
o0 = [0; 0.0278; 0];
vB0 = [20.99; 0; 0.5837];
omegaB0 = [0; 0; 0];
x0 = [pE0; o0; vB0; omegaB0];
u0 = [0.1079; 0; 0; 0.3182];
wE = [0; 0; 0];

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOM(t,x,u0,wE,aircraft_parameters), [0 10], x0 ,options); % Run ODE45

fig = 1:6;
col = {"b-","","2.2",1};
PlotAircraftSim(tout, xout', u0, fig, col);

%% 2.3
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1800];
o0 = deg2rad([15; -12; 270]);
vB0 = [19; 3; -2];
omegaB0 = deg2rad([0.08; -0.2; 0]);
x0 = [pE0; o0; vB0; omegaB0];
u0 = deg2rad([5; 2; -13; 0.3]);
wE = [0; 0; 0];

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOM(t,x,u0,wE,aircraft_parameters), [0 10], x0 ,options); % Run ODE45

fig = 1:6;
col = {"b-","","2.3",1};
PlotAircraftSim(tout, xout', u0, fig, col);