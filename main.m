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
[tout, xout] = ode45(@(t,x) AircraftEOM(t,x,u0,wE,aircraft_parameters), [0 100], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

fig = 211:216;
col = {"b-","","2.1",1};
% PlotAircraftSim(tout, xout', uout, fig, col);

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
[tout, xout] = ode45(@(t,x) AircraftEOM(t,x,u0,wE,aircraft_parameters), [0 100], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

fig = 221:226;
col = {"b-","","2.2",1};
% PlotAircraftSim(tout, xout', uout, fig, col);

%% 2.3
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1800];
o0 = deg2rad([15; -12; 270]);
vB0 = [19; 3; -2];
omegaB0 = [deg2rad(0.08); -deg2rad(0.2); 0];
x0 = [pE0; o0; vB0; omegaB0];
u0 = [deg2rad(5); deg2rad(2); -deg2rad(13); 0.3];
wE = [0; 0; 0];

options = odeset('AbsTol',1e-10,'RelTol',1e-10);
[tout, xout] = ode45(@(t,x) AircraftEOM(t,x,u0,wE,aircraft_parameters), [0 200], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

fig = 231:236;
col = {"b-","","2.3",1};
PlotAircraftSim(tout, xout', uout, fig, col);

%% 3.1
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1800];
o0 = [0; 0.0278; 0];
vB0 = [20.99; 0; 0.5837];
omegaB0 = [0; 0; 0];
x0 = [pE0; o0; vB0; omegaB0];
u0 = [0.1079; 0; 0; 0.3182];
wE = [0; 0; 0];
doublet_size = deg2rad(15);
doublet_time = 1.5;

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOMDoublet(t,x,u0,doublet_size,doublet_time,wE,aircraft_parameters), [0 3], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

beforeD = tout>0 & tout<doublet_time;
afterD = tout>doublet_time & tout<2*doublet_time;
uout(1,beforeD) = uout(1,beforeD)+doublet_size;
uout(1,afterD) = uout(1,afterD)-doublet_size;

fig = 311:316;
col = {"b-","","3.1",1};
PlotAircraftSim(tout, xout', uout, fig, col);

%% 3.2
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1800];
o0 = [0; 0.0278; 0];
vB0 = [20.99; 0; 0.5837];
omegaB0 = [0; 0; 0];
x0 = [pE0; o0; vB0; omegaB0];
u0 = [0.1079; 0; 0; 0.3182];
wE = [0; 0; 0];
doublet_size = deg2rad(15);
doublet_time = 1.5;

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOMDoublet(t,x,u0,doublet_size,doublet_time,wE,aircraft_parameters), [0 100], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

beforeD = tout>0 & tout<doublet_time;
afterD = tout>doublet_time & tout<2*doublet_time;
uout(1,beforeD) = uout(1,beforeD)+doublet_size;
uout(1,afterD) = uout(1,afterD)-doublet_size;

fig = 321:326;
col = {"b-","","3.2",1};
PlotAircraftSim(tout, xout', uout, fig, col);

%% 3.1
clear; clc; close all;

aircraft_parameters = ttwistor();

pE0 = [0; 0; -1800];
o0 = [0; 0.0278; 0];
vB0 = [20.99; 0; 0.5837];
omegaB0 = [0; 0; 0];
x0 = [pE0; o0; vB0; omegaB0];
u0 = [0.1079; 0; 0; 0.3182];
wE = [0; 0; 0];
doublet_size = deg2rad(15);
doublet_time = 0.25;

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOMDoublet(t,x,u0,doublet_size,doublet_time,wE,aircraft_parameters), [0 6], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

beforeD = tout>0 & tout<doublet_time;
afterD = tout>doublet_time & tout<2*doublet_time;
uout(1,beforeD) = uout(1,beforeD)+doublet_size;
uout(1,afterD) = uout(1,afterD)-doublet_size;

fig = 311:316;
col = {"b-","","3.1",1};
PlotAircraftSim(tout, xout', uout, fig, col);

pitch = xout(:,5);

%%

theta = rad2deg(xout(:,5));
theta = theta-theta(end);
minmax = islocalmin(theta) | islocalmax(theta);
thetaminmax = theta(minmax);
figure; hold on;
plot(tout,abs(theta));
scatter(tout(minmax),abs(thetaminmax),'r','filled');

%% 3.2
clear tout xout uout; clc; close all;

options = odeset('RelTol',1e-6);
[tout, xout] = ode45(@(t,x) AircraftEOMDoublet(t,x,u0,doublet_size,doublet_time,wE,aircraft_parameters), [0 100], x0 ,options); % Run ODE45
uout = repmat(u0,1,length(tout));

beforeD = tout>0 & tout<doublet_time;
afterD = tout>doublet_time & tout<2*doublet_time;
uout(1,beforeD) = uout(1,beforeD)+doublet_size;
uout(1,afterD) = uout(1,afterD)-doublet_size;

fig = 321:326;
col = {"b-","","3.2",1};
PlotAircraftSim(tout, xout', uout, fig, col);
