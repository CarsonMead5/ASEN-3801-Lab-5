function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

phi = aircraft_state(4,1);
theta = aircraft_state(5,1);
psi = aircraft_state(6,1);
u = aircraft_state(7,1);
v = aircraft_state(8,1);
w = aircraft_state(9,1);
p = aircraft_state(10,1);
q = aircraft_state(11,1);
r = aircraft_state(12,1);

c_phi = cos(phi);
c_theta = cos(theta);
c_psi = cos(psi);
s_phi = sin(phi);
s_theta = sin(theta);
s_psi = sin(psi);

% calling AeroForcesAndMoments fxn to get forces and moments 
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

% rotation matrix
 R = [
    c_phi * c_psi, (s_phi * s_theta * c_psi) - (c_phi * s_psi), (c_phi * s_theta * c_psi) + (s_phi * s_psi);
    c_phi * s_psi, (s_phi * s_theta * s_psi) + (c_phi * s_psi), (c_phi * s_theta * s_psi) - (s_phi * c_psi);
    -s_theta, s_phi * c_theta, c_phi * c_theta;
  ];

% pdot for x_edot, y_edot, z_edot
pdot = R * [u, v, w];

% odot for phidot, thetadot, psidot
odot = [1, s_phi * tan(theta), c_phi * tan(theta);
        0, c_phi , -s_phi;
        0, s_phi * sec(theta), c_phi *sec(theta)] * [p, q, r];

% vdot for u_edot, v_edot, w_edot
vdot = [(r*v - q*w), (p*w - r*u), (q*u - p*v)] * aircraft_parameters.g*[-s_theta, c_theta*s_psi, c_theta*c_phi] * aircraft_parameters.m *aero_forces;

% I matrix 
I = [aircraft_parameters.Ix, 0 , 0;
    0, aircraft_parameters.Iy, 0;
    aircraft_parameters.Ixz, 0, aircraft_parameters.Iz];

% wdot for pdot, qdot, rdot
w = [p,q,r];
Iw_matrix = I*w;

wdot = inv(I) * (-cross(w,Iw_matrix) + aero_moments);


xdot = [pdot, odot, vdot, wdot];

end
