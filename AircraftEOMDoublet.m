%% Aircraft EOM Function Taking in Doublet
% 

function [xdot] = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size,...
    doublet_time, wind_inertial, aircraft_parameters)

% AIRCRAFT EOM Doublet 
% @brief Computes the state derivative of an Aircraft given the current state and control inputs
% @param t Time (not used in this function but required for ODE solvers)
% @param aircraft_state 12x1 vector of the current state [x; y; z; phi; theta; psi; u; v; w; p; q; r]
% @param  ACc Struct containing FWAC constants (gravity, mass, inertia, control moment coef, etc.)
% @param control_input_array 4x1 vector of control inputs [D_Elevator; D_Aileron; D_Rudder; D_Throttle] (should not be included)
% @param Engine_Force 4x1 vector of motor forces (thrusts) [f1; f2; f3; f4]
% @return aircraft_state_derivative 12x1 vector of the state derivatives
    
    pE = aircraft_state(1:3);
    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);
    vB = aircraft_state(7:9);
    omegaB = aircraft_state(10:12);
    
    if time > 0 && time < doublet_time
        aircraft_surfaces(1) = aircraft_surfaces(1) + doublet_size;
    elseif time > doublet_time && time < 2*doublet_time
        aircraft_surfaces(1) = aircraft_surfaces(1) - doublet_size;
    end

    [~, ~, ~, density] = atmosisa(-pE(3));
    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

    REB = angle2dcm(psi,theta,phi,'ZYX');
    pEdot = REB'*vB;

    T = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
         0 cos(phi) -sin(phi);
         0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    odot = T*omegaB;

    FgB = aircraft_parameters.W*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];
    FB = aero_forces+FgB;
    vBdot = FB/aircraft_parameters.m - cross(omegaB,vB);

    omegaBdot = aircraft_parameters.IB^-1*(aero_moments - cross(omegaB,aircraft_parameters.IB*omegaB));

    xdot = [pEdot; odot; vBdot; omegaBdot];
end
