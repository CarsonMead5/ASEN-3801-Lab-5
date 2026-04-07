function aircraft_state_derivative = AircraftEOM(t, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
% AIRCRAFT EOM CV 
% @brief Computes the state derivative of an Aircraft given the current state and control inputs
% @param t Time (not used in this function but required for ODE solvers)
% @param aircraft_state 12x1 vector of the current state [x; y; z; phi; theta; psi; u; v; w; p; q; r]
% @param  ACc Struct containing FWAC constants (gravity, mass, inertia, control moment coef, etc.)
% @param control_input_array 4x1 vector of control inputs [D_Elevator; D_Aileron; D_Rudder; D_Throttle] (should not be included)
% @param Engine_Force 4x1 vector of motor forces (thrusts) [f1; f2; f3; f4]
% @return aircraft_state_derivative 12x1 vector of the state derivatives
    %% Preallocations
    aircraft_state_derivative = zeros(12,1);

    %% Parse out input args
    pos = aircraft_state(1:3);
    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);
    vel = aircraft_state(7:9);
    ang_rates = aircraft_state(10:12);
    
    [~, ~, ~, density] = atmosisa(pos(3));

    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
   
    %% Position Dot Calculations

    R1 = [ 1      0           0;
       0  cos(phi)  sin(phi);
       0 -sin(phi)  cos(phi) ];

    R2 = [ cos(theta)  0  -sin(theta);
           0       1       0;
       sin(theta)  0   cos(theta) ];

    R3 = [ cos(psi)  sin(psi)  0;
      -sin(psi)  cos(psi)  0;
           0          0    1 ];
    DCM = R3*R2*R1;

    % DCM = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    % cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    % -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

    aircraft_state_derivative(1:3) = DCM'*vel;
    
    %% Angle Dot Calculations

    angDot = ([1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                    0, cos(phi), -sin(phi);
                    0, sin(phi)*sec(theta), cos(phi)*sec(theta)])*ang_rates;
    
    aircraft_state_derivative(4:6) = angDot;
    
    %% Inertial Velocity Dot Calculation 

    velDot = cross(-ang_rates,vel)+(aero_forces./aircraft_parameters.m);

    aircraft_state_derivative(7:9) = velDot;

    %% Angular Velocity Dot Calculation

    omegaDot = (aircraft_parameters.IB^(-1)*(cross(-ang_rates,(aircraft_parameters.IB*ang_rates))+aero_moments));

    aircraft_state_derivative(10:12) = omegaDot;
    

end
