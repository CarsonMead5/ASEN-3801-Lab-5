function [vardot] = AircraftEOM(t, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
    pE = aircraft_state(1:3);
    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);
    vB = aircraft_state(7:9);
    omegaB = aircraft_state(10:12);
    
    density = stdatmo(-pE(3));
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

    omegaBdot = aircraft_parameters.IB\(aero_moments - cross(omegaB,aircraft_parameters.IB*omegaB));

    vardot = [pEdot; odot; vBdot; omegaBdot];
end
