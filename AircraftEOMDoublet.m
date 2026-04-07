%% Aircraft EOM Function Taking in Doublet
% 

function [xdot] = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size,...
    doublet_time, wind_inertial, aircraft_parameters)

    % Pulling out aircraft vectors
    position = aircraft_state(1:3);
    euler = aircraft_state(4:6);
    velocity = aircraft_state(7:9);
    angular = aircraft_state(10:12);

    % Calculating doublet addition
    if time > 0 && time < doublet_time
        aircraft_surfaces(1) = aircraft_surfaces(1) + doublet_size;
    elseif time > doublet_time && time < 2*doublet_time
        aircraft_surfaces(1) = aircraft_surfaces(1) - doublet_size;
    end

    % 















    function Inertial2Body(body_vec,inert_vec)




    end

end
