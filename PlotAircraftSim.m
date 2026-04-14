%% Plotting Aircraft Sim Data
% Contributors: Carson Mead

function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

% Inputs:
%   time - vector of time corrresponding to nth set of variables
%   aircraft_state_array - 12*n state vector of positions, attitude,
%       velocities, and angular velocities
%   control_input_array - 4*n array of control inputs for each time
%   fig - 6*1 vector of figure numbers to plot over
%   col - indicates the line color plotting option for specified run
%       col{1} - color and line style
%       col{2} - title add-on
%       col{3} - legend item name
%       col{4} - 0 if you don't want the ylim function to run

% Outputs:
%   6 Figures:
%       Inertial Position vs. Time, Euler Angles vs. Time, Inertial
%       Velocity vs. Time, Angular Velocity vs. Time, Control Inputs vs.
%       Time, and a 3D Path Plot

% Establishing Optional Plotting Variables
lwidth = 1.5; % Figure linewidth
markSize = 50; % 3D plot marker size
save_figures = 1; % Boolean to save figures or not

%% Plotting Inertial Position vs. Time

figure(fig(1));
if (length(col) > 1) 
    sgtitle(col{2} + "Inertial Position vs. Time","FontSize",10);
else 
    sgtitle("Inertial Position vs. Time","FontSize",10);
end
subplot(3,1,1);
plot(time, aircraft_state_array(1,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("x_E (m)");
adjustYlim(aircraft_state_array(1,:),col);
subplot(3,1,2);
plot(time, aircraft_state_array(2,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("y_E (m)");
adjustYlim(aircraft_state_array(2,:),col);
subplot(3,1,3);
h = plot(time, aircraft_state_array(3,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("z_E (m)");
xlabel("time (s)");
set(gca,'YDir','reverse');
adjustYlim(aircraft_state_array(3,:),col);
addLegend(h,col,1);
exportgraphics(gcf,"./Figures/Fig " + fig(1) + "_Inertial Position vs Time.pdf");

%% Plotting Euler Angles vs. Time

figure(fig(2));
if (length(col) > 1)
    sgtitle(col{2} + "Euler Angles vs. Time","FontSize",10)
else
    sgtitle("Euler Angles vs. Time","FontSize",10)
end
subplot(3,1,1);
plot(time, aircraft_state_array(4,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\phi (rad)");
adjustYlim(aircraft_state_array(4,:),col);
subplot(3,1,2);
plot(time, aircraft_state_array(5,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\theta (rad)");
adjustYlim(aircraft_state_array(5,:),col);
subplot(3,1,3);
h = plot(time, aircraft_state_array(6,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\psi (rad)");
xlabel("time (s)");
adjustYlim(aircraft_state_array(6,:),col);
addLegend(h,col,1);
exportgraphics(gcf,"./Figures/Fig " + fig(2) + "_Euler Angles vs Time.pdf");
%% Plotting Body Frame Velocity vs. Time

figure(fig(3));
if (length(col) > 1)
    sgtitle(col{2} + "Body Frame Velocity vs. Time","FontSize",10)
else
    sgtitle("Body Frame Velocity vs. Time","FontSize",10)
end
subplot(3,1,1);
plot(time, aircraft_state_array(7,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("u^E (m/s)");
adjustYlim(aircraft_state_array(7,:),col);
subplot(3,1,2);
plot(time, aircraft_state_array(8,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("v^E (m/s)");
adjustYlim(aircraft_state_array(8,:),col);
subplot(3,1,3);
h = plot(time, aircraft_state_array(9,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("w^E (m/s)");
xlabel("time (s)");
set(gca,'YDir','reverse');
adjustYlim(aircraft_state_array(10,:),col);
addLegend(h,col,1);
exportgraphics(gcf,"./Figures/Fig " + fig(3) + "_Body Velocity vs Time.pdf");

%% Plotting Angular Velocity vs. Time

figure(fig(4));
if (length(col) > 1)
    sgtitle(col{2} + "Angular Velocity vs. Time","FontSize",10)
else
    sgtitle("Angular Velocity vs. Time","FontSize",10)
end
subplot(3,1,1);
plot(time, aircraft_state_array(10,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("p (rad/s)");
adjustYlim(aircraft_state_array(10,:),col);
subplot(3,1,2);
plot(time, aircraft_state_array(11,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("q (rad/s)");
adjustYlim(aircraft_state_array(11,:),col);
subplot(3,1,3);
h = plot(time, aircraft_state_array(12,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("r (rad/s)");
xlabel("time (s)");
adjustYlim(aircraft_state_array(12,:),col);
addLegend(h,col,1);
exportgraphics(gcf,"./Figures/Fig " + fig(4) + "_Angular Velocity vs Time.pdf");
%% Plotting Control Inputs vs. Time

figure(fig(5));
if (length(col) > 1)
    sgtitle(col{2} + "Control Inputs vs. Time","FontSize",10);
else
    sgtitle("Control Inputs vs. Time","FontSize",10);
end
subplot(4,1,1);
plot(time, control_input_array(1,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\delta_e (°)");
adjustYlim(control_input_array(1,:),col);
subplot(4,1,2);
plot(time, control_input_array(2,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\delta_a (°)");
adjustYlim(control_input_array(2,:),col);
subplot(4,1,3);
plot(time, control_input_array(3,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\delta_r (°)");
adjustYlim(control_input_array(3,:),col);
subplot(4,1,4);
h = plot(time, control_input_array(4,:), col{1}, LineWidth=lwidth); hold on;
grid on;
ylabel("\delta_t (n.d.)")
xlabel("time (s)");
adjustYlim(control_input_array(4,:),col);
addLegend(h,col,1);
exportgraphics(gcf,"./Figures/Fig " + fig(5) + "_Control Surface Inputs vs Time.pdf");
%% Plotting 3D Path

figure(fig(6));
hold on;
grid on;
scatter3(aircraft_state_array(1,1),aircraft_state_array(2,1),aircraft_state_array(3,1),markSize,'green','filled','o','HandleVisibility', 'off');
scatter3(aircraft_state_array(1,end),aircraft_state_array(2,end),aircraft_state_array(3,end),markSize,'red','o','HandleVisibility','off',LineWidth=2);
h = plot3(aircraft_state_array(1,:),aircraft_state_array(2,:),aircraft_state_array(3,:),col{1},LineWidth=lwidth);
addLegend(h,col,2);
if (length(col) > 1)
    title(col{2} + "Quadrotor 3D Path","FontSize",10);
else
    title("Quadrotor 3D Path","FontSize",10);
end
xlabel("x_E");
ylabel("y_E");
zlabel("z_E");
set(gca, 'ZDir', 'reverse')
adjustXlim(aircraft_state_array(1,:));
adjustYlim(aircraft_state_array(2,:),col);
adjustZlim(aircraft_state_array(3,:));
view(30,30);
exportgraphics(gcf,"./Figures/Fig " + fig(6) + "_Quadrotor 3D Path.pdf");
function adjustXlim(var)
    curLim = xlim;
    newMin = min(var);
    newMax = max(var);
    
    % If data is near-zero, use a default window
    if (newMax < 0.5) && (newMin > -0.5)
        targetMin = -1;
        targetMax = 1;
    else
        targetMin = newMin - 1;
        targetMax = newMax + 1;
    end
    
    % If this is the first plot, just set it. 
    % If not, take the outer envelope of old and new.
    if isequal(curLim, [0 1])
        xlim([targetMin, targetMax]);
    else
        xlim([min(curLim(1), targetMin), max(curLim(2), targetMax)]);
    end
end

% adjustYlim function (Expansion Logic)
function adjustYlim(var,col)
    curLim = ylim;
    newMin = min(var);
    newMax = max(var);

    if length(col) > 3
        if col{4} == 0
            return;
        end
    end

    if (newMax < 0.5) && (newMin > -0.5)
        targetMin = -1;
        targetMax = 1;
    else
        targetMin = newMin - 1;
        targetMax = newMax + 1;
    end
    
    if isequal(curLim, [0 1])
        ylim([targetMin, targetMax]);
    else
        ylim([min(curLim(1), targetMin), max(curLim(2), targetMax)]);
    end
end

% adjustZlim function (Expansion Logic)
function adjustZlim(var)
    curLim = zlim;
    newMin = min(var);
    newMax = max(var);
    
    if (newMax < 0.5) && (newMin > -0.5)
        targetMin = -1;
        targetMax = 1;
    else
        targetMin = newMin - 1;
        targetMax = newMax + 1;
    end
    
    if isequal(curLim, [0 1]) || isequal(curLim, [-1 1])
        zlim([targetMin, targetMax]);
    else
        % Note: If you use 'ZDir', 'reverse', min/max still work the same mathematically
        zlim([min(curLim(1), targetMin), max(curLim(2), targetMax)]);
    end
end

% add legend function
function addLegend(hPlot, col, opt)
    % col{1} is style, col{2} is the title addon, col{3} is the string name
    if length(col) > 2
        
        hPlot.DisplayName = col{3}; 
        
        allLines = findobj(gca, 'Type', 'line');
        
        % Only show the legend if there are 2 or more lines
        if length(allLines) >= 2
            lgd = legend('show');
            if opt == 1
                set(lgd, 'Position', [0.612499998092651 0.0188919610855095 0.342142860957555 0.038095238776434], ...
                             'Orientation', 'horizontal', ...
                             'FontSize', 7, ...
                             'Box', 'on');
            elseif opt == 2
                set(lgd, 'Location', 'northeast', ...
                             'FontSize', 8, ...
                             'Box', 'on');
            end
        else
            legend('hide'); % Ensures it stays off for the first run
        end
    end
end

end