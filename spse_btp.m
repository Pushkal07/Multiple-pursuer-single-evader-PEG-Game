clc;
clear;
close all;

% Simulation parameters
dt = 0.1;  % Time step
T = 5;     % Total time for simulation

% Initial positions (x, y, theta)
pursuer_pos = [-1; -1; 0];   % [x; y; theta] for pursuer
evader_pos = [0; 0; 0];  % [x; y; theta] for evader

% Velocities
v_pursuer = 15;  % Velocity of pursuer
v_evader = 19;   % Velocity of evader

% Angular velocities (fixed for simplicity)
omega_pursuer = 0.2;  % Angular velocity of pursuer
omega_evader = 0.2;   % Angular velocity of evader

% Storage for trajectories
pursuer_traj = [];
evader_traj = [];

% Simulation loop
for t = 0:dt:T
    % Store current positions
    pursuer_traj = [pursuer_traj; pursuer_pos(1:2)'];
    evader_traj = [evader_traj; evader_pos(1:2)'];
    
    % Calculate distance between pursuer and evader
    distance = norm(pursuer_pos(1:2) - evader_pos(1:2));
    
    % Break the loop if the pursuer catches the evader
    if distance < 0.1
        disp('Pursuer has caught the evader!');
        break;
    end
    
    % Direction to evader
    direction = atan2(evader_pos(2) - pursuer_pos(2), evader_pos(1) - pursuer_pos(1));
    
    % Update the orientation of the pursuer to follow the evader
    pursuer_pos(3) = direction;
    
    % Update positions based on velocities and orientations
    pursuer_pos(1) = pursuer_pos(1) + v_pursuer * cos(pursuer_pos(3)) * dt;
    pursuer_pos(2) = pursuer_pos(2) + v_pursuer * sin(pursuer_pos(3)) * dt;
    
    % Evader moves in a fixed direction with some noise to make it unpredictable
    evader_pos(3) = evader_pos(3) + omega_evader * dt;
    evader_pos(1) = evader_pos(1) + v_evader * cos(evader_pos(3)) * dt;
    evader_pos(2) = evader_pos(2) + v_evader * sin(evader_pos(3)) * dt;
    
    % Plot positions and trajectories in real-time
    plot(pursuer_traj(:,1), pursuer_traj(:,2), 'b-', 'LineWidth', 2); % Pursuer trajectory
    hold on;
    plot(evader_traj(:,1), evader_traj(:,2), 'r-', 'LineWidth', 2);   % Evader trajectory
    plot(pursuer_pos(1), pursuer_pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Current position of pursuer
    plot(evader_pos(1), evader_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');   % Current position of evader
    xlim([-15 15]);
    ylim([-15 15]);
    legend('Pursuer Trajectory', 'Evader Trajectory', 'Pursuer', 'Evader');
    title(['Time: ' num2str(t) 's']);
    grid on;
    axis equal;
    pause(0.01);
    hold off;
end

% If the loop ends without capturing the evader
if distance >= 0.1
    disp('Evader escaped!');
end

% Final plot of the trajectories
figure;
plot(pursuer_traj(:,1), pursuer_traj(:,2), 'b-', 'LineWidth', 2); % Pursuer trajectory
hold on;
plot(evader_traj(:,1), evader_traj(:,2), 'r-', 'LineWidth', 2); % Evader trajectory
plot(pursuer_traj(1,1), pursuer_traj(1,2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Initial position of pursuer
plot(evader_traj(1,1), evader_traj(1,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Initial position of evader
legend('Pursuer Trajectory', 'Evader Trajectory', 'Pursuer Start', 'Evader Start');
xlabel('X Position');
ylabel('Y Position');
title('Pursuer and Evader Trajectories');
grid on;
axis equal;