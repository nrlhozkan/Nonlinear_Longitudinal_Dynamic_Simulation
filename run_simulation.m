clear all
clc
%% Initial Conditions 

V = 30;  % Initial velocity in m/s
theta_0 = 2.1471; % Initial pitch angle in degree
u_0 = V*cosd(theta_0);  
w_0 = V*sind(theta_0); 
theta_0_rad = deg2rad(theta_0); % Convert to radians
q_0 = 0; % Initial pitch rate in rad/s
x_0 = 0; % Initial x position in m
z_0 = 500; % Initial y position in m

%% Time Span
tspan = [0 250]; %Time span for the simulation in seconds

[t, States] = ode45(@EOM_Long, tspan, [u_0; w_0; theta_0_rad; q_0; x_0; z_0]);

%% Simulation Results
u = States(:, 1); % Forward velocity in m/s
w = States(:, 2); % Vertical velocity in m/s
theta = States(:, 3); % Pitch angle in radians
q = States(:, 4); % Pitch rate in rad/s
x = States(:, 5); % Horizontal position in m
z = States(:, 6); % Vertical position in m
alpha = atan2(w, u); % Angle of attack in radians

% Plotting the results
figure(1);
subplot(3, 1, 1); z
plot(t, z, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Vertical Position (m)');
title('Vertical Position');
grid on;

subplot(3, 1, 2); 
plot(t, u, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u (m/s)');
title('Forward Velocity');
grid on;

subplot(3, 1, 3);
plot(t, w, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('w (m/s)');
title('Vertical Velocity');
grid on;

saveas(gcf, 'Longitudinal_Dynamics_Velocities.png');

figure(2);
subplot(3, 1, 1); 
plot(t, rad2deg(alpha), 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Alpha (degrees)');
title('Angle of Attack'); 
grid on;

subplot(3, 1, 2); 
plot(t, rad2deg(theta), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Theta(degrees)');
title('Theta');
grid on;

subplot(3, 1, 3);
plot(t, rad2deg(q), 'c', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('q (degrees/s)');
title('Pitch Rate');
grid on;
saveas(gcf, 'Longitudinal_Dynamics_Angles.png');
