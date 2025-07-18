function States = EOM_Long(t, X)
    %% Constant Terms
    m = 13.5; % Mass in kg
    g = 9.81; % Gravitational acceleration in m/s^2
    I_yy = 1.135; % Moment of inertia in y axis in kg*m^2
    S = 0.55; % Reference area in m^2
    rho = 1.225; % Air density in kg/m^3
    c = 0.19; % Mean aerodynamic chord in m
    T_max = 0.2*g * m; % Maximum thrust in N ######
    delta_t = 0.5;

    %% Aerodynamic Coefficients
    % Lift Coefficients
    C_L_0 = 0.28; % Lift coefficient at zero angle of attack
    C_L_alpha = 3.45; % Lift coefficient slope
    C_L_delta_e = -0.36; % Lift coefficient due to elevator deflection
    
    % Drag Coefficients
    C_D_0 = 0.03; % Drag coefficient at zero angle of attack
    C_D_alpha = 0.3; % Drag coefficient slope

    % Moment Coefficients
    C_M_0 = -0.024; % Moment coefficient at zero angle of attack
    C_M_alpha = -0.38; % Moment coefficient slope
    C_M_delta_e = -0.5; % Moment coefficient due to elevator deflection

    %% State Extraction
    u = X(1); % Forward velocity in m/s
    w = X(2); % Vertical velocity in m/s
    theta = X(3); % Pitch angle in radians
    q = X(4); % Pitch rate in rad/s
    x = X(5); % Horizontal position in m
    z = X(6); % Vertical position in m
    V = sqrt(u^2 + w^2); % Total velocity in m/s
    alpha = atan2(w, u); % Angle of attack in radians
    delta_e = -4.3791 * (pi/180); % Elevator deflection in radians

    %% Aerodynamic Forces and Moments
    C_L = C_L_0 + C_L_alpha * alpha + C_L_delta_e * delta_e; % Lift coefficient
    C_D = C_D_0 + C_D_alpha * alpha; % Drag coefficient
    C_M = C_M_0 + C_M_alpha * alpha + C_M_delta_e * delta_e; % Moment coefficient

    L = 0.5 * rho * V^2 * S * C_L; % Lift force in N
    D = 0.5 * rho * V^2 * S * C_D; % Drag force in N
    M = 0.5 * rho * V^2 * S * c * C_M; % Pitching moment in N*m
    T = T_max*delta_t; % Thrust force in N

    %% Forces in Body Frame
    X_b = L*sin(alpha) - D*cos(alpha) + T; % Force in x direction in N
    Z_B = -L*cos(alpha) - D*sin(alpha); % Force in z direction in N

    %% Equations of Motions
    d_u = X_b/m - g*sin(theta) - q*w; % Acceleration in x direction in m/s^2
    d_w = Z_B/m + g*cos(theta) + q*u; % Acceleration in z direction in m/s^2
    d_theta = q; % Angular acceleration in rad/s^2
    d_q = M/I_yy; % Angular acceleration in rad/s^2
    d_x = u* cos(theta) + w*sin(theta); % Change in x position in m
    d_z = u*sin(theta) - w*cos(theta); % Change in z position in m

    %% State Derivative
    States = [d_u; d_w; d_theta; d_q; d_x; d_z];
end
    