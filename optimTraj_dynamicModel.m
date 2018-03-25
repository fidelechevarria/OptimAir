function [ propagatedState ] = optimTraj_dynamicModel( smooth_north, smooth_east, smooth_up )

    % Number of points
    N = numel(smooth_north);

    % Compute arclength
    arc = zeros(N-1,1);
    for i = 1:N-1
        arc(i) = sqrt((smooth_north(i+1)-smooth_north(i)).^2+...
                      (smooth_east(i+1)-smooth_east(i)).^2+...
                      (smooth_up(i+1)-smooth_up(i)).^2);
    end

    % Compute heading and pitch Euler angles
    pitch = zeros(1,N-1);
    heading = zeros(1,N-1);
    for i = 1:N-1
        horiz_dist = sqrt((smooth_north(i+1)-smooth_north(i))^2 +...
                          (smooth_east(i+1)-smooth_east(i))^2);
        pitch(i) = atan2(smooth_up(i+1)-smooth_up(i),horiz_dist);
        heading(i) = atan2(smooth_north(i+1)-smooth_north(i),...
                                  smooth_east(i+1)-smooth_east(i));
    end

    % Eliminate steps in Euler angles
    pitch_new = optimTraj_eliminateSteps(pitch);
    heading_new = optimTraj_eliminateSteps(heading);

    % Preallocate variables
    V_old = zeros(N,1);
    V_new = zeros(N,1);
    roll = zeros(N,1);
    L = zeros(N,1);
    D = zeros(N,1);
    alpha = zeros(N,1);
    alpha_old = zeros(N,1);
    lat_G = zeros(N,1);
    Clalpha = zeros(N,1);
    Cd0 = zeros(N,1);
    timestep = zeros(N,1);
    Qd = zeros(N,1);
    Cl = zeros(N,1);
    Cd = zeros(N,1);
    pitch_dot = zeros(N-1,1);
    heading_dot = zeros(N-1,1);

    % Constant parameters
    rho = 1.225;
    S = 9.1;
    K = 0.1779;
    m = 750;
    g = 9.81;

    % Look-up tables for Clalpha and Cd0
    Clalpha_xdata = [-3.14 -2.36 -1.57 -0.78 -0.6 -0.28 -0.25 -0.22 -0.2 0 0.2 0.22 0.25 0.28 0.6 0.78 1.57 2.36 3.14];
    Clalpha_ydata = convforce([0.05 1.0 0.0 1.0 0.7 -1.12 -1.34 -1.4 -1.34 0.0 1.34 1.4 1.34 1.12 0.7 1.0 0.0 -1.0 0.05],'lbf','N');
    Cd0_xdata = [-3.142 -1.57 -0.26 0 0.26 1.57 3.142];
    Cd0_ydata = convforce([0.06 1.5 0.036 0.028 0.036 1.5 0.06],'lbf','N');

    % Initial conditions
    V_old(1) = 92.95;
    alpha_old(1) = 0.1;
    T = 10000;

    % Dynamic_model
    % Assumptions: 1. No lateral force
    %              2. Thrust is always tangential to the trajectory
    for i = 1:N-2
        timestep(i) = arc(i)/V_old(i);
        pitch_dot(i) = (pitch_new(i+1)-pitch_new(i))/timestep(i);
        heading_dot(i) = (heading_new(i+1)-heading_new(i))/timestep(i);
        Qd(i) = 0.5*rho*V_old(i)^2;
        Clalpha(i) = fixpt_interp1(Clalpha_xdata,Clalpha_ydata,alpha_old(i),ufix(8),2^-8,sfix(16),2^-14,'Floor');
        Cd0(i) = fixpt_interp1(Cd0_xdata,Cd0_ydata,alpha_old(i),ufix(8),2^-8,sfix(16),2^-14,'Floor');
        roll(i) = atan2(V_old(i)*heading_dot(i)*cos(pitch(i)),g*cos(pitch(i))+V_old(i)*pitch_dot(i));
        L(i) = m*g*cos(pitch(i))*cos(roll(i))+m*V_old(i)*(pitch_dot(i)*cos(roll(i))+heading_dot(i)*cos(pitch(i))*sin(roll(i)));
        Cl(i) = L(i)/(Qd(i)*S);
        D(i) = Qd(i)*S*(Cd0(i)+K*Cl(i)^2);
        Cd(i) = D(i)/(Qd(i)*S);
        V_new(i) = V_old(i)+timestep(i)*((T-D(i)-m*g*sin(pitch(i)))/m);
        alpha(i) = Cl(i)/Clalpha(i); % Cl0 considered zero
        lat_G(i) = (V_new(i)*heading_dot(i))/9.8056;
        V_old(i+1) = V_new(i);
        alpha_old(i+1) = alpha(i);
    end
    
    propagatedState.total_time = sum(timestep);
    propagatedState.cum_time = cumsum(timestep);
    propagatedState.timestep = timestep;
    propagatedState.pitch_dot = pitch_dot;
    propagatedState.heading_dot = heading_dot;
    propagatedState.pitch_new = pitch_new;
    propagatedState.heading_new = heading_new;
    propagatedState.pitch = pitch;
    propagatedState.heading = heading;
    propagatedState.Qd = Qd;
    propagatedState.Clalpha = Clalpha;
    propagatedState.Cd0 = Cd0;
    propagatedState.roll = roll;
    propagatedState.L = L;
    propagatedState.Cl = Cl;
    propagatedState.D = D;
    propagatedState.Cd = Cd;
    propagatedState.V_new = V_new;
    propagatedState.alpha = alpha;
    propagatedState.lat_G = lat_G;
    propagatedState.V_old = V_old;
    propagatedState.alpha_old = alpha_old;
    propagatedState.arc = arc;

end

