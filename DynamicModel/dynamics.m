
function states_dot = dynamics(states,controls,params)

    % States
    vx = states(1,:);
    vy = states(2,:);
    vz = states(3,:);
    roll = states(4,:);
    pitch = states(5,:);
    yaw = states(6,:);
    p = states(7,:);
    q = states(8,:);
    r = states(9,:);
    x = states(10,:);
    y = states(11,:);
    h = states(12,:);

    % Controls
    de = controls(1,:);
    da = controls(2,:);
    dt = controls(3,:);

    % Constant parameters
    m = params.m;
    g = params.g;
    rho = params.rho;
    S = params.S;
    Clalpha = params.Clalpha;
    K = params.K;
    Cd0 = params.Cd0;
    Cdp = params.Cdp;
    
    % Dynamic model
    V = sqrt(vx.^2+vy.^2+vz.^2);
    cr = cos(roll);
    sr = sin(roll);
    cp = cos(pitch);
    sp = sin(pitch);
    tp = tan(pitch);
    cy = cos(yaw);
    sy = sin(yaw);
    Cd = ;
    Cy = ;
    Cl = ;
    qd_times_S = 0.5.*rho.*V.^2.*S;
    D = qd_times_S.*Cd;
    Y = qd_times_S.*Cy;
    L = qd_times_S.*Cl;
    LL = ;
    MM = ;
    NN = ;
    Xa = ;
    Ya = ;
    Za = ;
    Xt = ;
    vx_dot = r.*vy-q.*vz-g.*sp+(Xa+Xt)./m;
    vy_dot = -r.*vx+p.*vz+g.*sr.*cp+Ya./m;
    vz_dot = q.*vx-p.*vy+g.*cr.*cp+Za./m;
    roll_dot = p+tp.*(q.*sr+r.*cr);
    pitch_dot = q.*cr-r.*sr;
    yaw_dot = (q.*sr+r.*cr)./cp;
    aux = Ix.*Iz-Ixz.^2;
    p_dot = (Ixz.*(Ix-Iy+Iz).*p.*q-(Iz.*(Iz-Iy)+Ixz.^2).*q.*r+Iz.*LL+Ixz.*NN)./aux;
    q_dot = ((Iz-Ix).*p.*r-Ixz.*(p.^2-r.^2)+MM)./Iy;
    r_dot = (((Ix-Iy).*Ix+Ixz.^2).*p.*q-Ixz.*(Ix-Iy+Iz).*q.*r+Ixz.*LL+Ix.*NN)./aux;
    x_dot = vx.*cp.*cy+vy.*(-cr.*sy+sr.*sp.*cy)+vz.*(sr.*sy+cr.*sp.*cy);
    y_dot = vx.*cp.*sy+vy.*(cr.*cy+sr.*sp.*sy)+vz.*(-sr.*cy+cr.*sp.*sy);
    h_dot = vx.*sp-vy.*sr.*cp-vz.*cr.*cp;

    states_dot = [vx_dot;vy_dot;vz_dot;roll_dot;pitch_dot;yaw_dot;p_dot;q_dot;r_dot;x_dot;y_dot;h_dot];

end

