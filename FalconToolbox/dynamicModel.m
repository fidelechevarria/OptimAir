function [states_dot] = dynamicModel(states, controls)
% model interface created by falcon.m

% Extract states
vx = states(1,:);
vy = states(2,:);
vz = states(3,:);
roll = states(4,:);
pitch = states(5,:);
yaw = states(6,:);
p = states(7,:);
q = states(8,:);
r = states(9,:);
% x = states(10,:); % Not used in dynamics
% y = states(11,:); % Not used in dynamics
% h = states(12,:); % Not used in dynamics

% Extract controls
da = controls(1,:);
de = controls(2,:);
dr = controls(3,:);
dt = controls(4,:);

% Constants
m = 750; % kg
g = 9.8056; % m/s^2
rho = 1.225; % kg/m^3
S = 9.84; % m^2
Tmax = 9000; % N
b = 7.87; % m
c = S./b; % m
Cd0 = 0.15;
Cda2 = 0.1;
Cdb = 0.15;
Cyb = -0.4;
Cyda = 0;
Cydr = 0.19;
Cyp = 0;
Cyr = 0.4;
Cl0 = 0.1205;
Cla = 5.7;
Cllb = -0.0002;
Cllda = 0.33;
Clldr = 0.021;
Cllp = -0.79;
Cllr = 0.075;
Cmm0 = 0;
Cmma = -1.23;
Cmmda = 0;
Cmmde = -1.1;
Cmmdr = 0;
Cmmq = -7.34;
Cnnb = 0.21;
Cnnda = -0.014;
Cnndr = -0.11;
Cnnp = -0.024;
Cnnr = -0.265;
Ix = 3531.9; % kg*m^2
Iy = 2196.4; % kg*m^2
Iz = 4887.7; % kg*m^2
Ixz = 0; % kg*m^3

% Dynamic model
V = sqrt(vx.^2+vy.^2+vz.^2);
alpha = atan2(vz,vx);
beta = asin(vy./V);
cr = cos(roll);
sr = sin(roll);
cp = cos(pitch);
sp = sin(pitch);
tp = tan(pitch);
cy = cos(yaw);
sy = sin(yaw);
factor_1 = (b./(2.*V));
factor_2 = (c./(2.*V));
Cd = Cd0+Cda2.*alpha+Cdb.*abs(beta);
Cy = Cyb.*beta+Cydr.*dr+Cyda.*da+factor_1.*(Cyp.*p+Cyr.*r);
Cl = Cl0+Cla.*alpha;
Cll = Cllb.*beta+Cllda.*da+Clldr.*dr+factor_1.*(Cllp.*p+Cllr.*r);
Cmm = Cmm0+Cmma.*alpha+Cmmde.*de+Cmmdr.*dr+Cmmda.*abs(da)+factor_2.*(Cmmq.*q);
Cnn = Cnnb.*beta+Cnnda.*da+Cnndr.*dr+factor_1.*(Cnnp.*p+Cnnr.*r);
qd_times_S = 0.5.*rho.*V.^2.*S;
qd_times_S_times_b = qd_times_S.*b;
qd_times_S_times_c = qd_times_S.*c;
D = qd_times_S.*Cd;
Y = qd_times_S.*Cy;
L = qd_times_S.*Cl;
LL = qd_times_S_times_b.*Cll;
MM = qd_times_S_times_c.*Cmm;
NN = qd_times_S_times_b.*Cnn;
ca = cos(alpha);
sa = sin(alpha);
cb = cos(beta);
sb = sin(beta);
Xa = -ca.*cb.*D-ca.*sb.*Y+sa.*L;
Ya = -sb.*D+cb.*Y;
Za = -sa.*cb.*D-sa.*sb.*Y-ca.*L;
Xt = Tmax.*dt;
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