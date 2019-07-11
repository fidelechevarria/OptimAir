
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

vx = 0;
vy = 0;
vz = 0;
roll = 0;
pitch = 0;
yaw = 4.18879020478639;
p = 0;
q = 0;
r = 0;
x = 0;
y = 0;
h = 20;

da = 0;
de = 0;
dr = 0;
dt = 0;
%%
[V,alpha,beta] = dyn_alphaBeta(vx,vy,vz);
[cr,sr,cp,sp,tp,cy,sy] = dyn_eulTrigon(roll,pitch,yaw);
[coeffA,coeffB] = dyn_coeffsAux(b,c,V);
Cd = dyn_dragCoeff(Cd0,Cda2,Cdb,alpha,beta);
Cy = dyn_lateralForceCoeff(Cyb,Cydr,Cyda,Cyp,Cyr,beta,dr,da,coeffA,p,r);
Cl = dyn_liftCoeff(Cl0,Cla,alpha);
Cll = dyn_rollingMomentCoeff(Cllb,Cllda,Clldr,Cllp,Cllr,beta,da,dr,coeffA,p,r);
Cmm = dyn_pitchingMomentCoeff(Cmm0,Cmma,Cmmde,Cmmdr,Cmmda,Cmmq,alpha,de,dr,da,coeffB,q);
Cnn = dyn_yawingMomentCoeff(Cnnb,Cnnda,Cnndr,Cnnp,Cnnr,beta,da,dr,coeffA,p,r);
[D,Y,L,LL,MM,NN] = dyn_forcesMomentsWind(rho,V,S,b,c,Cd,Cy,Cl,Cll,Cmm,Cnn);
[Xa,Ya,Za,Xt] = dyn_forcesBody(alpha,beta,D,Y,L,Tmax,dt);
[vx_dot,vy_dot,vz_dot] = dyn_velBodyDot(vx,vy,vz,p,q,r,Xa,Xt,Ya,Za,g,m,sp,cp,sr,cr);
[roll_dot,pitch_dot,yaw_dot] = dyn_eulDot(p,q,r,tp,sr,cr,cp);
[p_dot,q_dot,r_dot] = dyn_angularRatesDot(Ix,Iy,Iz,Ixz,p,q,r,LL,MM,NN);
[x_dot,y_dot,h_dot] = dyn_posDot(vx,vy,vz,sr,cr,sp,cp,sy,cy);

dtime = 0.05;
vx = vx + vx_dot*dtime;
vy = vy + vy_dot*dtime;
vz = vz + vz_dot*dtime;
roll = roll + roll_dot*dtime;
pitch = pitch + pitch_dot*dtime;
yaw = yaw + yaw_dot*dtime;
p = p + p_dot*dtime;
q = q + q_dot*dtime;
r = r + r_dot*dtime;
x = x + x_dot*dtime;
y = y + y_dot*dtime;
h = h + h_dot*dtime;
