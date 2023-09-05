%@Author: Cristian Tiriolo - cristian.tiriolo@concordia.ca
clear; close all;
l=0.256;

%Sampling Time
Ts=0.1;

%% TRAJECTORY PARAMETERS

% Elliptic trajectory
alpha=10;
k=0:Ts:2*alpha*pi+Ts;
vxbar=2; vybar=2;

xr=vxbar*cos(k/alpha);
yr=vybar*sin(k/alpha);

xdr=-1/alpha*vxbar*sin(k/alpha);
ydr= 1/alpha*vybar*cos(k/alpha);

xddr=-1/alpha^2*vxbar*cos(k/alpha);
yddr=-1/alpha^2*vybar*sin(k/alpha);

xdddr=1/alpha^3*vxbar*sin(k/alpha);
ydddr=-1/alpha^3*vybar*cos(k/alpha);



%Driving velocity reference
v1r=sqrt(xdr.^2+ydr.^2); %Linear velocity of the trajectory


% Orientation reference
thetar=atan2(ydr./v1r,xdr./v1r); %Trajectory Orientation


%Adjusting Orientation ( a part of tetha(k) is out of phase
thetar_diff=diff(thetar);



thetar(159:end)=thetar(159:end)+2*pi;

%Vehicle Angular velocity reference
thetadr=(yddr.*xdr-xddr.*ydr)./v1r.^2; %Angular velocity velocity of the trajectory

%Steering angle reference
phir=atan(l*(yddr.*xdr-xddr.*ydr)./v1r.^3);

%Steering velocity reference
v2r=l*v1r.*((ydddr.*xdr-xdddr.*ydr).*v1r.^2-3*(yddr.*xdr-xddr.*ydr).*(xdr.*xddr+ydr.*yddr))./(v1r.^6+l^2*(yddr.*xdr-xddr.*ydr).^2);


%Nonlinear model reference state
qr=[xr;yr;thetar;phir];


x0=xr(1); y0=yr(1); theta0=thetar(1); phi0=0;


Delta=0.35;



%Linearized system matrices
A=zeros(2,2);
B=eye(2);

%Discrete-time system
Ad = eye(2)+A*Ts;
Bd = B*Ts;
Cd = eye(2);
Dd = [0; 0];

model = LTISystem('A',Ad,'B',Bd,'Ts',Ts);



%control parameters
R=0.00000000000001*eye(2);
Q=1*eye(2);
N_LQ=zeros(2,2);
[K_LQ P_LQ]=dlqr(Ad,Bd,Q,R,N_LQ);


