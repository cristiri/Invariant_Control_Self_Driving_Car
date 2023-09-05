%@Author: Cristian Tiriolo - cristian.tiriolo@concordia.ca
clear; close all;
l=0.5;


%Sampling Time
Ts=0.1;

%% TRAJECTORY PARAMETERS

%scaling factor of the trajectory
eta=1;
alpha=10;
k=0:Ts:2*pi*alpha*2;

xr=eta*sin(k/alpha); %trajectory position along x axis
yr=eta*sin(k/(2*alpha)); %trajetcory position along y axis


%Velocity trajectory
xdr=eta*cos(k/alpha)*(1/alpha); %trajectory velocity along x axis
ydr=eta*cos(k/(2*alpha))*(1/(2*alpha));%trajectory velocity along y axis

%Acceleration trajectory
xddr=-eta*sin(k/alpha)*(1/alpha)*(1/alpha); %trajetcory acceleration along x axis
yddr=-eta*sin(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));%trajetcory acceleration y axis


xdddr=-eta*cos(k/alpha)*(1/alpha)*(1/alpha)*(1/alpha);%trajetcory jerk along x axis
ydddr=-eta*cos(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));%trajetcory jerk y axis

%Driving velocity reference
v1r=sqrt(xdr.^2+ydr.^2); %Linear velocity of the trajectory


% Orientation reference
thetar=atan2(ydr./v1r,xdr./v1r); %Trajectory Orientation


%Adjusting Orientation ( a part of tetha(k) is out of phase
thetar_diff=diff(thetar);
for i=1:length(thetar_diff)
    if thetar_diff(i)<-6
        i1=i+1;
    elseif thetar_diff(i)>6
        i2=i;
    end
end
thetar(i1:i2)=thetar(i1:i2)+2*pi; %Ts=0.15


%Vehicle Angular velocity reference
thetadr=(yddr.*xdr-xddr.*ydr)./v1r.^2; %Angular velocity velocity of the trajectory

%Steering angle reference
phir=atan(l*(yddr.*xdr-xddr.*ydr)./v1r.^3);

%Steering velocity reference
v2r=l*v1r.*((ydddr.*xdr-xdddr.*ydr).*v1r.^2-3*(yddr.*xdr-xddr.*ydr).*(xdr.*xddr+ydr.*yddr))./(v1r.^6+l^2*(yddr.*xdr-xddr.*ydr).^2);


%Nonlinear model reference state
qr=[xr;yr;thetar;phir];


x0=0; y0=-0.035; theta0=thetar(1); phi0=phir(1);
q0=[x0;y0;theta0;phi0];  %Initial conditions


%Index for scanning the trajectory
K=1;
Kf=length(xr);


%% CONTROL PARAMETERS
%State Variables
n=4;
%Input Variables
m=2;


Delta=0.35;

%Feedback linearization matrix
T_FL=@(phi,theta,Delta,l)([cos(theta)-tan(phi)*(sin(theta)+Delta*sin(theta+phi)/l) -Delta*sin(theta+phi); sin(theta)+tan(phi)*(cos(theta)+Delta*cos(theta+phi)/l) Delta*cos(theta+phi)]);



%Constraints
v1max=0.5; %Driving velocity limit (m/s)
v2max=pi/4; %Steering velocity limit (rad/s)
T=[-1 0; 0 -1; 1 0; 0 1];
g=[v1max;v2max;v1max;v2max];


%Reference position feedback linearized system
zr=[xr+l*cos(thetar)+Delta*cos(phir+thetar);yr+l*sin(thetar)+Delta*sin(phir+thetar)];
zdr=zeros(2,length(xr));
for i=1:length(xr)
    
    vr_i=[v1r(i);v2r(i)];
    
    zdr(:,i)=T_FL(phir(i),thetar(i),Delta,l)*vr_i;
end



%% Variables to store and plot the results
tt=[];
useq=[];
qseq=q0;
wrwlseq=[];
zr_hat_seq=[];
ztilde_seq=[];



%% RESULTS COLLECTION
uk_seq=[];
vseq=[];
Hseq=[];


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
R=0.01*eye(2);
Q=1*eye(2);
N_LQ=zeros(2,2);
[K_LQ, P_LQ]=dlqr(Ad,Bd,Q,R,N_LQ);


%Worst case feasible input set
figure
phi_worst=pi/2;
r=sqrt((Delta^2*l^2*v2max^2)/(Delta^2-Delta^2*cos(phi_worst)^2+l^2));
Q_u=(1/(r^2))*eye(2);
ell_u=ellipsoid(inv(Q_u));
plot(ell_u)



%Worst-case feasible state set
f=K_LQ(1,1);
r_x=(Delta*l*v2max/f)*sqrt(1/(Delta^2+l^2));
Q_x_inv=r_x^2*eye(2);

figure
P_invariant=K_LQ'*Q_u*K_LQ;
inv_ell=ellipsoid(inv(P_invariant));
plot(inv_ell)
hold on
%Initial error
z_tilde_0=[x0+l*cos(theta0)+Delta*cos(phi0+theta0);y0+l*sin(theta0)+Delta*sin(phi0+theta0)]-[xr(1)+l*cos(thetar(1))+Delta*cos(phir(1)+thetar(1));yr(1)+l*sin(thetar(1))+Delta*sin(phir(1)+thetar(1))];


plot(z_tilde_0(1),z_tilde_0(2),'x--b');


F=-K_LQ;

%% Plot parameters
figure
grid
hold on
plot(xr,yr,'r')

plot(q0(1,1),q0(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',15)

%Trajectory Tracking error
eps=0;

%% TRACKING CONTROL SIMULATION
for i=0:Ts:10000000
    
    tt=[tt i];
    
    %Immagazino la sequenza di stato
    
    
    qk=qseq(:,end);
    
    x=qseq(1,end);
    y=qseq(2,end);
    theta=qseq(3,end);
    phi=qseq(4,end);
    
    T_FL=[cos(theta)-tan(phi)*(sin(theta)+Delta*sin(theta+phi)/l) -Delta*sin(theta+phi); sin(theta)+tan(phi)*(cos(theta)+Delta*cos(theta+phi)/l) Delta*cos(theta+phi)];
    
    
    
    
    q_tilde=qr(:,K)-qk;
    eps=eps+sqrt(q_tilde'*q_tilde);
    
    
    zr_k=[xr(K)+l*cos(thetar(K))+Delta*cos(phir(K)+thetar(K));yr(K)+l*sin(thetar(K))+Delta*sin(phir(K)+thetar(K))];
    
    z_k=[x+l*cos(theta)+Delta*cos(phi+theta);y+l*sin(theta)+Delta*sin(phi+theta)];
    
    
    
    T_FL_r=[cos(thetar(K))-tan(phir(K))*(sin(thetar(K))+Delta*sin(thetar(K)+phir(K))/l) -Delta*sin(thetar(K)+phir(K)); sin(thetar(K))+tan(phir(K))*(cos(thetar(K))+Delta*cos(thetar(K)+phir(K))/l) Delta*cos(thetar(K)+phir(K))];
    
    ur=T_FL_r*[v1r(K);v2r(K)];
    
    z_tilde=z_k-zr_k;
    
    
    ztilde_seq=[ztilde_seq z_tilde];
    
    uk=F*z_tilde;
    
    %output bias introduced by the FL
    d_FL=[l*cos(theta)+Delta*cos(phi+theta);l*sin(theta)+Delta*sin(phi+theta)];
    
    
    Hk=T*inv(T_FL);
    
    Hseq=[Hseq Hk];
    
    
    U_curr=Polyhedron(Hk,g); %time varying polyhedron
    
    
    
    uk_seq=[uk_seq uk];
    
    
    
    v=inv(T_FL)*uk;
    
    
    vseq=[vseq v];
    
    
    %Feeding the control inputs v2(k) and v2(k) into the car
    %model, using ode45 (Runge-Kutta solver)
    v1=v(1); v2=v(2);
    t=0:0.00001:Ts;
    [t,q]= ode45(@(t,q)car_model(t,q,v1,v2,l),t,qseq(:,end));
    
    
    %Updating the state sequence
    qseq=[qseq q(end,:)'];
    
    
    %Plotting robot trajectory
    plot(qseq(1,end),qseq(2,end),'b--x')
    %     plot(z_k(1),z_k(2),'b--x')
    pause(0.0001) %A little pause to obtain a live plot
    
    if K==Kf %If the trajetcory index reaches Kf the simulation is over
        break;
    end
    
    
    K=K+1; %Updating trajectory index
    
end

eps=eps/(Kf)


%%PLOTS



figure
%Right wheels angular velocites
subplot(4,1,1);
hold on;
p1=plot(tt,vseq(1,:));
p1.LineWidth=2;
p2=plot(tt,v1max*ones(1,length(tt)),'r--');
p2.LineWidth=2;
p2=plot(tt,-v1max*ones(1,length(tt)),'r--');
p2.LineWidth=2;

grid;
xlbl1=xlabel('Time[sec]','Interpreter','latex');
ylbl1=ylabel('$v_1(t)[m/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;

%Left wheels angular velocites
subplot(4,1,2);
hold on;
p3=plot(tt,vseq(2,:));
p3.LineWidth=2;
p4=plot(tt,v2max*ones(1,length(tt)),'r--');
p4.LineWidth=2;
p4=plot(tt,-v2max*ones(1,length(tt)),'r--');
p4.LineWidth=2;
grid;
xlbl2=xlabel('$Time[sec]$','Interpreter','latex');
ylbl2=ylabel('$v_2(t)[RAD/sec]$','Interpreter','latex');
ylbl2.FontSize=13;
xlbl2.FontSize=13;


%Orientation theta
subplot(4,1,3)
pl=plot(tt,qseq(3,1:end-1));
pl.LineWidth=2;
grid
% axis([0 tt(length(tt)) 0 6])
xlbl2=xlabel('$Time [sec]$','Interpreter','latex');
ylbl2=ylabel('$\theta(t)[RAD]$','Interpreter','latex');

%Steering angle phi
subplot(4,1,4)
pl=plot(tt,qseq(4,1:end-1));
pl.LineWidth=2;
grid
% axis([0 tt(length(tt)) 0 6])
xlbl2=xlabel('$Time [sec]$','Interpreter','latex');
ylbl2=ylabel('$\phi(t)[RAD]$','Interpreter','latex');

save('constraints_data','Hseq','g')
