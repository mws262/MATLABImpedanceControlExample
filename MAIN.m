%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Acrobot simulation with swing-up and balance control. The acrobot is a
% double pendulum with only a motor at the elbow (i.e. the second joint.
%
% Swing up control attempts to drive energy to the equivalent potential
% energy when the pendulum is upright. Balance control is LQR about the
% unstable upright equilibrium.
%
% Files:
% MAIN - Execute this file; parameters here.
% 
%
% 
% Matthew Sheen, 2014
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all;

clear all;

rederive = false;
%%%%%%%% System Parameters %%%%%%%%

%Total Simulation Time
simTime = 12;

%Initial conditions:
% init = [0 0 0 0]';
init = [pi/4    0.0    pi/4  0.0]';

p.g = 9.81;
p.m1 = 1; %Mass of link 1.
p.m2 = 1; %Mass of link 2.
p.l1 = 1; %Total length of link 1.
p.l2 = 1; %Total length of link 2.
p.d1 = p.l1/2; %Center of mass distance along link 1 from the fixed joint.
p.d2 = p.l2/2; %Center of mass distance along link 2 from the fixed joint.
p.I1 = 1/12*p.m1*p.l1^2; %Moment of inertia of link 1 about COM
p.I2 = 1/12*p.m2*p.l2^2; %Moment of inertia of link 2 about COM

p.T1 = 1; %Motor torques default to zero. Not really relevant unless you run this without control (or maybe disturbances at first joint?)
p.T2 = 1; %Joint torque 2 only affects the system when controls aren't used.

endZ = ForwardKin(p.l1,p.l2,init(1),init(3));
x0 = endZ(1); %End effector initial position in world frame.
y0 = endZ(2);

%%%%%%%% Control Parameters %%%%%%%%

%Controller Gains
p.Kp = 1.5;
p.Kd = 1.2;

%Tracked trajectory
%Single target:
xtarget = 1.5; %What points are we shooting for in WORLD SPACE?
ytarget = -1;

p.tt = linspace(0,simTime,100); %This is unnecessary for the single target case, but used so I can also put in trajectories in time.
p.xt = ones(1,length(p.tt))*xtarget;
p.yt = ones(1,length(p.tt))*ytarget;

p.xtdot = zeros(1,length(p.tt));
p.ytdot = zeros(1,length(p.tt));

%Actual trajectory
xtarget = 1.5; %What points are we shooting for in WORLD SPACE?
ytarget = -1;

p.tt = linspace(0,simTime,100); %This is unnecessary for the single target case, but used so I can also put in trajectories in time.
p.xt = linspace(-1.5,1.5,length(p.tt));
p.yt = ones(1,length(p.tt))*ytarget;

p.xtdot = zeros(1,length(p.tt));
p.ytdot = zeros(1,length(p.tt));

p.Fx = zeros(1,length(p.tt));
p.Fy = zeros(1,length(p.tt));

% p.sat = 50; %Actuator saturation threshold (Nm).

%%%%%%%% Other Parameters & Options %%%%%%%%

    %Animation playback speed (xRealtime):
    p.animationSpeed = 1.5;
    
    %Debug without control on?
    controlOn = false;

    %Integration tolerances
    tol = 1e-6;

    p.absoluteAngles = false; %Only relevant in this version for the plotter to be happy
%%%%%%%% Run Derivers %%%%%%%%

if rederive
%If the gain matrix hasn't been found yet, then we assume derivation hasn't
%happened yet.
        deriverRelativeAngles;
        disp('Equations of motion and control parameters derived using relative angles.');
end

%%%%%%%% Integrate %%%%%%%%
    
    options = odeset('AbsTol', tol,'RelTol',tol,'Events',@terminateFailure); %The event simply looks to see if the velocity of either link gets above 1000. In this case, it deems the system totally unstable.
    [tarray, zarray,TE,YE,IE] = ode15s(@FullDyn, [0 simTime], init, options, p);

%We have to post-calculate the torques because ode45 is stupid.
th1 = zarray(:,1);
thdot1 = zarray(:,2);
th2 = zarray(:,3);
thdot2 = zarray(:,4);
    
xTar = interp1(p.tt,p.xt,tarray);
yTar = interp1(p.tt,p.yt,tarray);

xdotTar = interp1(p.tt,p.xtdot,tarray);
ydotTar = interp1(p.tt,p.ytdot,tarray);

T = ImpedenceControl(p.Kd,p.Kp,p.l1,p.l2,th1,th2,thdot1,thdot2,xdotTar,xTar,ydotTar,yTar);

Tarray1 = T(1) + GravityCompT1(0,0,p.d1,p.d2,p.g,p.l1,p.l2,p.m1,p.m2,th1,th2,thdot1,thdot2);
Tarray2 = T(2) + GravityCompT2(0,0,p.d2,p.g,p.l1,p.l2,p.m2,th1,th2,thdot1);
    
Tarray = Tarray1;
    
% Tarray = zeros(size(zarray,1),1);

%Torques are post-calculated due to the difficulty of pulling numbers out
%of ODE45. Therefore, we also have to post-cast the values within the
%actuator limits.
% Tarray(Tarray>p.sat) = p.sat;
% Tarray(Tarray<-p.sat) = -p.sat;
%Call the animation script
Plotter


