%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Two link robot arm with control to track a point that the user clicks.
%
% Files:
% MAIN - Execute this file; parameters here.
% Matthew Sheen, 2014
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all;

clear all;

rederive = false;
%%%%%%%% System Parameters %%%%%%%%

%Initial conditions:
% init = [0 0 0 0]';
p.init = [pi/4    0.0    pi/4  0.0]';

p.g = 9.81;
p.m1 = 1; %Mass of link 1.
p.m2 = 1; %Mass of link 2.
p.l1 = 1; %Total length of link 1.
p.l2 = 1; %Total length of link 2.
p.d1 = p.l1/2; %Center of mass distance along link 1 from the fixed joint.
p.d2 = p.l2/2; %Center of mass distance along link 2 from the fixed joint.
p.I1 = 1/12*p.m1*p.l1^2; %Moment of inertia of link 1 about COM
p.I2 = 1/12*p.m2*p.l2^2; %Moment of inertia of link 2 about COM

endZ = ForwardKin(p.l1,p.l2,p.init(1),p.init(3));
x0 = endZ(1); %End effector initial position in world frame.
y0 = endZ(2);
p.Fx = 0;
p.Fy = 0;

%%%%%%%% Control Parameters %%%%%%%%

%Controller Gains
p.Kp = 10;
p.Kd = 8;

%Tracked trajectory
%Single target:
p.xtarget = x0; %What points are we shooting for in WORLD SPACE?
p.ytarget = y0;

%%%%%%%% Other Parameters & Options %%%%%%%%
    %Animation playback speed (xRealtime):
    p.animationSpeed = 1.5; %Not currently used.

    p.absoluteAngles = false; %Only relevant in this version for the plotter to be happy

%%%%%%%% Run Derivers %%%%%%%%

if rederive
%If the gain matrix hasn't been found yet, then we assume derivation hasn't
%happened yet.
        deriverRelativeAngles;
        disp('Equations of motion and control parameters derived using relative angles.');
end

%%%%%%%% Integrate %%%%%%%%

Plotter(p) %Integration is done in real time using symplectic euler like we did in the CS animation class.


