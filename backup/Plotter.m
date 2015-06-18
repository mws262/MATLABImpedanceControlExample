%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the acrobot after the MAIN script has been run.
%
%   Matthew Sheen, 2014
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Plotter(p)
close all
global f simArea xtarget ytarget tarControl Fx Fy xend yend

%Playback speed:
% playback = p.animationSpeed;
endtime = 10;
tarControl = true; %Whether we're applying a disturbance or changing the target point.

%Name the whole window and define the mouse callback function
f = figure;
set(f,'WindowButtonMotionFcn','','WindowButtonDownFcn',@ClickDown,'WindowButtonUpFcn',@ClickUp,'KeyPressFc',@KeyPress);

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%
simArea = subplot(3,4,[1 2 5 6 9 10]); %4x2 grid with animation taking the top panel of 3x2
axis equal
hold on

%Create pendulum link1 object:
width1 = p.l1*0.05;
xdat1 = 0.5*width1*[-1 1 1 -1];
ydat1 = p.l1*[0 0 1 1];
link1 = patch(xdat1,ydat1, [0 0 0 0],'r');

%Create pendulum link2 object:
width2 = p.l2*0.05;
xdat2 = 0.5*width2*[-1 1 1 -1];
ydat2 = p.l2*[0 0 1 1];
link2 = patch(xdat2,ydat2, [0 0 0 0],'b');
axis([-3.5 3.5 -3.6 3.6]);

%Dots for the hinges:
h1 = plot(0,0,'.k','MarkerSize',40); %First link anchor
h2 = plot(0,0,'.k','MarkerSize',40); %link1 -> link2 hinge

%Timer label:
timer = text(-3.2,-3.2,'0.00','FontSize',28);

%Torque meters on screen
tmeter1 = text(0.6,-3.2,'0.00','FontSize',22,'Color', 'r');
tmeter2 = text(2.2,-3.2,'0.00','FontSize',22,'Color', 'b');

%Target Pt.
targetPt = plot(p.xtarget,p.ytarget,'xr','MarkerSize',30);

hold off

%%%%%%%% 2nd Subplot -- the system energy %%%%%%%
%NOT USED
subplot(3,4,[3 4])
hold on
energyPlot = plot(0,0,'b'); %Energy plot over time.
xlabel('Time (s)','FontSize',16)
ylabel('Energy (J)','FontSize',16)
hold off

%%%%%%%% 3rd Subplot -- the control torque %%%%%%%
%NOT USED
subplot(3,4,[7 8])
hold on
torquePlot1 = plot(0,0,'r');
torquePlot2 = plot(0,0,'b');
xlim([0,endtime])
ylim([-8 8])
xlabel('Time (s)','FontSize',16)
ylabel('Torque (Nm)','FontSize',16)
legend('Motor 1 torque','Motor 2 torque');
hold off

%%%%%%%% 4th Subplot -- a placeholder %%%%%%%
subplot(3,4,[11 12])
workPlot = plot(0,0);
xlabel('Time (s)','FontSize',16)
ylabel('Placeholder','FontSize',16)
workdat = [];
timedat = [];

%Make the whole window big for handy viewing:
set(gcf, 'units', 'inches', 'position', [5 5 18 9])

%Animation plot loop -- Includes symplectic integration now.
TimeArray = []; %Use later to implement torque over time plot
T1Array = [];
T2Array = [];
z1 = p.init;
told = 0;

tic %Start the clock
while (ishandle(1))
    %%%% INTEGRATION %%%%
    tnew = toc;
    dt = tnew - told;
    
    %Old velocity and position
    xold = [z1(1),z1(3)];
    vold = [z1(2),z1(4)];
   
    %Call RHS given old state
    [zdot1, T1, T2] = FullDyn(tnew,z1,p);
    vinter1 = [zdot1(1),zdot1(3)];
    ainter = [zdot1(2),zdot1(4)];
    
    vinter2 = vold + ainter*dt; %Update velocity based on old RHS call
    
    %Update position.
    xnew = xold + vinter2*dt;
    vnew = (xnew-xold)/dt;
    
    z2 = [xnew(1) vnew(1) xnew(2) vnew(2)];

    z1 = z2;
    told = tnew;
    %%%%%%%%%%%%%%%%%%%%
    
    %If there are new mouse click locations, then set those as the new
    %target.
    if ~isempty(xtarget)
    p.xtarget = xtarget;
    end
    
    if ~isempty(ytarget)
    p.ytarget = ytarget;
    end
    set(targetPt,'xData',p.xtarget); %Change the target point graphically.
    set(targetPt,'yData',p.ytarget);
    
    %When you hit a key, it changes to force mode, where the mouse will
    %pull things.
    ra_e = ForwardKin(p.l1,p.l2,z1(1),z1(3));
    xend = ra_e(1);
    yend = ra_e(2);
    
    if ~isempty(Fx)
    p.Fx = Fx;
    end
    if ~isempty(Fy)
    p.Fy = Fy;
    end
    
    tstar = told; %Get the time (used during this entire iteration)
    
    %On screen timer.
    set(timer,'string',strcat(num2str(tstar,3),'s'))
    zstar = z1;%interp1(time,zarray,tstar); %Interpolate data at this instant in time.
    
    %Rotation matrices to manipulate the vertices of the patch objects
    %using theta1 and theta2 from the output state vector.
    rot1 = [cos(zstar(1)), -sin(zstar(1)); sin(zstar(1)),cos(zstar(1))]*[xdat1;ydat1];
    set(link1,'xData',rot1(1,:))
    set(link1,'yData',rot1(2,:))
    
    rot2 = [cos(zstar(3)+zstar(1)), -sin(zstar(3)+zstar(1)); sin(zstar(3)+zstar(1)),cos(zstar(3)+zstar(1))]*[xdat2;ydat2];
    
    set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) %We want to add the midpoint of the far edge of the first link to all points in link 2.
    set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)
    
    %Change the hinge dot location
    set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
    set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)
    
    %Show torques on screen (text only atm) update for time series later.
    set(tmeter1,'string',strcat(num2str(T1,2),' Nm'))
    set(tmeter2,'string',strcat(num2str(T2,2),' Nm'))
    
    %Subplots below not used.
    %Make the tracked trajectory also show up in real time.
%     plotIndTraj = p.tt<tstar;
%     set(trajectory,'xData',p.xt(plotIndTraj));
%     set(trajectory,'yData',p.yt(plotIndTraj));
    
    %Force vector:
%     xend = get(link2,'xData');
%     yend = get(link2,'yData');
%     set(ForceVector,'X',[.5 xend(3)]);
%     set(ForceVector,'Y',[.5 yend(3)]);    
      
    %Make the energy profile also plot out over time simultaneously.
%     plotInd = time<tstar;
%     set(energyPlot,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
%     set(energyPlot,'yData',energy(plotInd))
%   
%     %Make the power profile also plot out over time simultaneously.
%     set(torquePlot1,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
%     set(torquePlot1,'yData',Tarray1(plotInd))
%     
%     set(torquePlot2,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
%     set(torquePlot2,'yData',Tarray2(plotInd))
% %     
%     timedat = [timedat,tstar];
%     workdat = [workdat,trapz(time(plotInd),power(plotInd),1)];
%     
 
    %Put a little delay in to make iterations not freak out.
    pause(0.00001)
end
