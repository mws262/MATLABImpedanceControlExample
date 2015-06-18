%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the acrobot after the MAIN script has been run.
%
%   Matthew Sheen, 2014
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all

%Playback speed:
playback = p.animationSpeed;

%Repeat button
h = uicontrol('Style', 'pushbutton', 'String', 'Repeat',...
 'Position', [10 20 75 50], 'Callback', 'Plotter');

time = tarray;
endtime = tarray(end);

%Calculate the energy throughout the trajectory.
energy = TotEnergy(p.I1,p.I2,p.d1,p.d2,p.g,p.l1,p.m1,p.m2,zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4));

torque = zeros(size(tarray)); %PLACEHOLDER CRAP
torque(end) = 1;

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%

sub1 = subplot(3,4,[1 2 5 6 9 10]); %4x2 grid with animation taking the top panel of 3x2
axis equal
hold on

%Create pendulum link1 object:
width1 = p.l1*0.05;
xdat1 = 0.5*width1*[-1 1 1 -1];
% ydat1 = p.l1*[0 0 -1 -1]; %If I decide to parametrize with zero being
% down.
ydat1 = p.l1*[0 0 1 1];
link1 = patch(xdat1,ydat1, [0 0 0 0],'r');

%Create pendulum link2 object:
width2 = p.l2*0.05;
xdat2 = 0.5*width2*[-1 1 1 -1];
% ydat2 = p.l2*[0 0 -1 -1];
ydat2 = p.l2*[0 0 1 1];
link2 = patch(xdat2,ydat2, [0 0 0 0],'b');
axis([-3.5 3.5 -3.6 3.6]);

%Dots for the hinges:
h1 = plot(0,0,'.k','MarkerSize',40); %First link anchor
h2 = plot(0,0,'.k','MarkerSize',40); %link1 -> link2 hinge

%Timer label:
timer = text(-3.2,-3.2,'0.00','FontSize',28);

%Tracked Trajectory:
trajectory = plot(0,0,':g','LineWidth',1);
%Beginning pt:
plot(p.xt(1),p.yt(1),'xg','MarkerSize',30);
%EndPt
plot(p.xt(end),p.yt(end),'xr','MarkerSize',30);

%Add force vector at end effector
% ForceVector = annotation('arrow',[0 0],[1 1],'Color','k','Parent',sub1);

hold off

%%%%%%%% 2nd Subplot -- the system energy %%%%%%%
subplot(3,4,[3 4])
hold on

%Find the desired energy when the system is balanced.
desEnergy = TotEnergy(p.I1,p.I2,p.d1,p.d2,p.g,p.l1,p.m1,p.m2,0,0,0,0);

energyPlot = plot(0,0,'b'); %Energy plot over time.
% plot([0,endtime],[desEnergy,desEnergy],'r'); %The line showing the target energy

axis([0,endtime,min(energy)-1,desEnergy*1.2]); %size to fit energy bounds and timescale

xlabel('Time (s)','FontSize',16)
ylabel('Energy (J)','FontSize',16)

hold off

%%%%%%%% 3rd Subplot -- the control torque %%%%%%%
subplot(3,4,[7 8])
hold on
torquePlot1 = plot(0,0,'r');
torquePlot2 = plot(0,0,'b');
xlim([0,endtime])
ylim([-8 8])
xlabel('Time (s)','FontSize',16)
ylabel('Torque (Nm)','FontSize',16)
legend('Motor 1 torque','Motor 2 torque');
axis([0,endtime,1.1*min(min(Tarray1),min(Tarray2)),1.1*max(max(Tarray1),max(Tarray2))]); %size to fit whatever output given
hold off

subplot(3,4,[11 12])
workPlot = plot(0,0);
%axis([0,endtime,0,trapz(time,max(power,0))*1.1]); %size to fit whatever output given
xlabel('Time (s)','FontSize',16)
ylabel('Placeholder','FontSize',16)
workdat = [];
timedat = [];

%Make the whole window big for handy viewing:
set(gcf, 'units', 'inches', 'position', [5 5 18 9])

%Animation plot loop:
tic %Start the clock
while toc<endtime/playback
    
    %If i close the figure, this prevents the error message (anal
    %programming)
    if ishandle(1) == 0
        break;
    end
    
    tstar = playback*toc; %Get the time (used during this entire iteration)
    %On screen timer.
    set(timer,'string',strcat(num2str(tstar,3),'s'))
    zstar = interp1(time,zarray,tstar); %Interpolate data at this instant in time.
    
    %Rotation matrices to manipulate the vertices of the patch objects
    %using theta1 and theta2 from the output state vector.
    rot1 = [cos(zstar(1)), -sin(zstar(1)); sin(zstar(1)),cos(zstar(1))]*[xdat1;ydat1];
    set(link1,'xData',rot1(1,:))
    set(link1,'yData',rot1(2,:))
    
    if p.absoluteAngles %If the second link angle is relatived to fixed frame, different transformation is needed than if relative angle is used.
        rot2 = [cos(zstar(3)), -sin(zstar(3)); sin(zstar(3)),cos(zstar(3))]*[xdat2;ydat2];
    else
        rot2 = [cos(zstar(3)+zstar(1)), -sin(zstar(3)+zstar(1)); sin(zstar(3)+zstar(1)),cos(zstar(3)+zstar(1))]*[xdat2;ydat2];
    end
    
    set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) %We want to add the midpoint of the far edge of the first link to all points in link 2.
    set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)
    
    %Change the hinge dot location
    set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
    set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)
    
    %Make the tracked trajectory also show up in real time.
    plotIndTraj = p.tt<tstar;
    set(trajectory,'xData',p.xt(plotIndTraj));
    set(trajectory,'yData',p.yt(plotIndTraj));
    
    %Force vector:
%     xend = get(link2,'xData');
%     yend = get(link2,'yData');
%     set(ForceVector,'X',[.5 xend(3)]);
%     set(ForceVector,'Y',[.5 yend(3)]);    
      
    %Make the energy profile also plot out over time simultaneously.
    plotInd = time<tstar;
    set(energyPlot,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(energyPlot,'yData',energy(plotInd))
  
    %Make the power profile also plot out over time simultaneously.
    set(torquePlot1,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(torquePlot1,'yData',Tarray1(plotInd))
    
    set(torquePlot2,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(torquePlot2,'yData',Tarray2(plotInd))
%     
%     timedat = [timedat,tstar];
%     workdat = [workdat,trapz(time(plotInd),power(plotInd),1)];
%     
    %Cumulative work meter:
    %set(energy,'string',strcat(num2str(workdat(end),3),'J'))
    %Make the power profile also plot out over time simultaneously.
    set(workPlot,'xData',timedat) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(workPlot,'yData',workdat)
 
    %Put a little delay in to make iterations not freak out.
    pause(0.001)
end

