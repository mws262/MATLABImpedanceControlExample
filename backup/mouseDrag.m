%Testing mouse functions with graphing
function mouseDrag

init = [0,0,0,0]';
F = [0 0];
p.m = 1;
k = 10;
p.c = 15;

f = figure;
aH = axes('Xlim',[-5 5],'Ylim', [-5 5]);
h = plot(init(1),init(2),'.','MarkerSize',20); %Put a point on a plot and keep the figure handle.

set(f,'WindowButtonMotionFcn','','WindowButtonDownFcn',@ClickDown,'WindowButtonUpFcn',@ClickUp);

z1 = init;
told = 0; %start time;
tic
while (ishandle(1)) %As long as i don't close the figure, keep integrating.
    tnew = toc;
    dt = tnew - told;
    
    k1 = dynamics(tnew,z1,p);
    k2 = dynamics(tnew,z1+dt/2*k1,p);
    k3 = dynamics(tnew,z1+dt/2*k2,p);
    k4 = dynamics(tnew,z1+dt*k3,p);

    z2 = z1 + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    z1 = z2;

    set(h,'xData',z1(1));
    set(h,'yData',z1(2));
    axis([-5 5 -5 5]);
    told = tnew;
    pause(0.001);
    
end 
   

function zdot = dynamics(t,z,p)

xdot = z(3);
ydot = z(4);
xdotdot = F(1)/p.m-p.c*xdot;
ydotdot = F(2)/p.m-p.c*ydot;

zdot = [xdot ydot xdotdot ydotdot]';

end

function MousePos(varargin)

xcurrent = get(h,'xData');
ycurrent = get(h,'yData');
mousePos = get(aH,'CurrentPoint');

F = k*[(mousePos(1,1)-xcurrent), (mousePos(1,2)-ycurrent)];

end

function ClickDown(varargin)
    set(f,'WindowButtonMotionFcn',@MousePos);
end

function ClickUp(varargin)
    set(f,'WindowButtonMotionFcn','');
end

end