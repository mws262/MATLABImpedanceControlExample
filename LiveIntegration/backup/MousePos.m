function MousePos(varargin)
global simArea xtarget ytarget xend yend tarControl Fx Fy

mousePos = get(simArea,'CurrentPoint');
if tarControl
xtarget = mousePos(1,1);
ytarget = mousePos(1,2);
else
Fx = 10*(mousePos(1,1)-xend);
Fy = 10*(mousePos(1,2)-yend);
end

end