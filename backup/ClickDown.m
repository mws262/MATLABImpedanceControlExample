function ClickDown(varargin)
global f Fx Fy
Fx = 0;
Fy = 0;

    set(f,'WindowButtonMotionFcn',@MousePos);
end