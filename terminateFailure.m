function [value,isterminal,direction] = terminateFailure(t,Z,p)
%Stop simulation when the thing starts swinging wildly fast (i.e. an
%obvious failure. This only really occurs if the linear controller is
%trying to work outside its basin of attraction.

value = [1 1];

if abs(Z(2))>1000  ||  abs(Z(4))>1000 %Stop when either velocity gets over 1000. They should be quite small during balancing.
    value(1) = 0;
end


% %NOTE: this has been added to detect when balance has been achieved (within
% %a tolerance -- state close to 0).
% if sum(abs(Z)) < 0.001
%     value(2) = 0;
% end

isterminal = [1 1]; %terminate the simulation.
direction = [-1 -1]; %only when it goes down to zero.
end