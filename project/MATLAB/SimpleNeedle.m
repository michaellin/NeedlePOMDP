function [w,theta] = SimpleNeedle(v,F,dir,P_needle)
% solve a cantilever beam bending problem
% configuration:
%      ||
% wall ||========== beam
%      ||
%
% Input:
% v: insertion step size [mm] --> velocity
% F: bevel tip load [N]
% dir: bevel tip orientation, +1 for counterclockwise, -1 for clockwise
% P_needle: needle properties: E, I, etc.
%
% Output
% w: tip deflection [mm]
% theta: tip angle [rad]

F = dir*F;

E = P_needle.E;
I = P_needle.I;

w = F*(v/1000)^3/(3*E*I); % [m]
w = w*1000; % [mm]

theta = F*(v/1000)^2/(2*E*I);

end