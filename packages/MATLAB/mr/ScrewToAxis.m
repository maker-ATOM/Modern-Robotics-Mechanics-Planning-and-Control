function S = ScrewToAxis(q, s, h)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes q: a point lying on the screw axis,
%       s: a unit vector in the direction of the screw axis, 
%       h: the pitch of the screw axis.
% Returns the corresponding normalized screw axis.
% Example Input:
% 
% clear; clc;
 q = [0; 0; 2];
 s = [1; 0; 0];
 h = 1;
% S = ScrewToAxis(q, s, h)
% 
% Output:
% S =
%     0
%     0
%     1
%     0
%    -3
%     2

S = [s; cross(q, s) + h * s];
end