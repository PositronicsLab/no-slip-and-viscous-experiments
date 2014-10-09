%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% 3d_edge_edge_distance.m
%
% Calculates the distance between two edges, along with the closest points
%  Source: http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm

function [psi, ep1, ep2] = edge_edge_distance_3d(B1, B2, E1, E2)

% E1 and E2 are edge indicies
% psi is a positive scalar distance 
% ep1 and ep2 are the endpoints of the shortest segment between the two edges

u = B1.verts( B1.edges(E1).verts(2) ).world_coords - B1.verts( B1.edges(E1).verts(1) ).world_coords;
v = B2.verts( B2.edges(E2).verts(2) ).world_coords - B2.verts( B2.edges(E2).verts(1) ).world_coords;
w = B1.verts( B1.edges(E1).verts(1) ).world_coords - B2.verts( B2.edges(E2).verts(1) ).world_coords;

a = dot3(u, u); % always >= 0
b = dot3(u, v);
c = dot3(v, v); % always >= 0
d = dot3(u, w);
e = dot3(v, w);

D = a*c - b*b; % always >= 0

sD = D;
tD = D;

SMALL_NUM = 0.00000001;
if (D < SMALL_NUM)
    % The lines are almost parallel
    sN = 0.0; % force using point P0 on segment S1 to prevent div by 0 later
    sD = 1.0; 
    tN = e;
    tD = c;
else
    % get the closest points on the infinite lines
    sN = b*e - c*d;
    tN = a*e - b*d;

    if (sN < 0.0)
        % sc < 0 --> the s = 0 edge is visible
        sN = 0.0;
        tN = e;
        tD = c;
    elseif (sN > sD)
        % sc > 1 --> the s = 1 edge is visible
        sN = sD;
        tN = e + b;
        tD = c;
    end
end


if (tN < 0.0)
    % tc < 0 --> the t = 0 edge is visible
    tN = 0.0;

    % recompute sc for this edge
    if (-d < 0.0)
        sN = 0.0;
    elseif (-d > a)
        sN = sD;
    else
        sN = -d;
        sD = a;
    end
elseif (tN > tD)
    % tc > 1 --> the t = 1 edge is visible
    tN = tD;

    % recompute sc for this edge
    if ((-d + b) < 0.0)
        sN = 0;
    elseif ((-d + b) > a)
        sN = sD;
    else
        sN = (-d + b);
        sD = a;
    end
end

% divide to get sc and tc
if (abs(sD) < SMALL_NUM)
    sc = 0.0;
else
    sc = sN / sD;
end

if (abs(tD) < SMALL_NUM)
    tc = 0.0;
else
    tc = tN / tD;
end

%% calculate closest points

ep1 = B1.verts( B1.edges(E1).verts(1) ).world_coords + (sc * u);
ep2 = B2.verts( B2.edges(E2).verts(1) ).world_coords + (tc * v);

%  ep1 = E1.verts(1).world_coords + (sc * u);
%  ep2 = E2.verts(1).world_coords + (tc * v);

%% define the gap distance between line segments
psi = norm(ep2 - ep1);


