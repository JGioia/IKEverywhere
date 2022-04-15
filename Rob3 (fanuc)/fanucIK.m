function [is_solution, joint_angles] = fanucIK(T, prev_joint_angles, fanuc)
% Authors: Joseph Gioia & Prithve Shekar
% Does the inverse kinematics for fanuc and finds the closest valid
% solution.
%
% Args:
%  T: The transformation matrix describing the desired postion and
%   orientation of the end effector.
%  prev_joint_angle: A 6-vector describing the joint angles of the robot
%   before moving.
%  fanuc: The structure returned by fanucInit()
%
% Returns:
%  is_solution: A boolean of whether a solution exists
%  joint_angles: A 6-vector that represents the joint angles for the
%   nearest valid solution.

% Extract geometric variables from T
%x = T(1, 4)-0.17;
%y = T(2, 4)-0.006;
%z = T(3, 4)-0.18;
T(1,4) = T(1,4) - 180;
T(3,4) = T(3,4) - 180;
x = T(1, 4);
y = T(2, 4);
z = T(3, 4);

% Determine whether T is within the workspace, if not return false
xmin = fanuc.workspace(1);
xmax = fanuc.workspace(2);
ymin = fanuc.workspace(3);
ymax = fanuc.workspace(4);
zmin = fanuc.workspace(5); 
zmax = fanuc.workspace(6);
if (x < xmin || x > xmax || y < ymin || y > ymax || z < zmin || z > zmax)
    is_solution = false;
    return;
end

T(1,4) = x;
T(2,4) = y;
T(3,4) = z;

% Find joint angles 2 and 3
joint123Solutions = determineJoint123(T, fanuc);

% Find joint angles 4, 5, and 6
solutions = determineJoint456(T, joint123Solutions, fanuc);

% Return the closest valid solution, if none return false
[is_solution, joint_angles] = findClosestSolution(prev_joint_angles, solutions, fanuc);

end