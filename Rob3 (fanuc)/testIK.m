function [] = testIK(joint_angles)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
fanuc = fanucInit();
T = fanucFK(joint_angles, fanuc);
[is_solution, joint_angles2] = fanucIK(T, [0 0 0 0 0 0], fanuc);
joint_angles2
pos = T(1:3, 4)
T2 = fanucFK(joint_angles2, fanuc);
pos2 = T2(1:3, 4)
end