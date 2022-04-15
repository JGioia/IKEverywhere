function [is_valid] = isValidSolution(fanuc, joint_angles)
% Authors: Joseph Gioia & Prithve Shekar
% Determines whether the joint angle given is valid.
%
% Args:
%  fanuc: A structure returned by fanucInit()
%  joint_angles: A 6-vector that represents the joint angles
%
% Returns: A boolean value of whether the solution is valid.

valid1 = joint_angles(1) >= fanuc.joint_limits{1}(1) && joint_angles(1) <= fanuc.joint_limits{1}(2);
valid2 = joint_angles(2) >= fanuc.joint_limits{2}(1) && joint_angles(2) <= fanuc.joint_limits{2}(2);
valid3 = joint_angles(3) >= fanuc.joint_limits{3}(1) && joint_angles(3) <= fanuc.joint_limits{3}(2);
valid4 = joint_angles(4) >= fanuc.joint_limits{4}(1) && joint_angles(4) <= fanuc.joint_limits{4}(2);
valid5 = joint_angles(5) >= fanuc.joint_limits{5}(1) && joint_angles(5) <= fanuc.joint_limits{5}(2);
valid6 = joint_angles(6) >= fanuc.joint_limits{6}(1) && joint_angles(6) <= fanuc.joint_limits{6}(2);


is_valid = valid1 && valid2 && valid3 && valid4 && valid5 && valid6 && isreal(joint_angles(1)) && isreal(joint_angles(2)) && isreal(joint_angles(3)) && isreal(joint_angles(4)) && isreal(joint_angles(5)) && isreal(joint_angles(6));
end