function [T, fanuc_T] = fanucFK(joint_angles, fanuc)
% Authors: Joseph Gioia & Prithve Shekar
% Finds the transformation matrices between the frames of the fanuc at this
% joint angle.
% 
% Args:
%  joint_angles: A 6-element vector of joint angles (in radians)
%  fanuc: The structure output by fanuctInit()
%
% Returns: The transformation matrix from 6 to 0 and a cell array of the 6 
% trasformation matrices between each frame

% Find the individual tranformation matrices
alpha_0 = fanuc.parameters.a_1;
a_0 = 0;
d_1 = 0;
theta_1 = joint_angles(1);
T_0_1 = dhtf(alpha_0, a_0, d_1, theta_1);

alpha_1 = fanuc.parameters.a_2;
a_1 = fanuc.parameters.l_2;
d_2 = 0;
theta_2 = pi / 2 + joint_angles(2); % Remember this offset when calculating ik
T_1_2 = dhtf(alpha_1, a_1, d_2, theta_2);

alpha_2 = fanuc.parameters.a_3;
a_2 = fanuc.parameters.l_3;
d_3 = 0;
theta_3 = joint_angles(3);
T_2_3 = dhtf(alpha_2, a_2, d_3, theta_3);

alpha_3 = fanuc.parameters.a_4;
a_3 = fanuc.parameters.l_4;
d_4 = fanuc.parameters.d_4;
theta_4 = joint_angles(4);
T_3_4 = dhtf(alpha_3, a_3, d_4, theta_4);

alpha_4 = fanuc.parameters.a_5;
a_4 = fanuc.parameters.l_5;
d_5 = 0;
theta_5 = joint_angles(5);
T_4_5 = dhtf(alpha_4, a_4, d_5, theta_5);

alpha_5 = fanuc.parameters.a_6;
a_5 = fanuc.parameters.l_6;
d_6 = fanuc.parameters.d_6; % Is this right and if so how do I solve ik?
theta_6 = joint_angles(6);
T_5_6 = dhtf(alpha_5, a_5, d_6, theta_6);


% Return the cell array of transformations
fanuc_T = {T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6};
T = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6;
end