function [solutions] = determineJoint123(T, fanuc)
% Authors: Joseph Gioia and Prithve Shekar
% Determines potential solutions for joints 1,2,3 given joint angle 1
% Returns a cell array of 3-vectors

% Find position of joint 4 wrt 0
rotation_0_6 = T(1:3, 1:3);
pos_0_6 = T(1:3, 4);
pos_4_6 = [0; 0; fanuc.parameters.d_6];
pos_0_4 = pos_0_6 + (rotation_0_6 * -pos_4_6);

% Find theta 1
theta1 = atan2(pos_0_4(2), pos_0_4(1)); 

% Find position of joint 4 wrt 2 (note that pos_2_4 is actually the
% distance from origin 2 to origin 4 as measured within frame 0)
alpha_0 = fanuc.parameters.a_1;
a_0 = 0;
d_1 = 0;
T_0_1 = dhtf(alpha_0, a_0, d_1, theta1);
%pos_1_4 = computeTransInv(T_0_1) * [pos_0_4; 1];
pos_1_4 = pos_0_4; % TODO: IS this right??
pos_2_4 = [sqrt(pos_1_4(1)^2 + pos_1_4(2)^2) - fanuc.parameters.l_2; 0; pos_1_4(3)];

% Find theta 2
len_2_4 = sqrt(pos_2_4(1)^2 + pos_2_4(2)^2 + pos_2_4(3)^2);
len_link2 = fanuc.parameters.l_3;
len_link3 = sqrt(fanuc.parameters.d_4^2 + fanuc.parameters.l_4^2);
theta2_part1 = atan2(pos_2_4(3), sqrt(pos_2_4(1)^2 + pos_2_4(2)^2));
theta2_part2_1 = acos((len_2_4^2 + len_link2^2 - len_link3^2) / (2 * len_2_4 * len_link2));
theta2_part2_2 = -theta2_part2_1;
theta2_2 = theta2_part1 - theta2_part2_2 - pi/2;
theta2_1 = -theta2_2;

% Find theta 3
theta3_part1_1 = acos((len_link2^2 + len_link3^2 - len_2_4^2) / (2 * len_link3 * len_link2)) - pi / 2;
theta3_part1_2 = -theta3_part1_1;
theta3_offset = atan2(180, 1600);
theta3_1 = theta3_part1_1 - theta3_offset;
theta3_2 = theta3_part1_2 - theta3_offset; % TODO: This isn't always right (is it ever right?)

% TODO: Test if transform 0 to 4 (arbitrary choice for theta4) 
alpha_0 = fanuc.parameters.a_1;
a_0 = 0;
d_1 = 0;
T_0_1 = dhtf(alpha_0, a_0, d_1, theta1);

alpha_1 = fanuc.parameters.a_2;
a_1 = fanuc.parameters.l_2;
d_2 = 0;
T_1_2 = dhtf(alpha_1, a_1, d_2, theta2_2 + pi/2);

alpha_2 = fanuc.parameters.a_3;
a_2 = fanuc.parameters.l_3;
d_3 = 0;
T_2_3 = dhtf(alpha_2, a_2, d_3, theta3_2);

alpha_3 = fanuc.parameters.a_4;
a_3 = fanuc.parameters.l_4;
d_4 = fanuc.parameters.d_4;
T_3_4 = dhtf(alpha_3, a_3, d_4, 0);

T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4;
pos = T_0_4(1:3, 4);

% Return solutions
solutions = {[theta1 theta2_2 theta3_1]};
end