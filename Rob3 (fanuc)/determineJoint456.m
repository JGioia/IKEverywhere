function [solutions] = determineJoint456(T, joint123Solutions, fanuc)
% Authors: Joseph Gioia and Prithve Shekar
% Determines potential solutions given potential solutions for joint 
% Returns a cell array of 6-vectors

solutions = {};
i = 1;

for x = joint123Solutions
    joint123Solution = x{1};
    theta1 = joint123Solution(1);
    theta2 = joint123Solution(2);
    theta3 = joint123Solution(3);
    solution = [theta1 theta2 theta3 0 0 0];

    % Find transformation from 3 to 6
    alpha_0 = fanuc.parameters.a_1;
    a_0 = 0;
    d_1 = 0;
    T_0_1 = dhtf(alpha_0, a_0, d_1, theta1);
    
    alpha_1 = fanuc.parameters.a_2;
    a_1 = fanuc.parameters.l_2;
    d_2 = 0;
    T_1_2 = dhtf(alpha_1, a_1, d_2, theta2);
    
    alpha_2 = fanuc.parameters.a_3;
    a_2 = fanuc.parameters.l_3;
    d_3 = 0;
    T_2_3 = dhtf(alpha_2, a_2, d_3, theta3);
    
    T_3_6 = computeTransInv(T_0_1 * T_1_2 * T_2_3) * T;

    % Find joint angle 5
    cos_5 = -T_3_6(2,3);
    sin_5 = sqrt(T_3_6(1,3) ^ 2 + T_3_6(3,3) ^ 2);
    if (~(isreal(cos_5) && isreal(sin_5)))
        solution(5) = 100000000;
        % Append to array
        solutions{i} = solution;
        i = i + 1;
        continue;
    end
    solution(5) = atan2(sin_5, cos_5);

    % Find joint angle 4
    cos_4 = T_3_6(1,3) / sin_5;
    sin_4 = T_3_6(3,3) / sin_5;
    solution(4) = atan2(sin_4, cos_4);

    % Find joint angle 6
    cos_6 = T_3_6(2, 1) / sin_5;
    sin_6 = T_3_6(2, 2) / sin_5;
    solution(6) = atan2(sin_6, cos_6);

    % TODO: If sin_5 == 0 determine correct rotation since 4+6 rotate in
    % same direction (gimbal lock)
    if (sin_5 == 0)
        solution(4) = 0;
        solution(6) = 0;
    end

    % Append to array
    solutions{i} = solution;
    i = i + 1;
end
end