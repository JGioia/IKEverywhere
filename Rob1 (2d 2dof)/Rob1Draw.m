function [] = Rob1Draw(robot, joint_angles)
    [T, T_0_1, T_1_2, ~] = Rob1FK(robot, joint_angles);
    P_0 = [0; 0; 0; 1];
    P_1 = T_0_1 * T_1_2 * [0; 0; 0; 1];
    P_2 = T * [0; 0; 0; 1];

    x = [P_0(1), P_1(1), P_2(1)];
    y = [P_0(2), P_1(2), P_2(2)];

    plot(x, y);
    dist = robot.l_1 + robot.l_2;
    xlim([-dist, dist]);
    ylim([-dist, dist]);
end