function [ T, T_0_1, T_1_2, T_2_3 ] = Rob1FK(robot, joint_angles)
    c_1 = cos(joint_angles(1));
    s_1 = sin(joint_angles(1));
    c_2 = cos(joint_angles(2));
    s_2 = sin(joint_angles(2));
    T_0_1 = [
        c_1, -s_1, 0, 0;
        s_1, c_1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
    T_1_2 = [
        c_2, -s_2, 0, robot.l_1;
        s_2, c_2, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
    T_2_3 = [
        1, 0, 0, robot.l_2;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
    T = T_0_1 * T_1_2 * T_2_3;
end