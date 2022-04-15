function [T, T_array] = Rob2FK(robot, joint_angles)
    T = [
        1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
    T_array = {dhtf(0, 0, 0, joint_angles(1))};
    T = T * T_array{1};
    for i = 2:length(joint_angles)
        T_array{i} = dhtf(0, robot.l(i - 1), 0, joint_angles(i));
        T = T * T_array{i};
    end
    T_array{length(joint_angles) + 1} = dhtf(0, robot.l(length(joint_angles)), 0, 0);
    T = T * T_array{length(joint_angles) + 1};
end