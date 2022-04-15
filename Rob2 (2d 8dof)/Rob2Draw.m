function [] = Rob2Draw(robot, joint_angles)
    [T, T_array] = Rob2FK(robot, joint_angles);
    x = [0];
    y = [0];

    T_cur = [
        1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
    for i = 1:length(joint_angles)
        T_cur = T_cur * T_array{i};
        pos = T_cur * [0; 0; 0; 1];
        x(i + 1) = pos(1);
        y(i + 1) = pos(2);
    end
    pos = T * [0; 0; 0; 1];
    x(length(joint_angles) + 2) = pos(1);
    y(length(joint_angles) + 2) = pos(2);

    plot(x, y);
    dist = sum(robot.l);
    xlim([-dist, dist]);
    ylim([-dist, dist]);
end
