function [is_solution, joint_angles] = Rob1AIK(robot, pos_desired)
    l_1 = robot.l_1;
    l_2 = robot.l_2;
    x = pos_desired(1);
    y = pos_desired(2);
    r = sqrt(x^2 + y^2);
    if (r > l_1 + l_2 || r < l_1 - l_2)
        is_solution = false;
        joint_angles = [0, 0];
        return;
    end

    t1_almost = acos((-(l_2 ^ 2) + l_1 ^2 + r ^ 2) / (2 * l_1 * r));
    t = atan2(y, x);
    t1_1 = t + t1_almost;
    t1_2 = t - t1_almost;

    t2_almost = acos((-(r ^ 2) + l_1 ^ 2 + l_2 ^2) / (2 * l_1 * l_2));
    t2_1 = pi - t2_almost;
    t2_2 = -t2_1;

    is_solution = true;
    joint_angles = [t1_1, t2_2];
end
