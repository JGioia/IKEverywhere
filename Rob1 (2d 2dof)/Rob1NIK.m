function [is_solution, joint_angles] = Rob1NIK(robot, prev_angles, pos_desired)
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

    joint_angles = prev_angles;
    for i = 1:100
        pos_current = Rob1Pos(robot, joint_angles);
        delta_pos = pos_desired - pos_current;
        delta_theta = pinv(Rob1Jacobian(robot, joint_angles)) * delta_pos;
        joint_angles = joint_angles + delta_theta;
    end

    is_solution = true;
end

function [jacobian] = Rob1Jacobian(robot, joint_angles)
    l_1 = robot.l_1;
    l_2 = robot.l_2;
    s_1 = sin(joint_angles(1));
    c_1 = cos(joint_angles(1));
    s_2 = sin(joint_angles(2));
    c_2 = cos(joint_angles(2));

    dx_dt1 = -l_2 * s_1 * c_2 - l_2 * c_1 * s_2 - l_1 * s_1;
    dy_dt1 = -l_2 * s_1 * s_2 + l_2 * c_1 * c_2 + l_1 * c_1;
    dx_dt2 = -l_2 * c_1 * s_2 - l_2 * s_1 * c_2;
    dy_dt2 = l_2 * c_1 * c_2 - l_2 * s_1 * s_2;

    jacobian = [dx_dt1, dx_dt2; dy_dt1, dy_dt2];
end
