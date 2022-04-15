function [is_solution, joint_angles] = GeneralIK(robot, robotFK, num_joints, pos_desired)
    joint_angles = zeros(num_joints);
    for i = 1:10
        pos_current = GetPos(robot, robotFK, joint_angles);
        delta_pos = pos_desired - pos_current;
        delta_theta = pinv(GetJacobian(robot, robotFK, joint_angles)) * delta_pos;
        joint_angles = joint_angles + delta_theta;
    end

    pos_current = GetPos(robot, robotFK, joint_angles);
    dpos = GetDiff(pos_desired, pos_current);
    if (dpos < 0.005)
        is_solution = true;
    else
        is_solution = false;
    end
end

function [jacobian] = GetJacobian(robot, robotFK, joint_angles)
    dx = [];
    dy = [];
    dz = [];
    
    dt = 0.01;
    for i = 1:length(joint_angles)
        adjusted_angles = joint_angles;
        adjusted_angles(i) = adjusted_angles(i) - (dt / 2);
        pos_1 = GetPos(robot, robotFK, adjusted_angles);
        adjusted_angles(i) = adjusted_angles(i) + dt;
        pos_2 = GetPos(robot, robotFK, adjusted_angles);
        dpos = (pos_2 - pos_1) / dt;
        dx(1, i) = dpos(1);
        dy(1, i) = dpos(2);
        dz(1, i) = dpos(3);
    end

    jacobian = [dx;dy;dz];
end