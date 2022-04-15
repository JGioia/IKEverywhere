function [] = Rob2()
    robot = Rob2Init();
    joint_angles = zeros(length(robot.l));
    while true
        Rob2Draw(robot, joint_angles);
        pos_desired = ginput(1);
        pos_desired = [pos_desired(1); pos_desired(2)];
        pos_desired3d = [pos_desired(1); pos_desired(2); 0];
        [is_solution, new_joint_angles] = GeneralIK(robot, @Rob2FK, length(robot.l), pos_desired3d);
        if is_solution
            joint_angles = new_joint_angles;
            pos_real = GetPos(robot, @Rob2FK, joint_angles);
            dif = GetDiff(pos_desired, pos_real);
            disp("Position diff: ");
            disp(dif);
        else
            disp("Invalid location");
        end
    end
end