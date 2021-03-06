function [] = Rob1()
    robot = Rob1Init();
    joint_angles = [0, 0];
    while true
        Rob1Draw(robot, joint_angles);
        pos_desired = ginput(1);
        pos_desired = [pos_desired(1); pos_desired(2)];
        pos_desired3d = [pos_desired(1); pos_desired(2); 0];
        [is_solution, new_joint_angles] = GeneralIK(robot, @Rob1FK, 2, pos_desired3d);
        f = @() GeneralIK(robot, @Rob1FK, 2, pos_desired3d);
        t = timeit(f, 2)
        if is_solution
            joint_angles = new_joint_angles;
            pos_real = Rob1Pos(robot, joint_angles);
            dif = GetDiff(pos_desired, pos_real);
            disp("Position diff: ");
            disp(dif);
        else
            disp("Invalid location");
        end
    end
end