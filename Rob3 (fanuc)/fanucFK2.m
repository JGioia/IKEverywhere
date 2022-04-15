function [T] = fanucFK2(robot, joint_angles)
    T = fanucFK(joint_angles, robot) * robot.tool{robot.brush};
end