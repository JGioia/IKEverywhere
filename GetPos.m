
function [pos] = GetPos(robot, robotFK, joint_angles)
    T = robotFK(robot, joint_angles);
    pos = T * [0; 0; 0; 1];
    pos = [pos(1); pos(2); pos(3)];
end