function [current_pos] = Rob1Pos(robot, joint_angles)
    current_pos = Rob1FK(robot, joint_angles) * [0; 0; 0; 1];
    current_pos = [current_pos(1); current_pos(2)];
end