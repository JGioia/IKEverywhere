function [is_solution, joint_angles] = findClosestSolution(prev_joint_angles, solutions, fanuc)
% Authors: Joseph Gioia & Prithve Shekar
% Finds the joint angle vector closest to the previous joint angle in the
% cell array of valid solutions
%
% Args:
%  prev_joint_angles: A 6-vector that represents the joint angles before
%   moving
%  valid_solutions: A cell array of 6-vectors that represent the joint
%   angles of possible solutions
%  fanuc: The structure output by fanucInit()
%
% Returns: A 6-vector that is the closest element of valid_solutions to
% prev_joint_angles.

closest_dist = Inf;
joint_angles = solutions{1};
is_solution = false;

for x = solutions
    solution = x{1};
    for i = 1:6
        solution(1) = mod(solution(1), 2 * pi);
        if (solution(1) > pi)
            solution(1) = solution(1) - 2 * pi;
        end
    end

    if (isValidSolution(fanuc, solution))
        is_solution = true;
        dist = 0;
        dist = dist + abs(solution(1) - prev_joint_angles(1));
        dist = dist + abs(solution(2) - prev_joint_angles(2));
        dist = dist + abs(solution(3) - prev_joint_angles(3));
        dist = dist + abs(solution(4) - prev_joint_angles(4));
        dist = dist + abs(solution(5) - prev_joint_angles(5));
        dist = dist + abs(solution(6) - prev_joint_angles(6));
    
        if (dist < closest_dist)
            closest_dist = dist;
            joint_angles = solution;
        end
    end
end

end