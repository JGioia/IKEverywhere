function fanucDraw3D( path_file )
% Authors: Joseph Gioia & Prithve Shekar
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Plot a graphical representation of the FANUC S-500
%    Industrial robot with attached coordinate frames as it moves through a
%    series of poses defined by path_file.
%    

% Initialize the fanuc struct
fanuc = fanucInit();

% Get path position and color data
data = load(path_file);
s = data.s; % position
c = data.c;

% Draw FANUC initially in zero position (do not change)
prev_angles = zeros(1,6);
fanuc.handles = drawFanuc(prev_angles,fanuc);
hold on;

% Draw in 3D
for t = 1:size(s,2)
    % Set desired brush color from path file (think about how to handle
    % changes in color)
    fanuc.brush = c(t);
    theta = -pi/4 + (pi/2) * fanuc.brush;
    orientation = [1, 0, 0; 0, -1, 0; 0, 0, -1] * [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    
    % Set desired position for the tool from path file (not your choice)
    position = s(:, t);

    % Solve inverse kinematics for nearest solution
    transformation = [orientation, position; 0, 0, 0, 1];
    % [is_solution, next_joint_angles] = fanucIK(transformation, prev_angles, fanuc);
    [is_solution, next_joint_angles] = GeneralIK(fanuc, @fanucFK2, 6, position);
    f = @() GeneralIK(fanuc, @fanucFK2, 6, position);
    t = timeit(f)
    % Move robot using setFanuc() if solution exists
    
    if (is_solution)
        prev_angles = next_joint_angles;
        setFanuc(prev_angles, fanuc);
    end
    
    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    if (fanuc.brush ~= 0)
        brush_T = fanuc.tool{fanuc.brush};
        [T, ~] = fanucFK(prev_angles, fanuc);
        %position_ik = T(1:3, 4);
        brush_pos = T * brush_T * [0; 0; 0; 1];
        plot3(brush_pos(1), brush_pos(2), brush_pos(3) + 1000, '.', 'Color', fanuc.brush_colors{fanuc.brush})
    end
    
end
end
