function [ fanuc_struct ] = fanucInit()
% Authors: Joseph Gioia & Prithve Shekar
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Initialize a structure fanuc_struct to contain important
%    robot information that will be passed into various FANUC simulation
%    functions.
%
%    ADDITIONAL CODE NEEDED:
%
%    Add all dimensions that you will make use of to the nested structure,
%    fanuc_struct.parameters. (Now you never have to remember the actual
%    values!)
%
%    Observe the forward transforms from the end effector to the four
%    different tool brush frames.
%
%    Provide the joint limits from the FANUC data sheet, assuming the
%    ranges are centered at the zero configuration. (Their values are
%    currently all zero.
%
%    Provide the limits of the workspace. (Estimate a box that tightly
%    surrounds the reachable workspace of the robot.)
%

% FANUC dimensions in millimeters
l_1 = 1000; % [mm]
l_2 = 300; % [mm]
l_3 = 900; % [mm]
d_4 = 1600;
l_4 = 180; % [mm]
l_5 = 0; % [mm]
d_6 = 180; % [mm] (End Effector)
l_6 = 0;

% Tool dimensions in millimeters
l_t_rad = 50; % [mm]
l_t = 300; % [mm]

% Fill in FANUC D-H parameters and other necessary parameters 
%Lengths one to six. See DH parameter Excel file to see what they
%correspond to
fanuc_struct.parameters.l_1 = l_1;
fanuc_struct.parameters.l_2 = l_2;
fanuc_struct.parameters.l_3 = l_3;
fanuc_struct.parameters.d_4 = d_4;
fanuc_struct.parameters.l_4 = l_4;
fanuc_struct.parameters.l_5 = l_5;
fanuc_struct.parameters.d_6 = d_6;
fanuc_struct.parameters.l_6 = l_6;
fanuc_struct.parameters.l_t_rad = l_t_rad;
fanuc_struct.parameters.l_t = l_t;

%Non-parameter angles one to six. See DH parameter Excel file to see what
%they correspond to
fanuc_struct.parameters.a_1=0;
fanuc_struct.parameters.a_2=pi/2;
fanuc_struct.parameters.a_3=0;
fanuc_struct.parameters.a_4=pi/2;
fanuc_struct.parameters.a_5=-pi/2;
fanuc_struct.parameters.a_6=pi/2;

%Tool lengths
fanuc_struct.parameters.l_t = l_t;
fanuc_struct.parameters.l_t_rad = l_t_rad;

% FANUC tool brush frames relative to end-effector frame (do not change)
fanuc_struct.tool{1} = makehgtform('zrotate',5*pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);
fanuc_struct.tool{2} = makehgtform('zrotate',7*pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);
fanuc_struct.tool{3} = makehgtform('zrotate',pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);
fanuc_struct.tool{4} = makehgtform('zrotate',3*pi/4,...
    'translate',[-l_t_rad,0,0],'yrotate',-pi/4,'translate',[0,0,l_t]);

% FANUC tool brush selection (0 through 4)
fanuc_struct.brush = 4; % (change this and see what happens)

% FANUC tool brush colors (play with these if you want)
fanuc_struct.brush_colors{1} = [0.4940,0.1840,0.5560];
fanuc_struct.brush_colors{2} = [0.9290,0.6940,0.1250];
fanuc_struct.brush_colors{3} = [0.8500,0.3250,0.0980];
fanuc_struct.brush_colors{4} = [0,0.4470,0.7410];

% FANUC base (zero) frame relative to the "station" frame
fanuc_struct.base = makehgtform('translate',[0,0,l_1]);

% FANUC joint limits (deg)
% As joints are centered on range, have half range in each direction
deg2rad = pi/180;
fanuc_struct.joint_limits{1} = [-150,150]*deg2rad;
fanuc_struct.joint_limits{2} = [-80,80]*deg2rad;
fanuc_struct.joint_limits{3} = [-80,80]*deg2rad;
fanuc_struct.joint_limits{4} = [-240,240]*deg2rad;
fanuc_struct.joint_limits{5} = [-120,120]*deg2rad;
fanuc_struct.joint_limits{6} = [-450,450]*deg2rad;

% Set bounds on the cartesian workspace of the FANUC for plotting in the
% form:  [ xmin, xmax, ymin, ymax, zmin, zmax]
% Added 100mm to all machine limits, using z axis as height
% NOTE: Not sure about z bc frame 0 definition
fanuc_struct.workspace = [-2839, 2839, -2839, 2839, -1821, 3336];

% Set colors to be drawn for each link and associated frame, including the
% tool
%I like red...
fanuc_struct.colors{1} = [1,0,0];
fanuc_struct.colors{2} = [1,0,0];
fanuc_struct.colors{3} = [1,0,0];
fanuc_struct.colors{4} = [1,0,0];
fanuc_struct.colors{5} = [1,0,0];
fanuc_struct.colors{6} = [1,0,0];
fanuc_struct.colors{7} = [1,0,0];

end

