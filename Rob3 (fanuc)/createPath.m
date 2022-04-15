function createPath
% Authors: Joseph Gioia & Prithve Shekar
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Create a path file for the FANUC S-500 robot to follow and
%    for fanucDraw3D to plot. Also visualizes path using plot3, which can
%    be compared with fanucDraw3D's output to verify validity of fanucIK
%    OUTPUTS - pathSet.m, containing the path and plotting colors
%            - 3D plot of expected path

    %Initializing variables to store path data
    angles = [];
    xSet = [];
    ySet = [];
    zSet = [];
    degree = 1; %Loop variable to keep robot in reachable workspace
    length = 1000; %Distance of points from axis 1 
   
    %Robot has a working set of angles in the approx 180-360 deg range of
    %joint 1
    for i = 180:360
        angles = [angles i];
    end
    %Allows reciprocating motion
    angles=[angles flip(angles)];
    
    %A simple helical curve for the first half of motion
    for i = 1:360
        zSet=[zSet 2*i];
    end
    %Rising sinusoidal curve for the second half of motion
    for i = 361:720
        zSet=[zSet (178+2*i-180*cos(deg2rad(10*i)))];
    end

    %Keeping the robot in the reachable workspace as the EE moves in the
    %z-direction
    for i=1:720
        [xval,yval]=Coordinates(length,angles(degree));
        xSet=[xSet xval];
        ySet=[ySet yval];
        length=length-1;
        degree=degree+1;
        if degree>359
            degree=1;
        end
    end
    
    %Example plot
    plot3(xSet,ySet,zSet)
    title('Expected path of FANUC-S500')
    
    s=[xSet;
       ySet;
       zSet];
   %Giving each half turn a color
   c = [1*ones(1,180) 2*ones(1,180) 3*ones(1,180) 4*ones(1,180)];
   save pathSet s c
   
end

function [xloc, yloc] = Coordinates(length,angle)
    %Returns coordinates of a point on a circle, given a radius and angle
    xloc=length*(sin(deg2rad(angle)));
    yloc=length*(cos(deg2rad(angle)));
end

