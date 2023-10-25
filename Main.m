clc
clear all
close all
rosshutdown
%%
rosinit
%%
dobot = DobotMagician();
%%
dobot.InitaliseRobot;

%% Publish custom end effector pose
end_effector_position = [0.2,0.0,0.1];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
%%
joint_target = [pi/4, pi/6, pi/4, pi/2, 0;];
dobot.PublishTargetJoint(joint_target);

%% Main servo loop
% images = imageDatastore("cameraEffector/");
% imageFiles = images.Files;
cameraRgbSub = rossubscriber('/camera/color/image_raw');
pause(1);

image_data = readImage(cameraRgbSub.LatestMessage);
%image_matrix = [image_matrix;{image_data}];
%imwrite(image_data,'pixelLocs.png');
%
while 1
    image_data = readImage(cameraRgbSub.LatestMessage);
    pause(0.5);
    
    % Detect checkerboard points
    [imagePoints, boardSize] = detectCheckerboardPoints(image_data);
    pause(0.25);

    % Control
    f = params.FocalLength(1,1); %400; %params.FocalLength(1,1);-0.01
    p = params.PrincipalPoint; %length(img)/2; %params.PrincipalPoint;
    Z = 50; %63.5?
    l = 0.1; %lambda 
    
    % Populate Observation vector using values from "feature detection section"
    % Populate target vector using image size (define your own target size for
    % the square)
    Target = [  275.94,127.88; %corner pixel locations
                275.94,219.64;
                312.64,200.67;
                388.37,218.60];
    
    Obs = [ imagePoints(42,1),imagePoints(42,2); % topl, bottoml, topr, bottom r
            imagePoints(37,1),imagePoints(37,2);
            imagePoints(26,1),imagePoints(26,2);
            imagePoints(1,1),imagePoints(1,2)];
    
    %
    xy = (Target-p)/f;
    Obsxy = (Obs-p)/f;
    
    %
    n = length(Target(:,1));
    
    Lx = [];
    for i=1:n;
        Lxi = FuncLx(xy(i,1),xy(i,2),Z);
        Lx = [Lx;Lxi];
    end
    
    %
    e2 = Obsxy-xy;
    e = reshape(e2',[],1);
    de = -e*l;
    
    %
    Lx2 = inv(Lx'*Lx)*Lx';
    Vc = -l*Lx2*e

    %%
    current_joints = dobot.GetCurrentJointState;
    joint_target = [current_joints(1) - Vc(1)*0.05, current_joints(2), current_joints(3), current_joints(4)]

    if joint_target(1) > deg2rad(135)
        joint_target(1) = deg2rad(135);
    elseif joint_target(1) < deg2rad(-135)
        joint_target(1) = deg2rad(-135);
    end
    if joint_target(2) > deg2rad(80)
        joint_target(2) = deg2rad(80);
    elseif joint_target(2) < deg2rad(5)
        joint_target(2) = deg2rad(5);
    end
    if joint_target(3) > pi/2
        joint_target(3) = pi/2;
    elseif joint_target(3) < -pi/2
        joint_target(3) = -pi/2;
    end
    if joint_target(4) > deg2rad(85)
        joint_target(4) = deg2rad(85);
    elseif joint_target(4) < deg2rad(-85)
        joint_target(4) = deg2rad(-85);
    end

    %joint_target
    dobot.PublishTargetJoint(joint_target);
    pause(0.75);

end

