% clc
% clear all
% close all
% rosshutdown
% pause(1)
% %%
% rosinit
% pause(1)
% %%
% dobot = DobotMagician();
% pause(6)
% %%
% dobot.InitaliseRobot;

%% Publish custom end effector pose
end_effector_position = [0.2,0.0,0.1];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause(3)
dobot.GetCurrentJointState
%%
% joint_target = [pi/4, pi/6, pi/4, pi/2, 0;];
% j = [-2.1911, 0.5236, 0.7854, 0.14908];
% dobot.PublishTargetJoint(j);
% pause(1)

%% Image capture calibration
cameraRgbSub = rossubscriber('/camera/color/image_raw');
pause(1);

image_data = readImage(cameraRgbSub.LatestMessage);
imshow(image_data);
%imwrite(image_data,'pixelLocs.png');
% image_h = imshow(image_data);
[cornerPoints,boardSize] = detectCheckerboardPoints(image_data);

pattern_effector = 0; % change to match camera setup, 0 or 1

switch pattern_effector
    case 0
        % record images at dif positions
        % end_effector_matrix = [ 0.2,0.0,0.1;
        %                 0.2,0.0,0.15;
        %                 0.2,0.03,0.15;
        %                 0.2,-0.02,0.15;
        %                 0.2,-0.05,0.15;
        %                 0.2,0.05,0.1;
        %                 0.195,0.01,0.15;
        %                 0.195,0.02,0.15;
        %                 0.2,-0.03,0.1;
        %                 0.2,-0.01,0.15;
        %               ];
        end_effector_matrix = [ % 63.5cm away x direction, 11.5cm camera height centre
            0, -0.0184, 0.2402, 0, 0;
            0, -0.0439, -0.1031, 0, 0;
            0.1489, -0.0273, -0.1026, -0.1489, 0;
            -0.0997, -0.0365, -0.1028, 0.0997, 0;
            -0.2450, 0.0016, -0.1022, 0.2450, 0;
            0.1549, -0.0151, 0.1750, -0.1549, 0;
            0.0549, -0.0786, -0.0989, -0.0549, 0;
            0.1022, -0.0732, -0.1047, -0.1022, 0;
            -0.1425, -0.0065, 0.2311, 0.1425, 0;
            -0.0500,  -0.0420, -0.1030, 0.0500, 0;
        ];
        image_path = "demo/cameraEffector/";
    case 1
        end_effector_matrix = [ % 63.5cm away x direction, 11.5cm camera height centre
            0, pi/4, pi/4, pi/2, 0;
            0, pi/6, pi/4, pi/2, 0;
            pi/6, pi/4, pi/4, pi/2, 0;
            pi/6, pi/6, pi/4, pi/2, 0;
            pi/8, pi/4, pi/4, pi/2, 0;
            pi/8, pi/6, pi/4, pi/2, 0;
            -pi/4, pi/4, pi/4, pi/2, 0;
            -pi/4, pi/6, pi/4, pi/2, 0;
            -pi/6, pi/4, pi/4, pi/2, 0;
            -pi/6, pi/6, pi/4, pi/2, 0;
            -pi/8, pi/4, pi/4, pi/2, 0;
            -pi/8, pi/6, pi/4, pi/2, 0;
        ];
        image_path = "demo/patternEffector/";
end

image_matrix = [];

for i = 1:size(end_effector_matrix)
    %dobot.PublishEndEffectorPose(end_effector_matrix(i,:),end_effector_rotation)
    dobot.PublishTargetJoint(end_effector_matrix(i,:));
    image_data = readImage(cameraRgbSub.LatestMessage);
    image_matrix = [image_matrix;{image_data}];
    imwrite(image_matrix{i},strcat(image_path,'image',num2str(i),'.png'));
    pause(3);
end
%
% for i = 1:size(image_matrix)
%     f1 = figure
%     hold on
%     imshow(image_matrix{i});
%     images_h = imshow(image_matrix{i});
%     %imwrite(image_matrix{i},strcat('image',num2str(i),'.png'));
% end

%% Calculate calibration params
images = imageDatastore(image_path);
imageFiles = images.Files;

% Detect checkerboard points
[imagePoints, boardSize] = detectCheckerboardPoints(imageFiles);

% Define world points of the checkerboard (assuming square size is 1 unit)
squareSize = 14;
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

I = readimage(images,1);
imageSize = [size(I,1) size(I,2)];
params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);

for i = 1:size(imageFiles)
    figure
    hold on
    image_h = imshow(imageFiles{i});
    [imagePoints,boardSize] = detectCheckerboardPoints(image_h.CData);
    J = insertText(image_h.CData,imagePoints,1:size(imagePoints,1));
    J = insertMarker(J,imagePoints,'o','MarkerColor','red','Size',5);
    % imshow(J);
    image_h.CData = J;
    title(sprintf('Detected a %d x %d Checkerboard',boardSize));
end
%% Main visual servo loop
while 1
    image_data = readImage(cameraRgbSub.LatestMessage);
    pause(0.5);
    
    % Detect checkerboard points
    [imagePoints, boardSize] = detectCheckerboardPoints(image_data);
    pause(0.25);
    if size(imagePoints) < 42
        continue;
    end

    % Control
    f = params.FocalLength(1,1); %400; %params.FocalLength(1,1);-0.01
    p = params.PrincipalPoint; %length(img)/2; %params.PrincipalPoint;
    Z = 50; %63.5?
    l = 0.1; %lambda 
    
    % Populate Observation vector using values from "feature detection section"
    % Populate target vector using image size (define your own target size for
    % the square)
    % Target = [  275.94,127.88; %corner pixel locations
    %             275.94,219.64;
    %             312.64,200.67;
    %             388.37,218.60];
    Target = [  cornerPoints(42,1),cornerPoints(42,2);
                cornerPoints(37,1),cornerPoints(37,2);
                cornerPoints(26,1),cornerPoints(26,2);
                cornerPoints(1,1),cornerPoints(1,2)];
    
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
    pause(0.5);
    joint_target = [current_joints(1) - Vc(1)*0.1, current_joints(2), current_joints(3), current_joints(4)]

    if joint_target(1) > deg2rad(120)
        joint_target(1) = deg2rad(120);
    elseif joint_target(1) < deg2rad(-120)
        joint_target(1) = deg2rad(-120);
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
    
    % if abs(joint_target(1) - current_joints(1)) < 0.1
    %     joint_target(1) = current_joints(1);
    % end

    new_joint_target = joint_target
    dobot.PublishTargetJoint(joint_target);
    pause(0.75);

end

