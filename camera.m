clc
clear all
close all
rosshutdown
%%
rosinit
%%
dobot = DobotMagician();
%%
dobot.InitaliseRobot();
%% set joint
joint_target = [0.0,0.4,0.0,0.0];
dobot.PublishTargetJoint(joint_target);

%% Publish custom end effector pose
end_effector_position = [0.2,0,0.05];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% record images at dif positions
end_effector_matrix = [ 0.2,0.0,0.1;
                        0.2,0.0,0.15;
                        0.2,0.0,0.1;
                        0.3,0.0,0.1;
                        0.2,-0.1,0.15;
                        0.2,0.1,0.1;
                        0.3,0.05,0.1;
                        0.25,0.1,0.15;
                        0.25,-0.05,0.05;
                        0.25,-0.1,0.1;
                      ];
image_matrix = [];
cameraRgbSub = rossubscriber('/camera/color/image_raw');
pause(2);

for i = 1:size(end_effector_matrix)
    dobot.PublishEndEffectorPose(end_effector_matrix(i,:),end_effector_rotation)
    image_data = readImage(cameraRgbSub.LatestMessage);
    image_matrix = [image_matrix;{image_data}];
    imwrite(image_matrix{i},strcat('/images/image',num2str(i),'.png'));
    pause(3);
end
%%
for i = 1:size(image_matrix)
    f1 = figure
    hold on
    imshow(image_matrix{i});
    % images_h = imshow(image_matrix{i});
    %imwrite(image_matrix{i},strcat('image',num2str(i),'.png'));
end


%% captured calibration corners
for i = 1:size(image_matrix)
    f1 = figure
    hold on
    [imagePoints,boardSize] = detectCheckerboardPoints(images_h.CData);
    J = insertText(images_h.CData,imagePoints,1:size(imagePoints,1));
    J = insertMarker(J,imagePoints,'o','MarkerColor','red','Size',5);
    images_h.CData = J %imshow(J);
    % image_h.CData = J;
    title(sprintf('Detected a %d x %d Checkerboard',boardSize));
end

%% camera
cameraRgbSub = rossubscriber('/camera/color/image_raw');
pause(2);
% cam_message = cameraRgbSub.LatestMessage;
% image_data = readImage(cam_message);
% testread = imread('test.jpg');
% imshow(image_data);
%imshow(image_data);
image_data = readImage(cameraRgbSub.LatestMessage);
image_h = imshow(image_data);
set (gcf,'units','normalized','outerposition',[0 0 1 1]);

[imagePoints,boardSize] = detectCheckerboardPoints(image_h.CData);
J = insertText(image_h.CData,imagePoints,1:size(imagePoints,1));
J = insertMarker(J,imagePoints,'o','MarkerColor','red','Size',5);
% imshow(J);
image_h.CData = J;
title(sprintf('Detected a %d x %d Checkerboard',boardSize));



% 
% tic
% while 1
%     image_h.CData = readImage(cameraRgbSub.LatestMessage);
%     [imagePoints,boardSize] = detectCheckerboardPoints(image_h.CData);
%     J = insertText(image_h.CData,imagePoints,1:size(imagePoints,1));
%     J = insertMarker(J,imagePoints,'o','MarkerColor','red','Size',5);
%     image_h.CData = J;
%     title(sprintf('Detected a %d x %d Checkerboard',boardSize));
%     drawnow;
%     toc;
% end

% 
% depthSub = rossubscriber('/camera/depth/image_rect_raw');
% pause(1);
% msg = depthSub.LatestMessage;
% img = readImage(msg);
% depthImage_h =imshow(img)