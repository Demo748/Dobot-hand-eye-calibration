images = imageDatastore("demo/patternEffector/");
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
%%
% visualise
% showReprojectionErrors(params);
showExtrinsics(params,'CameraCentric')
% saveas(gcf,'demo/cameraEffector/visualised/showExtrinsics.png');
% plotCamera
%%
% estimate pose
% ext = estimateExtrinsics
% intrinsics = params.Intrinsics;
% camExtrinsics = estimateExtrinsics(imagePoints,worldPoints,intrinsics)
camPose1 = extr2pose(params.PatternExtrinsics(1));

%%
for i = 1:size(imageFiles)
    figure
    hold on
    image_h = imshow(imageFiles{i});
    [imagePoints,boardSize] = detectCheckerboardPoints(image_h.CData);
    J = insertText(image_h.CData,imagePoints,1:size(imagePoints,1));
    J = insertMarker(J,imagePoints,'o','color','red','Size',5);
    % J = insertMarker(J,imagePoints,'o','MarkerColor','red','Size',5);
    % imshow(J);
    image_h.CData = J;
    title(sprintf('Detected a %d x %d Checkerboard',boardSize));
    % exportgraphics(gcf,[strcat('demo/patternEffector/visualised/imagePoints',num2str(i),'.png')]);
    % saveas(gcf,[strcat('demo/cameraEffector/imagePoints',num2str(i),'.png')]);
end