image_matrix = [];
cameraRgbSub = rossubscriber('/camera/color/image_raw');
pause(2);

for i = 1:10
    image_data = readImage(cameraRgbSub.LatestMessage);
    image_matrix = [image_matrix;{image_data}];
    imwrite(image_matrix{i},strcat('images/test/image',num2str(i),'.png'));
    pause(3);
end