image_h = imshow("pixelLocs.png")
[imagePoints,boardSize] = detectCheckerboardPoints(image_h.CData);
J = insertText(image_h.CData,imagePoints,1:size(imagePoints,1));
J = insertMarker(J,imagePoints,'o','MarkerColor','red','Size',5);
% imshow(J);
image_h.CData = J;
title(sprintf('Detected a %d x %d Checkerboard',boardSize));
