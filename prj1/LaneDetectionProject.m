clc;
clear all;
close all;
%% camera sensor parameters
% define intrinsic and estrinsic parameters of the camera
% this structure contains camera parameters
% VGA camera with pretty low res
camera = struct('ImageSize',[480 640],'PrincipalPoint',[320 240],...
                'FocalLength',[320 320],'Position',[1.8750 0 1.2000],...
                'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
                'LaneDetectionRanges',[6 30],'DetectionRanges',[6 50],...
                'MeasurementNoise',diag([6,1,1]));
% in this script we don't do any camera calibration beacuse it's not needed since we already know instinsic and estrinsic camera parameter
% camera calibration is needed only when camera parameters are unknown
focalLength    = camera.FocalLength; % in pixels what does it mean?
principalPoint = camera.PrincipalPoint;  % principal point is the center of the image
imageSize      = camera.ImageSize;
height         = camera.Position(3);  % mounting height in meters from the ground
pitch          = camera.Rotation(2);  % pitch of the camera in degrees
            
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize); % matlab function that creates an object of the camera params
sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch); % creates the sensor object
% monoCamera is a matlab function that creates the pinhole camera model and populates the matrices -> it stores it into the sensor object
% matlab supports different kinds of camera models (in this case Monocular Camera -> the simplest)
% other examples are stereoscopic camera and gopro

%% define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 8;  % all other distance quantities are also in meters
bottomOffset      = 6;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
outImageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, outImageSize); % matlab function that creates an object with the type of transformation we want to apply to our images
% given the properties it generates the matrix for the birds eye view transformation 
% (correlation between pixels in the original image and pixels in the transformed image)
% take an image from the pov of the camera and transform it into the birds eye view

%videoReader = VideoReader('straightRoad.mp4'); % function to process a video
videoReader = VideoReader('driftLeft.mp4'); % camera position is changing and it creates issues in this algorithm 
%videoReader = VideoReader('custom.mp4'); % curves and diffrent types of road 


%%Project
left_lane_past=0;
right_lane_past=0;
offset_left = 40;
offset_right = 40;
t=0;
f1 = figure;
f2 = figure;

%% process video frame by frame
while hasFrame(videoReader)
    
    frame = readFrame(videoReader); % get the next video frame -> 480x640x3 vector
    
    birdsEyeImage = transformImage(birdsEyeConfig, frame); % func parameters: config of the transform we want to apply and frame to transform
    % the number of pixel in the transformed image is different because of the transformation -> 375x250
    % intuitively we are looking from a different perspective (top-down) so
    % not every part of the image gets into the transformed one but this is okay beacuse we only car about the road in this situation
    birdsEyeImage = rgb2gray(birdsEyeImage); % from rgb color space to grayscale
    
    [h,w] = size(birdsEyeImage); % get the dimensions of the birdsEyeView image

    %% do image binarization
    regionA = cast(max(birdsEyeImage,[], 'all'), 'double'); % get the max intensity value in the image and transform it in double since by default is uint
    regionB = cast(min(birdsEyeImage,[], 'all'),'double'); % get the min value in the image
    % we need the double or we would loose the fractional part each time we compute the mean
    
    th_new = cast((regionA+regionB)/2, 'double'); % mean of max and min of the image
    th_old = cast(1e3, 'double'); % default inizialization -> 1000 so that it will surely cause the th to be updated at the first step where we don't have a real old th

    % find threshold using iterative process
    while abs(th_new-th_old) > 1e-2  %we need to consider an error -> if the diff is > 0.01 keep on, else stop the search
        th_old = th_new;
    
        A = birdsEyeImage(birdsEyeImage>th_old); % values of all elements with higher intensity than th
        regionA = sum(A)/numel(A); % average intensity of A region

        B = birdsEyeImage(birdsEyeImage<=th_old); % values of all elements with <= intensity than th
        regionB = sum(B)/numel(B); % average intensity of B region

        th_new = (regionA+regionB)/2; % new threshold
    end

    % build binary image
    binaryImage = birdsEyeImage;
    for i = 1:h
        for j = 1:w      % consider only the current lane
            if birdsEyeImage(i,j) <= th_new
                binaryImage(i,j)=0; % if <= than th put it to the min 0
            else
                binaryImage(i,j)=255; % if > than th put it to the max 255
            end
        end
    end

    %% do lane recognition
    % Single window
    histogram = zeros(1,w); % matrix of zeros (1 row and w col)
    for i = 1:h
        for j = 1:w    
            if binaryImage(i,j) ~= 0  % if different from 0
                histogram(1,j) = histogram(1,j)+1; 
            end 
        end
    end

    % find the peak correspondings to lane
    % consider the car in the center of the image
    [value, left_lane] = max(histogram(w/2-offset_left:w/2)); % look from 40 pixels before the half of the image to the half of the image
    [value, right_lane] = max(histogram(w/2:w/2+offset_right)); % look from the half of the image to 40 pixels before the half of the image

    left_lane = left_lane+w/2-offset_left; % add the offset because max and min on the histogram considered only a part of the image and the res will not consider that there's a part of image outside on both sides
    right_lane = right_lane+w/2; % e.g. if left_lane=0 it means that the lane was found in the first position considered by the max but the max started from w/2-40 not from the leftmost part of the original image

    %%Project
    
    %Car is always in the center of the frame -> lanes are placed in different positions
    %Shift left 
    if(left_lane_past < left_lane && right_lane_past < right_lane && t>0)
        diff_left = left_lane-left_lane_past;
        diff_right = right_lane-right_lane_past;
        offset_left = offset_left - diff_left;
        offset_right = offset_right + diff_right;
    end

    %Shift right
    if(left_lane_past > left_lane && right_lane_past > right_lane && t>0)
        diff_left = left_lane_past-left_lane;
        diff_right = right_lane_past-right_lane;
        offset_left = offset_left + diff_left;
        offset_right = offset_right - diff_right;
    end

    %Compute again lane position with new offset
    [value, left_lane] = max(histogram(w/2-offset_left:w/2)); % look from 40 pixels before the half of the image to the half of the image
    [value, right_lane] = max(histogram(w/2:w/2+offset_right)); % look from the half of the image to 40 pixels before the half of the image

    left_lane = left_lane+w/2-offset_left; % add the offset because max and min on the histogram considered only a part of the image and the res will not consider that there's a part of image outside on both sides
    right_lane = right_lane+w/2; % e.g. if left_lane=0 it means that the lane was found in the first position considered by the max but the max started from w/2-40 not from the leftmost part of the original image

    center = w/2;
    if(abs(left_lane-center) < 20 || abs(right_lane-center) < 20)
        fprintf("Warning: lane crossing! \n") %activate warning if the car is about to cross a lane -> 20 is just an estimate
    end

    %update previous frame info
    left_lane_past = left_lane;
    right_lane_past = right_lane;
    t = t+1;

    % display lanes over the bird's eye view
    lanes = zeros(h,w); % create a new matrix/image with all zeros
    lanes(:,right_lane)=w; % for each row of the image but only in the column corresponding to the lane put an higher value to create the two green lines to be put on top of the other image
    lanes(:,left_lane)=w; % why w though??
    imageOverlap = imoverlay(binaryImage,lanes,'green'); % put lines image over binaryImage
    set(0, 'CurrentFigure', f1)
    imshow(imageOverlap);
    set(0, 'CurrentFigure', f2)
    plot(histogram);
    tmp=right_lane-left_lane
end

