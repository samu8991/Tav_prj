clc;
clear all;
close all;

%% Camera sensor parameters
camera = struct('ImageSize',[480 640],'PrincipalPoint',[320 240],...
                'FocalLength',[320 320],'Position',[1.8750 0 1.2000],...
                'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
                'LaneDetectionRanges',[6 30],'DetectionRanges',[6 50],...
                'MeasurementNoise',diag([6,1,1]));
focalLength    = camera.FocalLength;
principalPoint = camera.PrincipalPoint;
imageSize      = camera.ImageSize;
% mounting height in meters from the ground
height         = camera.Position(3);  
% pitch of the camera in degrees
pitch          = camera.Rotation(2);  
            
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch);


%% Define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 8;  % all other distance quantities are also in meters
bottomOffset      = 6;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
outImageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, outImageSize);


%% Select video
%videoReader = VideoReader('straightRoad.mp4');
videoReader = VideoReader('driftLeft.mp4');
%videoReader = VideoReader('curvedRoad.mp4');
%videoReader = VideoReader('custom.mp4');

% f1 = figure('Name', 'Left Lane Points');
% f2 = figure('Name', 'Right Lane Points');
% f2.Position(1:2) = [1620, 918];
f3 = figure('Name', 'Left Fit');
f4 = figure('Name', 'Right Fit');
f5 = figure('Name', 'Street video');
f5.Position(1:2) = [0 400];
f6 = figure('Name', 'Histogram');

%% Process video frame by frame
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



    %% Lane Detection
    y_start = 340; %to avoid car nose noise at the bottom of the frame
    y_end = 0;
    margin = 70;

    nwindow = 15;
    w_height = floor((y_start-y_end)/nwindow); %single window height

    %[nonzerox, nonzeroy] = nonzero(binaryImage, w, h);

    left_lane_inds = [];
    right_lane_inds = [];

    w_y_low = y_start;
    w_y_height = w_y_low - w_height;

    
    for window = 1 : nwindow
        histogram = zeros(1,w);
        for i = w_y_height:w_y_low
            %for j = w/2-margin:w/2+margin
            for j = 1:w
                if binaryImage(i,j) ~= 0
                    histogram(1,j) = histogram(1,j)+1; 
                end 
            end
         end

        [value, left_lane] = max(histogram(w/2-margin:w/2));
        leftx_lane = value;
    
        [value, right_lane] = max(histogram(w/2:w/2+margin));
        rightx_lane = value;

        set(0, 'CurrentFigure', f6)
        plot(histogram)

        left_lane = left_lane+w/2-margin;
        right_lane = right_lane+(w/2);

        %check for windows with blank space of dashed lane lines
        if(leftx_lane == 0)
            if(isempty(left_lane_inds))
                left_lane = w/2+margin-40;
            else
                left_lane = left_lane_inds(1, end);
            end
        end
        if(rightx_lane == 0)
            if(isempty(right_lane_inds))
                right_lane = w/2+margin-40;
            else
                right_lane = right_lane_inds(1, end);
            end
        end
    
        left_lane_inds = cat(2, left_lane_inds, [left_lane, w_y_height].');  % .' ??
        right_lane_inds = cat(2, right_lane_inds, [right_lane, w_y_height].');
    
        w_y_low = w_y_height; %update new window values
        w_y_height = w_y_height - w_height; %update new window values
    
        if(w_y_height < y_end) 
            w_y_height = y_end;
        end
    end

%     set(0, 'CurrentFigure', f1)
%     plot(left_lane_inds(1, :), left_lane_inds(2, :), 'o');
% 
%     set(0, 'CurrentFigure', f2)
%     plot(right_lane_inds(1, :), right_lane_inds(2, :), 'o');

    left_lane = polyfit(left_lane_inds(2, :), left_lane_inds(1, :), 3);
    f_left_lane = polyval(left_lane, linspace(min(left_lane_inds(2,:)), max(left_lane_inds(2,:)), 250));
    set(0, 'CurrentFigure', f3)
    plot(f_left_lane, linspace(0, w, 250))
    xlim([0 250])
    

    right_lane = polyfit(right_lane_inds(2, :), right_lane_inds(1, :), 3);
    f_right_lane = polyval(right_lane, linspace(min(right_lane_inds(2, :)), max(right_lane_inds(2, :)), 250));
    set(0, 'CurrentFigure', f4)
    plot(f_right_lane, linspace(0, w, 250))
    xlim([0 250])
    

    % display lanes over the bird's eye view
    lanes = zeros(h,w);
    %option 1
    setx_left = cast(linspace(min(left_lane_inds(2,:)), max(left_lane_inds(2,:)), 250), 'uint16');
    sety_left = cast(f_left_lane, 'uint16');
    setx_right = cast(linspace(min(right_lane_inds(2, :)), max(right_lane_inds(2, :)), 250), 'uint16');
    sety_right = cast(f_right_lane, 'uint16');
    for i = 1:w
        lanes(setx_right(i), sety_right(i)) = w;
        lanes(setx_left(i), sety_left(i)) = w;
    end

    %option 2
%     lanes(:, cast(f_left_lane, 'uint16')) = w;
%     lanes(:, cast(f_right_lane, 'uint16')) = w;

    %lane departure warning
    center = w/2;
    if(abs(left_lane_inds(1, 3)-center) < 20 || abs(right_lane_inds(1, 3)-center) < 20)
        fprintf("Warning: lane crossing! \n") %activate warning if the car is about to cross a lane -> 20 is just an estimate
    end

    %show image overlap
    imageOverlap = imoverlay(binaryImage, lanes, 'red');
    set(0, 'CurrentFigure', f5)
    imshow(imageOverlap);
   
    %lane width
    tmp = right_lane_inds(1, 3) - left_lane_inds(1, 3)
end