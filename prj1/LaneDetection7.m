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
height         = camera.Position(3);  % mounting height in meters from the ground
pitch          = camera.Rotation(2);  % pitch of the camera in degrees         
camIntrinsics  = cameraIntrinsics(focalLength, principalPoint, imageSize);
sensor         = monoCamera(camIntrinsics, height, 'Pitch', pitch);

%% Define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 8;  % all other distance quantities are also in meters
bottomOffset      = 6;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
outImageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, outImageSize);

%% Select video
%videoReader = VideoReader('straightRoad.mp4');
%videoReader = VideoReader('driftLeft.mp4');
videoReader = VideoReader('curvedRoad.mp4');
%videoReader = VideoReader('curvedRoadSteer.mp4');
%videoReader = VideoReader('custom.mp4');

%% Figures definition
% f1 = figure('Name', 'Left Lane Points');
% f2 = figure('Name', 'Right Lane Points');
% f2.Position(1:2) = [1620, 918];
f5 = figure('Name', 'Street video');
f5.Position(1:2) = [0 400];
f6 = figure('Name', 'Histogram L');
f7 = figure('Name', 'Histogram R');

%% Hypothesis
lane_width_m = 3.7;
car_width_m = 1.8;
lane_margin = 0.1;

%% Process video frame by frame

last_good_l = 1;
last_good_r = 1;
f = 0;

while hasFrame(videoReader)
    
    frame = readFrame(videoReader); % get the next video frame -> 480x640x3 vector
    f = f+1;

    if(f==1 || f==2)
        continue
    end

    birdsEyeImage = transformImage(birdsEyeConfig, frame); % func parameters: config of the transform we want to apply and frame to transform
    % the number of pixel in the transformed image is different because of the transformation -> 375x250
    % intuitively we are looking from a different perspective (top-down) so
    % not every part of the image gets into the transformed one but this is okay beacuse we only car about the road in this situation

    birdsEyeImage = rgb2gray(birdsEyeImage); % from rgb color space to grayscale
    
    [h,w] = size(birdsEyeImage); % get the dimensions of the birdsEyeView image

    %% Do image binarization
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
    y_end = 100;
    margin = 40;
    margin_short = 20;

    nwindow = 15;
    w_height = floor((y_start-y_end)/nwindow); %single window height

    left_lane_inds = [];
    right_lane_inds = [];

    w_y_low = y_start;
    w_y_height = w_y_low - w_height;

    w_start_l = w/2-margin;
    w_end_l = w/2;
    w_start_r = w/2;
    w_end_r = w/2+margin;

    if(f==3)
        last_good_l = w/2-margin;
        last_good_r = w/2+margin;
    end
    
    % sliding window strategy in the frame
    for window = 1 : nwindow
        for i = w_y_height:w_y_low
            % left window
            histogram_l = zeros(1,w);
            for j = w_start_l:w_end_l
                if binaryImage(i,j) ~= 0
                    histogram_l(1,j) = histogram_l(1,j)+1; 
                end 
            end
            % right window
            histogram_r = zeros(1,w);
            for j = w_start_r:w_end_r
                if binaryImage(i,j) ~= 0
                    histogram_r(1,j) = histogram_r(1,j)+1; 
                end 
            end
         end

        [value, left_lane] = max(histogram_l(w_start_l:w_end_l));
        leftx_lane = value;
    
        [value, right_lane] = max(histogram_r(w_start_r:w_end_r));
        rightx_lane = value;

        set(0, 'CurrentFigure', f6)
        plot(histogram_l)
        set(0, 'CurrentFigure', f7)
        plot(histogram_r)

        left_lane = cast(left_lane+w_start_l, 'double');
        right_lane = cast(right_lane+w_start_r, 'double');

        %check for windows with blank space of dashed lane lines
        if(leftx_lane == 0)
            if(isempty(left_lane_inds))
                left_lane = cast(last_good_l, 'double');
            else
                left_lane = left_lane_inds(1, end);
            end
        end
        if(rightx_lane == 0)
            if(isempty(right_lane_inds))
                right_lane = cast(last_good_r, 'double');
            else
                right_lane = right_lane_inds(1, end);
            end
        end

        %check lane overlap
        if(right_lane-left_lane < 0 || abs(right_lane-left_lane) < 50)
            last_l=0;
            last_r=0;
            if(~isempty(left_lane_inds))
                last_l = left_lane_inds(1, end);
                last_r = right_lane_inds(1, end);

                if(abs(left_lane-last_l) > abs(right_lane-last_r))
                    left_lane = last_l;
                else
                    right_lane = last_r;
                end
            else
                last_l = cast(last_good_l, 'double');
                last_r = cast(last_good_r, 'double');

                if(abs(left_lane-last_l) > abs(right_lane-last_r))
                    left_lane = last_l;
                else
                    right_lane = last_r;
                end
            end
        end
    
        % store lane points in vector
        left_lane_inds = cat(2, left_lane_inds, [left_lane, w_y_height].');  % .' ??
        right_lane_inds = cat(2, right_lane_inds, [right_lane, w_y_height].');
    
        % update window for next iteration
        w_y_low = w_y_height;
        w_y_height = w_y_height - w_height;

        w_start_l = cast(left_lane-margin_short, 'uint16');
        w_end_l = cast(left_lane+margin_short, 'uint16');
    
        w_start_r = cast(right_lane-margin_short, 'uint16');
        w_end_r = cast(right_lane+margin_short, 'uint16');

        % check window updated positions
        if(w_start_l <= 0)
            w_start_l = 1;
        end
        if(w_start_r <= 0)
            w_start_r = 1;
        end
        if(w_end_l > 250)
            w_end_l = 250;
        end
        if(w_end_r > 250)
            w_end_r = 250;
        end

        if(w_y_height < y_end) 
            w_y_height = y_end;
        end

    end

%     set(0, 'CurrentFigure', f1)
%     plot(left_lane_inds(1, :), left_lane_inds(2, :), 'o');
% 
%     set(0, 'CurrentFigure', f2)
%     plot(right_lane_inds(1, :), right_lane_inds(2, :), 'o');

    left_lane = polyfit(left_lane_inds(2, :), left_lane_inds(1, :), 2);
    f_left_lane = polyval(left_lane, linspace(min(left_lane_inds(2,:)), max(left_lane_inds(2,:)), 250));

    right_lane = polyfit(right_lane_inds(2, :), right_lane_inds(1, :), 2);
    f_right_lane = polyval(right_lane, linspace(min(right_lane_inds(2, :)), max(right_lane_inds(2, :)), 250));
    
    % display lanes over the bird's eye view
    lanes = zeros(h,w);
    
    setx_left = cast(linspace(min(left_lane_inds(2,:)), max(left_lane_inds(2,:)), 250), 'uint16');
    sety_left = cast(f_left_lane, 'uint16');
    setx_right = cast(linspace(min(right_lane_inds(2, :)), max(right_lane_inds(2, :)), 250), 'uint16');
    sety_right = cast(f_right_lane, 'uint16');
    for i = 1:w
        if(sety_right(i) == 0)
            lanes(setx_right(i), sety_right(i)+1) = w;
        else
            lanes(setx_right(i), sety_right(i)) = w;
        end
        if(sety_left(i) == 0)
            lanes(setx_left(i), sety_left(i)+1) = w;
        else
            lanes(setx_left(i), sety_left(i)) = w;
        end
    end



    %% Compute distances in pixel and meters
    center = w/2;
    lane_width_p = right_lane_inds(1, 2) - left_lane_inds(1, 2);
    curr_dist_p = min(abs(left_lane_inds(1,2)-center), abs(right_lane_inds(1,2)-center)); % current distance in pixel from closest lane delimiter (left or right)

    r_mp = lane_width_m/lane_width_p; % meter/pixel ratio
    curr_dist_m = curr_dist_p * r_mp; % current distance from car center to lane delimiter

    dist_lim_m = car_width_m/2 + lane_margin; % if car center is 100 cm from lane delimiter left or right tyres are 10 cm from lane delimiter (car width 1,8m)
    warning_th = dist_lim_m/r_mp; % warning threshold 
    
    %% Lane departure warning
    if(abs(left_lane_inds(1, 2)-center) < warning_th)
        fprintf("Warning: lane LEFT delimiter crossing! \n") %activate warning if the car is about to cross a lane
        curr_dist_m
        %beep  %uncomment to hear alarm sound when lane crossing
    end
    if(abs(right_lane_inds(1, 2)-center) < warning_th)
        fprintf("Warning: lane RIGHT delimiter crossing! \n") %activate warning if the car is about to cross a lane
        curr_dist_m
        %beep  %uncomment to hear alarm sound when lane crossing
    end

    %% Show image overlap
    imageOverlap = imoverlay(binaryImage, lanes, 'red');
    set(0, 'CurrentFigure', f5)
    imshow(imageOverlap);
   

    last_good_l = left_lane_inds(1, 3);
    last_good_r = right_lane_inds(1, 3);
end