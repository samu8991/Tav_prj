clc;
clear all;
close all;
%% camera sensor parameters
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

%% define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 8;  % all other distance quantities are also in meters
bottomOffset      = 6;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
outImageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, outImageSize);

%videoReader = VideoReader('straightRoad.mp4');
videoReader = VideoReader('driftLeft.mp4');
%videoReader = VideoReader('custom.mp4');

%% process video frame by frame
while hasFrame(videoReader)
    
    frame = readFrame(videoReader); % get the next video frame
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);

    [h,w] = size(birdsEyeImage);

%     %do image binarization
%     regionA = cast(max(birdsEyeImage,[], 'all'), 'double');
%     regionB = cast(min(birdsEyeImage,[], 'all'),'double');
%     
%     th_new = cast((regionA+regionB)/2, 'double');
%     th_old = cast(1e3, 'double');
% 
%     %find threshold using iterative process
%     while abs(th_new-th_old) > 1e-2
%        th_old = th_new;
%     
%        A = birdsEyeImage(birdsEyeImage>th_old);
%         regionA = sum(A)/numel(A);
% 
%         B = birdsEyeImage(birdsEyeImage<=th_old);
%         regionB = sum(B)/numel(B);
% 
%         th_new = (regionA+regionB)/2;
%     end
% 
%     %build binary image
%     
%     for i = 1:h
%        for j = 1:w      % consider only the current lane
%            if birdsEyeImage(i,j) <= th_new
%                binaryImage(i,j)=0;
%             else
%                binaryImage(i,j)=255;
%             end
%         end
%     end
   binaryImage = im2bw(birdsEyeImage, 0.5);
    %% do lane recognition
    % Single window
%     histogram = zeros(1,w);
%     for i = 1:h
%         for j = 1:w    
%             if binaryImage(i,j) ~= 0
%                 histogram(1,j) = histogram(1,j)+1; 
%             end 
%         end
%     end
    
    
    % find the peak correspondings to lane
  
%     [value, left_lane] = max(histogram(w/2-40:w/2));
%     leftx_lane = value;
% 
%     [value, right_lane] = max(histogram(w/2:w/2+40));
%     rightx_lane = value;
%     
% 
%     left_lane = left_lane+w/2-40;
%     right_lane = right_lane+w/2;
    
    nwindow = 15;

    w_height = floor(h/nwindow);

    [nonzerox, nonzeroy] = nonzero(binaryImage, w, h);

%     leftx_current = left_lane;
%     rightx_current = right_lane;

    margin = 20;

    left_lane_inds = [];
    right_lane_inds = [];

    w_y_low = 1;
    w_y_hight = w_height;

    midpoint = floor(w/2);

    minpixel = 50;
histogram = zeros(1,w);
    for window = 1 : nwindow
        
         for i = 1:w_y_hight
        for j = 1:w    
            if binaryImage(i,j) ~= 0
                histogram(1,j) = histogram(1,j)+1; 
            end 
        end
         end

    [value, left_lane] = max(histogram(w/2-40:w/2));
    leftx_lane = value;

    [value, right_lane] = max(histogram(w/2:w/2+40));
    rightx_lane = value;
    

    left_lane = left_lane+w/2-40;
    right_lane = right_lane+w/2;

    left_lane_inds = cat(2, left_lane_inds, [left_lane, w_y_low].');
    %right_lane_inds = cat(1, right_lane_inds, right_lane);

            w_y_low = w_y_hight;
        w_y_hight = w_y_hight + w_height;

        if(w_y_hight > h) 
            w_y_hight = h;
        end

%         w_xleft_low = left_lane - margin;
%         w_xleft_high = left_lane + margin;
%         w_xright_low = left_lane - margin;
%         w_xright_high = left_lane + margin;
% 
%         histogram = zeros(1,w);
%   
%         good_left_inds = selectRegionIndex(nonzerox, nonzeroy, w_xleft_low, w_xleft_high, w_y_low, w_y_hight);
%         good_right_inds = selectRegionIndex(nonzerox, nonzeroy, w_xright_low, w_xright_high, w_y_low, w_y_hight);
% 
%         left_lane_inds = cat(2, left_lane_inds, good_right_inds);
%         right_lane_inds = cat(2, right_lane_inds, right_lane_inds);
% 
%         w_y_low = w_y_hight;
%         w_y_hight = w_y_hight + w_height;
% 
%         if(w_y_hight > h) 
%             w_y_hight = h;
%         end
% 
% %         i = size(good_left_inds);
% %         if(i > minpixel)
% %             leftx_current = floor(mean(nonzerox(good_left_inds)));
% %         end
% %         i = size(good_right_inds);
% %         if(i > minpixel)
% %             rightx_current = floor(mean(nonzerox(good_right_inds)));
% %         end
    end

    figure(4);
    plot(left_lane_inds(1, :), left_lane_inds(2, :), 'o');

    left_lane = polyfit(left_lane_inds(2, :), left_lane_inds(1, :), 2);
    f_left_lane = polyval(left_lane, linspace(0, w));
    %plot(linspace(0, w), f_left_lane);
    %plot(righty, rightx, 'o')

%     leftx = nonzerox(left_lane_inds);
%     lefty = nonzeroy(left_lane_inds);
%     rightx = nonzerox(right_lane_inds);
%     righty = nonzerox(right_lane_inds);
% 
%     [left_curverad, right_curverad] = calc_curve(leftx, lefty, rightx, righty, w, h);
%     
%     
%     left_fit = polyfit(leftx, lefty, 2);
%     x = linspace(0, w, 3);
%     f_left_fit = polyval(x, left_fit);
% 
%     figure(3)
%     plot(x, f_left_fit)
    %plot(left_fit, 'o')
    %plot(righty, rightx, 'o')

    % display lanes over the bird's eye view
    lanes = zeros(h,w);
%     [i,j] = size(left_lane_inds);
%     if((i == 2) && (j >= 2) )
%     %lanes(left_lane_inds(2, :), left_lane_inds(1, :))=w;
%     figure(2)
%     plot(left_lane_inds(2, :), left_lane_inds(1, :), 'o');
%     end
    %lanes(rightx, righty)=w;
    imageOverlap = imoverlay(binaryImage,lanes,'green');
    figure(1)
    imshow(imageOverlap);
    
    
    tmp=right_lane-left_lane
end