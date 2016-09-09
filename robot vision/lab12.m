%% Setup

% add support packages on path >> pathtool
% if fails: >> restoredefaultpath; matlabrc
% Link ACM to S named ports $ sudo ln -s /dev/ttyACM0 /dev/ttyS102
s = serial('COM4', 'Baudrate', 115200);
fopen(s)

%% Interface camera and place (v4l2 driver)

vid = videoinput('winvideo', 1, 'MJPG_320x240');
vid.TriggerRepeat = Inf;
vid.FrameGrabInterval = 1; 

% src = getselectedsource(vid);
% vid_src.Tag = 'motion detection setup';

start(vid)
preview(vid)

%% Interface camera and place (webcam driver)

% use snapshot instead of getsnapshot

% camList = webcamlist
% cam = webcam(1);
% preview(cam);

%% Get frame for color segmentation app

img = getsnapshot(vid);
[imgWarp, ref] = imwarp(img, tform);
imgCrop = imcrop(imgWarp, rect);
image(imgCrop)

%% Append image to base image
img = getsnapshot(vid);
[imgWarp, ref] = imwarp(img, tform);
imgCropP = imcrop(imgWarp, rect);
image(imgCropP)

r = round(getrect());
idxi = r(2) + (0:r(4));
idxj = r(1) + (0:r(3));
imgCrop(idxi, idxj) = imgCropP(idxi, idxj);
image(imgCrop)
pause(1)
close

%% Get ROI

img = getsnapshot(vid);
image(img);
[x, y] = ginput(4);
close

factor = 1;
movingPoints = [x y];
fixedPoints = [100 100; -100 100; -100 -100; 100 -100] * factor;
transformationType = 'projective';
tform = fitgeotrans(movingPoints, fixedPoints, transformationType);
[imgWarp, ref] = imwarp(img, imref2d(size(img)), tform);

[xw,yw] = worldToIntrinsic(ref, fixedPoints(:,1), fixedPoints(:,2));
rect = [min(xw) min(yw) max(xw)-min(xw) max(yw)-min(yw)];
imgCrop = imcrop(imgWarp, rect);
image(imgCrop)

%% Main loop

figure
h = image(imgCrop);
drawnow

STEPS = 100;
k = STEPS + 1;
target = [100 100]*factor;
centroid = [100 100]*factor;
score = 0;
while true
    img = getsnapshot(vid);
    [imgWarp, ref] = imwarp(img, tform);
    imgCrop = imcrop(imgWarp, rect);
    
%     imgInt = imgCrop(:,:,1) > 200;
    imgInt = createMaskBlue(imgCrop);

    stats = regionprops(imgInt, 'Centroid');
%     notRows = [stats.Area] < 10*factor^2 | [stats.Area] > 30*factor^2;
%     stats(notRows) = [];
    centroids = reshape([stats.Centroid], [2 length(stats)]).';
    centDist = sqrt(sum(bsxfun(@plus, centroids, -0*centroid).^2, 2));
    [~, idx] = max(centDist);
    
%     [~, idx] = max([stats.Area]);

    if k > STEPS
        t0 = target;
        t1 = rand(1, 2)*200*factor;
        k = 0;
    end
    if ~isempty(idx)
        centroid = centDist(idx,:);
        reading = centroids(idx,:);
    else
        reading = [999 999]*factor;
        disp([num2str(rand) ': Laser not found!'])
    end
    score = score + 0.5 - 0.01*norm(target-reading);
    if score > 100
        score = 100;
    elseif score < 0
        score = 0;
    end
    target = t0 + (t1-t0)*k/STEPS;
    k = k + 1;
    
    readingOut = reading/factor-100;
    targetOut = target/factor-100;

    coord = [target; reading];
    coord(:,2) = 200 - coord(:,2);
    text = {sprintf('Target:\n%.1f\n%.1f', targetOut), sprintf('Reading:\n%.1f\n%.1f', readingOut)};
    box_color = {'red','green'};
    imgAnn = insertText(flipud(imgCrop), coord, text, 'BoxColor', box_color, 'FontSize', 10*factor);
    imgAnn = insertText(imgAnn, [0 0], num2str(score), 'BoxColor', 'white', 'FontSize', 10*factor);
    imgAnn = insertMarker(imgAnn, coord, 's', 'color', box_color, 'size', 5*factor);
    
    set(h, 'CData', imgAnn);
    drawnow
    str = sprintf('%f %f %f %f\n', readingOut, targetOut);
%     fprintf(s, str);
    pause(0.05)
end

%% Clean up
% Once the connection is no longer needed, clear the associated variable.

stop(vid)
% clear cam
fclose(s)