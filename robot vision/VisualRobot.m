classdef VisualRobot < handle
    %VisualRobot Commands a robot according to the visual feedback
    %   Requires a connected Arduino with the code robot_motion
    %
    %   Hints:
    %    - Close all opened serial ports and camera: fclose(instrfind), imaqreset, stop(timerfind)
    %    - Discard frames to free memory: flushdata(obj.vid)
    %
    %   How to check and set camera's available frame rates:
    %     src = getselectedsource(v)
    %     frameRates = set(src, 'FrameRate')
    %     src.FrameRate = frameRates{1};
    
    properties (Constant)
        home = [0.22 0 0.05 pi/2-0.01];
    end
    
    properties
        vid
        s
        
        res = [160 120];    % camera resolution
        T0c = [Ry(pi) [25e-2; 0; 50e-2]; 0 0 0 1]; % pose from world to camera
        
        pose_state = VisualRobot.home;
        pose_goal = VisualRobot.home;
        grip_state = 0;
        ji_goal
        ji_state
    end
    
    methods
        function obj = VisualRobot(varargin)
            
            obj.vid = videoinput('winvideo', 1, 'MJPG_160x120', ...
                'TriggerRepeat', Inf, ...
                'FrameGrabInterval', 1, ...
                'FramesPerTrigger', 1, ...
                'FramesAcquiredFcn', {@obj.frame_acquired_fcn}, ...
                'FramesAcquiredFcnCount', 1);
            
            obj.s = serial('COM4', 'Baudrate', 115200, ...
                'Terminator', 'CR/LF', ...
                'Timeout', 0.1, ...
                'BytesAvailableFcnMode', 'terminator', ...
                'BytesAvailableFcn', {@obj.serial_data_available});

            % assign custom options
            for k = 1 : 2 : (nargin-3)
                switch varargin{k}
                    case properties(obj)
                        obj.(varargin{k}) = varargin{k+1};
                        
                    otherwise
                        warning(['Unknown input parameter: ' varargin{k}]);
                end
            end
            
            obj.ji_state = obj.res / 2;
            obj.ji_goal = obj.res / 2;
       
%             start(obj.vid)
            fopen(obj.s);
            while ~strcmp(obj.s.Status, 'open')
                % wait
            end
%             start(obj.motion_timer)

            pause(3) % wait for Arduino to setup
            obj.move(obj.pose_state, obj.grip_state);
        end
        
        function manual(obj)
            
            home = [0.2 0 0.08 pi/2-0.05];
            pos = home;
            obj.move(pos, 0);
            gstate = 0;
            
            dt = 0.1;
            h = imshow(obj.getsnapshot);

            while isvalid(h)
                tic
                set(h, 'CData', obj.getsnapshot);

                [x, y, b] = ginput(1);

                switch b
                    case 1,     % navigate
                        v = 5e-1 * ([x y] - obj.res/2) ./ obj.res/2;
                        v(1) = -v(1);

                        pos(1:2) = pos(1:2) + v * dt;
                        if norm(pos - home, 2) > 0.1
                            pos(1:2) = pos(1:2) - v * dt;
                        end

                    case 2,     % return home
                        pos = home;

                    case 3,     % grab/release object
                        pos(3) = 0.01;
                        obj.move(pos, gstate);
                        pause(0.5)

                        gstate = 100 * ~gstate;
                        obj.move(pos, gstate);
                        pause(1)

                        pos(3) = home(3);
                end

                obj.move(pos, gstate);
                pause(dt - toc)
            end
        end
        
        function serial_data_available(obj, src, event)
            disp(fgets(src))
        end

        function preview(obj)
            preview(obj.vid)
        end
        
        
        function move(obj, pose, grip)
            
            persistent q
            if isempty(q)
                q = zeros(4, 1);    % save previous joint values for fast ik
            end
            tol = 1e-1;
            
            % constrain to bounds
            bounds = [  0.1     -0.15   0       -pi/2+0.005
                        0.35    0.15    0.1    pi/2-0.005];
            if any(pose < bounds(1,:)) || any(pose > bounds(2,:))
                pose(pose < bounds(1,:)) = bounds(1,pose < bounds(1,:));
                pose(pose > bounds(2,:)) = bounds(2,pose > bounds(2,:));
                warning(['Pose out of bounds. Constraining to ' num2str(pose)])
            end
            
            % solve ik and actuate if solution exists
            [qp, dev] = ik(pose(:), q(:) * 0.9);
            if norm(dev, 2) < tol && all(~isnan(dev))
                str = sprintf('%f %f %f %f %f', qp, grip);
                q = qp;
                fprintf(obj.s, str);
            end
        end
        
        function img = getsnapshot(obj)
            img = getsnapshot(obj.vid);
        end
        
        function [target, success] = search_by_color(obj, HSV, props)
            % props: hsvmin, hsvmax, marea

            BW = (HSV(:,:,1) >= props.hsvmin(1) ) & (HSV(:,:,1) <= props.hsvmax(1)) & ...
                (HSV(:,:,2) >= props.hsvmin(2) ) & (HSV(:,:,2) <= props.hsvmax(2)) & ...
                (HSV(:,:,3) >= props.hsvmin(3) ) & (HSV(:,:,3) <= props.hsvmax(3));

            BWclose = imclose(BW, strel('sphere', round(5e-3*props.marea)));      % fill gaps
%             BWopen = imopen(BWclose, strel('sphere', round(12e-3*props.marea)));   % remove small blobs
            
            target = regionprops(BWclose, 'Area', 'Centroid');
            success = ~isempty(target);
            if success
                [~, idx] = min(abs([target.Area] - props.marea));
                target = target(idx);
            end
              
            %{
            subplot(221), imshow(BW)
            subplot(222), imshow(BWclose)
%             subplot(223), imshow(BWopen)
%             subplot(224), imshow(BWclose)
            set(gcf, 'position', [38    74   801   592])
            %}
        end          

      
        function Tcb = find_board(obj, RGB)
            % find a large square on center of image

            HSV = rgb2hsv(RGB);

            channel3Max = 0.625;
            BW = HSV(:,:,3) <= channel3Max;
            
            %%
            se = strel('square', round(prod(obj.res)/7500));
            BWclose = imclose(BW, se);
            BWopen = imopen(BWclose, se);
            
            %%
            subplot(121), imshow(BW)
            subplot(122), imshow(BWopen)
            set(gcf, 'position', [68         246        1263         420])
            %%
            stats = regionprops(BWopen, 'Area', 'BoundingBox');
            
            [maxarea, idx] = max([stats.Area]);
            if maxarea < 0.3 * prod(obj.res)
                warning('Board not found')
                bbox = [1 1 obj.res];
            else
                bbox = stats(idx).BoundingBox + [-10 -10 20 20];
            end
            BWcrop = imcrop(BWopen, bbox);

            BWedge = edge(BWcrop,'canny');
            [H,T,R] = hough(BWedge, 'RhoResolution', 3, 'Theta', -90:1:89);
            P = houghpeaks(H, 6, 'threshold', ceil(0.3*max(H(:))), 'NHoodSize', [31 71]);
            lines = houghlines(BWedge, T, R, P, 'FillGap', 10, 'MinLength', 20);
            
            %%
            figure
            imshow(H,[],'XData',T,'YData',R,...
                        'InitialMagnification','fit');
            xlabel('\theta'), ylabel('\rho');
            axis on, axis normal, hold on;

            figure, imshow(imcrop(RGB, bbox)), hold on
            max_len = 0;
            for k = 1:length(lines)
               xy = [lines(k).point1; lines(k).point2];
               plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

               % Plot beginnings and ends of lines
               plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
               plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

               % Determine the endpoints of the longest line segment
               len = norm(lines(k).point1 - lines(k).point2);
               if ( len > max_len)
                  max_len = len;
                  xy_long = xy;
               end
            end
        end
        
        function xyz = aproj(obj, ji, d)
            % inverse camera projection function, given a guess d
            
            f = 97 * 53e-2 / 25e-2;
            ji(:,1) = ji(:,1) - obj.res(1)/2;
            ji(:,2) = ji(:,2) - obj.res(2)/2;
            xyz = [ji*d/f ones(size(ji, 1), 1)*d];            
        end
        
        function [target, success] = find_self(obj, HSV)
            % find itself in the image frame
            
            purpleprops = struct('hsvmin', [0.759 0.153  0.478], 'hsvmax', [0.867 0.668 1], 'marea', 380);
            [target, success] = obj.search_by_color(HSV, purpleprops);
        end
        
        function [target, success] = find_target(obj, HSV)
            % find itself in the image frame
            
            blueprops = struct('hsvmin', [0.351 0 0], 'hsvmax', [0.739 1 1], 'marea', 180);
            [target, success] = obj.search_by_color(HSV, blueprops);
            if success
                success = abs(target.Area - 180) < 180/5;
            end
        end
        
        function frame_acquired_fcn(obj, hvid, event, varargin)
            
            % acquire image
            RGB = getdata(hvid);
            
            % find self and update position state
            HSV = rgb2hsv(RGB);
            [self, success] = obj.find_self(HSV);
            pose_state = obj.pose_state;
            if success
                obj.ji_state = self.Centroid;
                
                dself = 35e-2;
                xyzself = [obj.aproj(self.Centroid, dself) 1] * obj.T0c';
                pose_state(1:2) = 0.0 * xyzself(1:2) + 1 * obj.pose_state(1:2);
            end
            
            % find target and update goal state
            [target, success] = obj.find_target(HSV);
            if success
                dtarget = 50e-2;
                xyztarget = [obj.aproj(target.Centroid, dtarget) 1] * obj.T0c';
                obj.pose_goal = [xyztarget(1:2) obj.pose_goal(3:4)]; % kf here
            end
            
            % move to target ji
%             J = [-1.7020e-03 0; 0 1.7020e-03];
            error = obj.pose_goal - pose_state;
            dpose = 0.1 * error;
            obj.pose_state = pose_state + dpose;
            obj.move(obj.pose_state, obj.grip_state)
            
            % update image
            h = get(gca, 'Children');
            set(h, 'CData', RGB)
        end
        
        function click_motion(obj)
            
            obj.start
            while true
                
                try
                    [x, y, b] = ginput(1);
                    
                    if b == 1
                        obj.ji_goal = [x y];
                    else
                        break
                    end
                catch ME
                    break
                end               
            end
            obj.stop
        end
        
        function [t, traj_ji, meas_ji] = auto_motion(obj)
            
            dt = 1/30;
            t = (0 : dt : 20)';
            traj_ji = bsxfun(@plus, bsxfun(@times, sin(2*pi*0.3*t), 40*[1 -1]), obj.res/2);
            traj_ji(301:400,:) = repmat(obj.res/2, [100 1]);
            meas_ji = zeros(size(traj_ji));
            
            obj.ji_goal = traj_ji(1,:);
            obj.start
            for k = 1 : length(t)
                tic
                obj.ji_goal = traj_ji(k,:);
                meas_ji(k,:) = obj.ji_state;
                pause(dt - toc)
            end
            obj.stop
            
            figure(2)
            plot(t, traj_ji, '--')
            hold on, set(gca, 'ColorOrderIndex', 1)
            plot(t, meas_ji)
        end
        
        function start(obj)
            obj.pose_goal = VisualRobot.home;
            figure
            imshow(obj.getsnapshot)
            start(obj.vid)
            pause(2)
        end
        
        function stop(obj)
            stop(obj.vid)
        end
            
        
        function delete(obj)
            stop(obj.vid)
            fclose(obj.s);
            disp('Stoping timers, camera and serial accesses')
        end
    end
    
    methods (Static)

        function [x, y, th] = find_rectangle(BW)
            
            if nargin == 0
                BW = any(insertShape(uint8(zeros(240, 320)), 'FilledPolygon', [10 50 300 20 310 230 60 200], 'Color', 'white') > 0, 3);
            end
            
                
%%
            tic
            BWedge = edge(BW,'canny');
            [H,T,R] = hough(BWedge);
            P  = houghpeaks(H, 8, 'threshold', ceil(0.3*max(H(:))), 'NHoodSize', [11 11]);
            lines = houghlines(BWedge, T, R, P, 'FillGap', 10, 'MinLength', 20);
            toc
            
            imshow(BWedge)
            
            %% plots ----------------------------------------------
            figure
            imshow(H,[],'XData',T,'YData',R,...
                        'InitialMagnification','fit');
            xlabel('\theta'), ylabel('\rho');
            axis on, axis normal, hold on;

            figure, imshow(BW), hold on
            max_len = 0;
            for k = 1:length(lines)
               xy = [lines(k).point1; lines(k).point2];
               plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

               % Plot beginnings and ends of lines
               plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
               plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

               % Determine the endpoints of the longest line segment
               len = norm(lines(k).point1 - lines(k).point2);
               if ( len > max_len)
                  max_len = len;
                  xy_long = xy;
               end
            end
            

        end
        

        function test
            %%
            dt = 0.1;
            home = [0.2 0 0.05 pi/2-0.05];
            purpleprops = struct('hsvmin', [0.759 0.153  0.478], 'hsvmax', [0.867 0.668 1], 'marea', 380);
            
            v.move(home, 50);
            pause(1)
            
            t = (0 : dt : 20)';
            posxy = bsxfun(@plus, bsxfun(@times, sin(2*pi*0.2*t), 7e-2*[1 -1]), home(1:2));
            posxy(51:70,:) = repmat(home(1:2), [20 1]);
            posij = zeros(size(posxy));
            tr = zeros(size(t));
            
            t0 = tic;
            for k = 1 : size(posxy, 1)
                tic
                
                [target, success] = v.search_by_color(rgb2hsv(v.getsnapshot), purpleprops);
                if success
                    posij(k,:) = target.Centroid;
                    tr(k) = toc(t0);
                end
                
                pose = [posxy(k,:) home(3:4)];
                v.move(pose, 50);
                pause(dt - toc)
            end
         
            %%
            disp('Real and expected fps: ')
            [1/mean(diff(tr)) 1/dt]
            
            tposc = v.aproj(posij, 30e-2);    % target position in camera frame
            tpos0 = [tposc ones(size(tposc,1), 1)] * v.T0c'; % in world frame
                    
            figure, plot(t, tpos0(:,1:2), '--')
            hold on, set(gca, 'ColorOrderIndex', 1)
            plot(t, posxy)
            
        end
    end
    
end

