classdef VisualRobot < handle
    %VisualRobot Commands a robot according to the visual feedback
    %   Requires a connected Arduino with the code robot_motion
    %
    %   Hints:
    %    - Close all opened serial ports and camera: fclose(instrfind), imaqreset, stop(timerfind)
    %    - Discard frames to free memory: flushdata(obj.vid)
    
    properties
        vid
        s
        
        motion_timer
        
        res = [160 120];    % camera resolution
        T0c = [Ry(pi) [25e-2; 0; 50e-2]; 0 0 0 1]; % pose from world to camera
        
        goal_pose = [0.2 0 0.1 pi/2-0.05];
        goal_grip = 0;
    end
    
    methods
        function obj = VisualRobot(varargin)
            
            obj.vid = videoinput('winvideo', 1, 'MJPG_160x120', 'TriggerRepeat', Inf, 'FrameGrabInterval', 1);
            obj.s = serial('COM4', 'Baudrate', 115200, 'Terminator', 'CR/LF', 'Timeout', 0.1);

            % assign custom options
            for k = 1 : 2 : (nargin-3)
                switch varargin{k}
                    case properties(obj)
                        obj.(varargin{k}) = varargin{k+1};
                        
                    otherwise
                        warning(['Unknown input parameter: ' varargin{k}]);
                end
            end
            
            % enable serial data available callback
            obj.s.BytesAvailableFcnMode = 'terminator';
            obj.s.BytesAvailableFcn = {@obj.serial_data_available};
            
%             % create motion timer
%             obj.motion_timer = timer(...
%                 'ExecutionMode', 'fixedRate', ...   % Run timer repeatedly
%                 'Period', 1/20, ...                % Initial period is 1 sec.
%                 'TimerFcn', {@obj.update_motion}); % Specify callback
            
            start(obj.vid)
            fopen(obj.s);
%             start(obj.motion_timer)
        end
        
        function update_motion(obj, event, self)
            
            persistent statev b
            if isempty(statev)
                statev = repmat([obj.goal_pose obj.goal_grip], [5 1]);
                b = fir1(5, 0.5*1/(1/obj.motion_timer.Period));
            end
            
            next_state = b * [obj.goal_pose obj.goal_grip; statev];
            statev = [obj.goal_pose obj.goal_grip; statev(1:end-1,:)];
            obj.command_motion(next_state(1:end-1), next_state(end));
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
            obj.command_motion(pose, grip);

%             obj.goal_pose = pose;
%             obj.goal_grip = grip;

        end
        
        function command_motion(obj, pose, grip)
            
            persistent q
            if isempty(q)
                q = zeros(4, 1);
            end
            tol = 1e-1;
            
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
        
        function click_motion(obj)

            figure, h = imshow(obj.getsnapshot);
            set(gcf, 'position', [175   104   854   556])
            
            timerplot = timer(...
                'ExecutionMode', 'fixedRate', ...   % Run timer repeatedly
                'Period', 0.52, ...                % Initial period is 1 sec.
                'TimerFcn', {@(self, event, obj) set(h, 'CData', obj.getsnapshot), obj}); % Specify callback
            
            timerctl = timer(...
                'ExecutionMode', 'fixedRate', ...   % Run timer repeatedly
                'Period', 0.05, ...                % Initial period is 1 sec.
                'TimerFcn', {@update_control, obj}); % Specify callback

            purpleprops = struct('hsvmin', [0.759 0.153  0.478], 'hsvmax', [0.867 0.668 1], 'marea', 380);
            
            home = [0.2 0 0.05 pi/2-0.05];
            
            % image error to world error
            %{
            home = [0.2 0 0.05 pi/2-0.05];
            dcam = obj.T0c(3,4)-home(3)-5e-2;
            syms xvar yvar
            J = jacobian([obj.aproj([xvar yvar], dcam) 1] * obj.T0c', [xvar yvar]);
            J = double(J(1:2,1:2));
            %}
            J = [-1.7020e-03 0; 0 1.7020e-03];
            
            obj.move(home, 0)
            pause(2)
            [target, success] = obj.search_by_color(rgb2hsv(obj.getsnapshot), purpleprops);
            x = target.Centroid(1);
            y = target.Centroid(1);
            posxy = home(1:2);
            start(timerplot), start(timerctl)
            while isvalid(h)
                
                try
                    [x, y] = ginput(1);
                catch ME
                    break
                end               
            end
            
            stop(timerplot), stop(timerctl)
            
            function update_control(self, event, obj)
                
                [target, success] = obj.search_by_color(rgb2hsv(obj.getsnapshot), purpleprops);
                if success
                    error = [x y] - target.Centroid;
                    dxy = 0.1 * error * J';
                    posxy = posxy + dxy;
                    if norm(posxy - home(1:2), 2) < 0.12
                        obj.move([posxy home(3:4)], 0)
                    end
                end 
            end
        end
        
        function delete(obj)
%             stop(obj.motion_timer)
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

