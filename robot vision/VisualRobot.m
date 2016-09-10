classdef VisualRobot < handle
    %VisualRobot Commands a robot according to the visual feedback
    %   Requires a connected Arduino with the code robot_motion
    %
    %   Hints:
    %    - Close all opened serial ports and camera: fclose(instrfind), imaqreset
    %    - Discard frames to free memory: flushdata(obj.vid)
    
    properties
        vid = videoinput('winvideo', 1, 'MJPG_320x240', 'TriggerRepeat', Inf, 'FrameGrabInterval', 1);
        s = serial('COM4', 'Baudrate', 115200, 'Terminator', 'CR/LF', 'Timeout', 0.1);
        
        res = [320 240];
    end
    
    methods
        function obj = VisualRobot(varargin)

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
            
            if ~isvalid(obj.vid)
                warning('Video was invalid')
                obj.vid = videoinput('winvideo', 1, 'MJPG_320x240', 'TriggerRepeat', Inf, 'FrameGrabInterval', 10);
            end
            start(obj.vid)
            fopen(obj.s);
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
                q = zeros(4, 1);
            end
            tol = 1e-1;
            
            [qp, dev] = ik(pose(:), q(:) * 0.8);
            if norm(dev, 2) < tol && all(~isnan(dev))
                str = sprintf('%f %f %f %f %f', qp, grip);
                q = qp;
                fprintf(obj.s, str);
            end
        end
        
        function img = getsnapshot(obj)
            img = getsnapshot(obj.vid);
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
        
        function delete(obj)
            stop(obj.vid)
            fclose(obj.s);
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
            
            v = VisualRobot();
            
            home = [0.25 0 0.05 pi/2-0.05];
            posxy = home(1:2);
            dt = 0.05;
            
            v.move(home, 50);
            
            velxy = 5e-2/1 * [1 0];
            t0 = tic;
            while toc(t0) < 10
                tic
                posxy = posxy + dt * velxy;
                if norm(posxy - home(1:2), 2) > 0.08
                    velxy = -velxy;
                end
                norm(posxy - home(1:2), 2)
                pose = [posxy home(3:4)];
                v.move(pose, 50);
                pause(dt - toc)
                tic
            end
            
            
        end
    end
    
end

