classdef VisualRobot < handle
    %VisualRobot Commands a robot according to the visual feedback
    %   Requires a connected Arduino with the code robot_motion
    %
    %   Hints:
    %    - Close all opened serial ports: fclose(instrfind)
    %    - Close all cameras: imaqreset
    %    - Discard frames to free memory: flushdata(obj.vid)
    
    properties
        s = serial('COM4', 'Baudrate', 115200, 'Terminator', 'CR/LF', 'Timeout', 0.1);
        vid = videoinput('winvideo', 1, 'MJPG_320x240', 'TriggerRepeat', Inf, 'FrameGrabInterval', 100);
        
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
            
            fopen(obj.s);
            start(obj.vid)
        end

        function preview(obj)
            preview(obj.vid)
        end
        
        function success = move(obj, pose, grip)
            str = sprintf('%f %f %f %f %f', pose, grip);
            fprintf(obj.s, str);
            
            [msg, count] = fscanf(obj.s, '%s');
            if count == 0
                warning('Serial link is not responding')
                warning(msg)
            elseif ~strcmp(msg, 'Success')
                warning(['Pose ' num2str(pose) ' out of reach'])
            else
                success = true;
                return
            end
            success = false;
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
            
            home = [0.2 0 0.1 pi/2];
            posxy = home(1:2);
            dt = 0.1;
            
            v.move(home, 50);
            
            velxy = 10e-2/1 * [1 0];
            t0 = tic;
            while toc(t0) < 10
                tic
                posxy = posxy + dt * velxy;
                if norm(posxy - home(1:2), 2) > 0.05
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

