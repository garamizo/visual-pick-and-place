classdef VisualRobot < handle
    %VisualRobot Commands a robot according to the visual feedback
    %   Requires a connected Arduino with the code robot_motion
    %
    %   Hints:
    %    - Close all opened serial ports and camera: fclose(instrfind), imaqreset
    %    - Discard frames to free memory: flushdata(obj.vid)
    
    properties
        vid
        s
        
        res = [320 240];    % camera resolution
        T0c = [Ry(pi) [20e-2; 0; 50e-2]; 0 0 0 1]; % pose from world to camera
    end
    
    methods
        function obj = VisualRobot(varargin)
            
            obj.vid = videoinput('winvideo', 1, 'MJPG_320x240', 'TriggerRepeat', Inf, 'FrameGrabInterval', 1);
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

        function v0 = convert(obj, vim)
            % convert velocity in image to world
            % change direction and scale
            
            % vim [pixels/sec]
            % v0 [m/sec]
            
            v0 = [-vim(:,1) vim(:,2)] * (25e-2/200);
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
            %%
            dt = 0.2;
            home = [0.2 0 0.1 pi/2-0.05];
            purpleprops = struct('hsvmin', [0.759 0.153  0.478], 'hsvmax', [0.867 0.668 1], 'marea', 1000);
            
            v.move(home, 50);
            pause(1)
            
            t = (0 : dt : 10)';
            posxy = bsxfun(@plus, bsxfun(@times, sin(2*pi*0.2*t), 7e-2*[1 -1]), home(1:2));
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
%             figure; plot(vlog(:,1:2)); hold on; set(gca, 'ColorOrderIndex', 1); plot(vlog(:,3:4), '--'); hold off

%%
            subplot(211), plot(posxy)
            subplot(212), plot(posij)
%%
            velxyc = v.convert(bsxfun(@ldivide, diff(posij), diff(tr)));
%             posxyc = bsxfun(@plus, posxy(1,:), [0 0; posxyc]);
            figure; plot(t(2:end), diff(posxy)/dt)
            hold on, set(gca, 'ColorOrderIndex', 1)
            plot(tr(2:end), velxyc, '--'); hold off
            
        end
    end
    
end

