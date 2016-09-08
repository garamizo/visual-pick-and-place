classdef VisualRobot < handle
    %VisualRobot Commands a robot according to the visual feedback
    %   Requires a connected Arduino with the code robot_motion
    %
    %   Hints:
    %    - Close all opened serial ports: clear(instrfind)
    
    properties
        s = serial('COM4', 'Baudrate', 115200, 'Terminator', 'CR/LF', 'Timeout', 0.1);
        vid = videoinput('winvideo', 1, 'MJPG_320x240');
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
        
        function delete(obj)
            stop(obj.vid)
            fclose(obj.s);
        end
    end
    
end

