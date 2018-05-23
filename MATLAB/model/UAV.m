
classdef UAV < handle
    
    properties
        
        curr_pos = [0 0];
        dest_pos = [0 0];
        
        footprint_size = 1;
        
        heading = 0;
        radius = 1;
        
        update_time = .1;
        velocity = 1;
        
        flying = false;
    end
        
    properties (SetAccess = private)
          path = [];
          path_i = 0;
    end
   
    methods
        % class constructor
        function uav_obj = UAV(start_pos, start_heading, footprint_size, radius, velocity, update_time)
            if nargin > 0
                % set the starting position if an initial one is given
                if size(start_pos) == size(uav_obj.curr_pos)
                    if isnumeric(start_pos)
                        uav_obj.curr_pos = start_pos;
                    end
                end                
            end
            
            if nargin > 3
                % set the start_heading size if one is given
                if size(start_heading) == size(uav_obj.heading)
                    if isnumeric(start_heading)
                        uav_obj.heading = start_heading;
                    end
                end                
            end
            
            if nargin > 2
                % set the footprint size if one is given
                if size(footprint_size) == size(uav_obj.footprint_size)
                    if isnumeric(footprint_size)
                        uav_obj.footprint_size = footprint_size;
                    end
                end                
            end
            
            if nargin > 3
                % set the radius size if one is given
                if size(radius) == size(uav_obj.radius)
                    if isnumeric(radius)
                        uav_obj.radius = radius;
                    end
                end                
            end
            
            if nargin > 4
                % set the velocity size if one is given
                if size(velocity) == size(uav_obj.velocity)
                    if isnumeric(velocity)
                        uav_obj.velocity = velocity;
                    end
                end                
            end
            
            if nargin > 5
                % set the update_time size if one is given
                if size(update_time) == size(uav_obj.update_time)
                    if isnumeric(update_time)
                        uav_obj.update_time = update_time;
                    end
                end                
            end
            
            if nargin > 6
               error('Too many arguments given to UAV constructor class') 
            end
            
        end % end constructor
        
        function start_flight(uav_obj)
            uav_obj.flying = true;
        end
        
        function stop_flight(uav_obj)
            uav_obj.flying = false;
            uav_obj.path_i = 0;
            uav_obj.path = [];
            
        end
        
        function set_veloctiy(uav_obj, new_velocity)
            if isnumeric(new_velocity) && new_velocity > 0
                uav_obj.velocity = new_velocity;
            end
        end
        
        function set_radius(uav_obj, new_radius)
            if isnumeric(new_radius) && new_radius > 0
                uav_obj.radius = new_radius;
            end
        end
        
        function set_heading(uav_obj, new_heading)
            if isnumeric(new_heading)
                uav_obj.heading = new_heading;
            end
        end
        
        function set_destination(uav_obj, new_dest)
            if size(new_dest) == size(uav_obj.dest_pos)
                uav_obj.dest_pos = new_dest;
                
                pointA = [ uav_obj.curr_pos, degtorad(uav_obj.heading) ]; 
                
                end_heading = atan2d(new_dest(2)-uav_obj.curr_pos(2), new_dest(1)-uav_obj.curr_pos(1));
                pointB = [ uav_obj.dest_pos, degtorad(end_heading)];   
                
                stepsize = uav_obj.velocity * uav_obj.update_time;  
                
                uav_obj.path = dubins_curve(pointA, pointB, uav_obj.radius, stepsize, true);
                uav_obj.path_i = 1;
                
                uav_obj.curr_pos = uav_obj.path(uav_obj.path_i, 1:2);
                uav_obj.heading = radtodeg(uav_obj.path(uav_obj.path_i, 3));
                
            end
        end
        
        function update(uav_obj)
            if uav_obj.flying 
                if uav_obj.path_i > 0
                    uav_obj.path_i = uav_obj.path_i + 1;
                    
                    if uav_obj.path_i <= size(uav_obj.path, 1)
                        uav_obj.curr_pos = uav_obj.path(uav_obj.path_i, 1:2);
                        uav_obj.heading = radtodeg(uav_obj.path(uav_obj.path_i, 3));
                    else
                        uav_obj.path_i = 0;
                        uav_obj.path = [];
                    end
                else
                    uav_obj.set_destination([uav_obj.curr_pos + [cosd(uav_obj.heading) sind(uav_obj.heading)]]);
                end

            end
        end
        
    end
end

