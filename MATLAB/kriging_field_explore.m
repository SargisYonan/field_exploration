% @author Sargis S Yonan
% @date October 2018
% 
% A function to explore a field using the Kriging Method
%
% Department of Computer Engineering
% Autonomous Systems Lab
% University of California, Santa Cruz
%
% Last Revised October 2018

function [wps, pred_field_recs, var_field_recs, percentages] = kriging_field_explore(X0, field, method, max_scan_percentage, live_plot, save_image_path)


%% Vehicle/Field Dynamics Parameters

wb = waitbar(0, 'Setting up environment...');

% initial positon of the UAV
x0 = X0(1);
y0 = X0(2);

% the turning radius of the UAV
r = 0.000001;

width = field.width;

% prediction area
[xi, yi] = meshgrid(1:width);
xi = xi(:);
yi = yi(:);

%% Kriging Parameters
a = 40; % initial seed for fmincon's range search
c = 5; % initial seed for fmincon's sill search

zz_r = 1 / max_scan_percentage;
initial_waypoint = ceil([(width/2)-zz_r, (width/2)-zz_r]);

pred_field = zeros(size(field.z));
var_field = inf .* ones(size(field.z));

waitbar(.05, wb, 'Creating UAV object...');

% create a UAV object with initial position x0,y0, radius=r
u1 = UAV([x0 y0]);
u1.set_radius(r);
u1.set_field(field);
u1.set_destination(floor(initial_waypoint)); 

% Only cover at most a percentage of the field
uav_fuel = max_scan_percentage;

waitbar(.06, wb, 'Starting flight...');

u1.start_flight()

if (live_plot)
    live_plot_figure = figure();
end

%% Simulation

% records
var_field_recs = zeros(width, width, 50);
pred_field_recs = zeros(width, width, 50);

wps = 1;
percentages = [];

pathfound = [];

if (live_plot)
    figure(live_plot_figure);

    % actual field - upper left quad
    h1=subplot(2,2,1);
    imagesc(field.z')
    title('Actual Field')
    colorbar;
    pause(0.01);
end

last_area_covered = 0;
save_snapshot = false;

while (true)
    area_covered = length(u1.s_loc)/(width^2);
    waitbar(area_covered/max_scan_percentage, wb, 'Exploring field...');
    
    u1.update();
    
    
    if (u1.waypoint_reached && isempty(pathfound))        
        d = variogram(u1.s_loc, u1.samples(:), 'plot', false);
        [a,c,~,vstruct] = variogramfit(d.distance,d.val,a,c,[],'plotit',false);
        
        % Running Kriging prediction
        [zi,s2zi] = kriging(vstruct, u1.s_loc(:,1),u1.s_loc(:,2), ...
        u1.samples(:), ...
        xi, yi, ...
        false, ...
        length(u1.samples));

        % Loading predictions and variances into field memory
        for ix = 1:width^2
            pred_field(xi(ix),yi(ix)) = zi(ix);
            var_field(xi(ix),yi(ix)) = s2zi(ix);
        end

        % It could take a while to compute the trajectory to just plot the
        % known fields for this iteration now and after.
        if (live_plot)
            figure(live_plot_figure);
            
            % pred. field - upper right quad
            h2=subplot(2,2,2);
            imagesc(pred_field') % trans() because of the way imagesc works
            current_rms_error = norm(pred_field - field.z);
            title(strcat(['Predicted Field - RMS Error: ', num2str(current_rms_error)]))
            colorbar;
            
            % var field - lower left quad
            h3=subplot(2,2,3);     
            %surf(var_field);
            imagesc(var_field') % trans() because of the way imagesc works
            hold on
            plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2)
            hold off
            colormap(h3,hot(8))
            colorbar;
            title('Variance Field')
            
            % traced field - lower right quad
            h4 = subplot(2,2,4);       
            plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5)
            hold on
            plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['g',u1.cursor]), 'LineWidth', 2)
            hold on
            plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2)
            hold off
            set(h4,'Ydir','reverse');
            title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
            % set the boundry of the frame
            axis([-1 width+1 -1 width+1]);
            
            pause(.01)
        end
        
        if (wps <= 50)
            var_field_recs(:,:,wps) = var_field;   
            pred_field_recs(:,:,wps) = pred_field;   
        else
            var_field_recs(:,:,end + 1) = var_field;   
            pred_field_recs(:,:,end + 1) = pred_field;
        end
        
        percentages(end + 1) = area_covered;
        wps = wps + 1;
        
        if (strcmp(method, 'mc'))
            [pathfound] = mc_krigpathfind(var_field, pred_field, u1.curr_pos, u1.s_loc, u1.samples(:), xi, yi, a, c);
        elseif (strcmp(method, 'oc'))
            [pathfound] = optimal_kriging_pathfinder(var_field, pred_field, u1.curr_pos, u1.s_loc, u1.samples(:), xi, yi, a, c);
        elseif (strcmp(method, 'nhv'))
            [pathfound] = nhv_krigpathfind(var_field);
        elseif (strcmp(method, 'nnhv'))
            [pathfound] = nhv_sets_krigpathfind(var_field, pred_field, u1.curr_pos, u1.s_loc, u1.samples(:), xi, yi, a, c);
        elseif (strcmp(method, 'gradient'))
            
            % try this, it works pretty well (save for later, not paper)
%             step_size = vstruct.range;

            step_size = 3;
            
            pathfound = [-1 -1];
            max_candidate_var_found = 0;
            % make a circle of candidate locations
            for thetad = 0 : 360
                candidate_point = u1.curr_pos + step_size*[cosd(thetad) sind(thetad)];
                
                candidate_point = ceil(candidate_point);
                if (candidate_point(1) > width)
                    candidate_point(1) = width;
                end
                if (candidate_point(1) < 1)
                    candidate_point(1) = 1;
                end
                
                if (candidate_point(2) > width)
                    candidate_point(2) = width;
                end
                if (candidate_point(2) < 1)
                    candidate_point(2) = 1;
                end
                
                candidate_var = var_field(candidate_point(1), candidate_point(2));
                if (candidate_var > max_candidate_var_found)
                   pathfound = candidate_point;
                   max_candidate_var_found = candidate_var;
                end
                
            end

         elseif (strcmp(method, 'gr'))
            
            step_size = vstruct.range;
            
            pathfound = [-1 -1];
            max_candidate_var_found = 0;
            % make a circle of candidate locations
            for thetad = 0 : 360
                candidate_point = u1.curr_pos + step_size*[cosd(thetad) sind(thetad)];
                
                candidate_point = ceil(candidate_point);
                if (candidate_point(1) > width)
                    candidate_point(1) = width;
                end
                if (candidate_point(1) < 1)
                    candidate_point(1) = 1;
                end
                
                if (candidate_point(2) > width)
                    candidate_point(2) = width;
                end
                if (candidate_point(2) < 1)
                    candidate_point(2) = 1;
                end
                
                candidate_var = var_field(candidate_point(1), candidate_point(2));
                if (candidate_var > max_candidate_var_found)
                   pathfound = candidate_point;
                   max_candidate_var_found = candidate_var;
                end
                
            end
            
        elseif (strcmp(method, 'nbv'))
            
            step_size = 2;
            
            % make a circle of candidate locations
            maximum_variances = sort(unique(max(var_field)), 'descend');
            [pix, piy] = find(var_field == maximum_variances(1));
            pix = pix(1);
            piy = piy(1);
            
            if (pix > width)
                pix = width;
            end
            if (pix < 1)
                pix = 1;
            end

            if (piy > width)
                piy = width;
            end
            if (piy < 1)
                piy = 1;
            end
                
            thetad = atan2d(piy - u1.curr_pos(2), pix - u1.curr_pos(1));
            pathfound = u1.curr_pos + step_size*[cosd(thetad) sind(thetad)];
                            
        elseif (strcmp(method, 'poi'))
            [pathfound] = npoi_krigpathfind( var_field, pred_field );
        else
            close(wb);
            fprintf('Error! Improper method type.\n')
            return;
        end
        
        if uav_fuel - area_covered <= 0
            waitbar(1, wb, 'Exploration complete!');
            break;
        end
        
        if (~isempty(save_image_path))
            if last_area_covered < 10 && area_covered*100 >= 10   
                snapshot_name = strcat(['path_', num2str(10), '_snapshot_', save_image_path]);
                save_snapshot = true;
            end
            
            if last_area_covered < 20 && area_covered*100 >= 20   
                snapshot_name = strcat(['path_', num2str(20), '_snapshot_', save_image_path]);
                save_snapshot = true;
            end
            
            if last_area_covered < 30 && area_covered*100 >= 30   
                snapshot_name = strcat(['path_', num2str(30), '_snapshot_', save_image_path]);
                save_snapshot = true;
            end
            
            if (save_snapshot)
                path_fig = figure('visible', 'off');
                plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5);
                hold on;
                plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['r',u1.cursor]), 'LineWidth', 2);
                hold off;
                set(gca, 'Ydir', 'reverse');
                title(strcat(['Trace - Area Covered: ', num2str(area_covered * 100), '%']));
                % set the boundry of the frame
                axis([-1 width+1 -1 width+1]);
                saveas(gca, snapshot_name, 'png'); 
                
                save_snapshot = false;
            end
            
            last_area_covered = area_covered * 100;
        end
        
        u1.set_destination(pathfound(1,:));     
        if (length(pathfound) > 1)
            pathfound = pathfound(2:end, :);
        else
            pathfound = [];
        end
        
        if (live_plot)
            figure(live_plot_figure);
            
            % actual field - upper left quad
            h1=subplot(2,2,1);
            imagesc(field.z')
            title('Actual Field')
            colorbar;
            
            % pred. field - upper right quad
            h2=subplot(2,2,2);
            imagesc(pred_field') % trans() because of the way imagesc works
            current_rms_error = norm(pred_field - field.z);
            title(strcat(['Predicted Field - Error: ', num2str(sqrt(mean(abs(pred_field(:).^2 - field.z(:).^2))))]));
            colorbar;
            
            % var field - lower left quad
            h3=subplot(2,2,3);     
            %surf(var_field);
            imagesc(var_field') % trans() because of the way imagesc works
            hold on
            plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2)
            hold off
            colormap(h3,hot(8))
            colorbar;
            title('Variance Field')
            
            % traced field - lower right quad
            h4 = subplot(2,2,4);       
            plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5)
            hold on
            plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['g',u1.cursor]), 'LineWidth', 2)
            hold on
            plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2)
            hold on
            plot(pathfound(:, 1), pathfound(: ,2), 'rx', 'LineWidth', 2)
            hold off
            set(h4,'Ydir','reverse');
            title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
            % set the boundry of the frame
            axis([-1 width+1 -1 width+1]);
            pause(.01)           
        end
       
    elseif (u1.waypoint_reached && ~isempty(pathfound))
        u1.set_destination(pathfound(1, :));
        pathfound = pathfound(2:end, :);
        
        if (live_plot)
            h4 = subplot(2,2,4);       
            plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5)
            hold on
            plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['g',u1.cursor]), 'LineWidth', 2)
            hold on
            plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2)
            hold on
            plot(pathfound(:, 1), pathfound(: ,2), 'rx', 'LineWidth', 2)        
            hold off
            set(h4,'Ydir','reverse');
            title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
            % set the boundry of the frame
            axis([-1 width+1 -1 width+1]);
            pause(.01)
        end
    end
end

wps = wps - 1;

if (live_plot)
    figure(live_plot_figure);

    % actual field - upper left quad
    h1=subplot(2,2,1);
    imagesc(field.z')
    title('Actual Field')
    colorbar;

    % pred. field - upper right quad
    h2=subplot(2,2,2);
    imagesc(pred_field') % trans() because of the way imagesc works
    current_rms_error = norm(pred_field - field.z);
    title(strcat(['Predicted Field - Error: ', num2str(sqrt(mean(abs(pred_field(:).^2 - field.z(:).^2))))]));
    colorbar;

    % var field - lower left quad
    h3=subplot(2,2,3);     
    %surf(var_field);
    imagesc(var_field') % trans() because of the way imagesc works
    hold on
    plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2)
    hold off
    colormap(h3,hot(8))
    colorbar;
    title('Variance Field')

    % traced field - lower right quad
    h4 = subplot(2,2,4);       
    plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5)
    hold on
    plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['r',u1.cursor]), 'LineWidth', 2)
    hold off
    set(h4,'Ydir','reverse');
    title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
    % set the boundry of the frame
    axis([-1 width+1 -1 width+1]);

    pause(.01)
end

if (~isempty(save_image_path))
    return_fig = figure('visible', 'off');
    % actual field - upper left quad
    h1=subplot(2,2,1);
    imagesc(field.z');
    title('Actual Field');
    colorbar;

    % pred. field - upper right quad
    h2=subplot(2,2,2);
    imagesc(pred_field'); % trans() because of the way imagesc works
    title(strcat(['Predicted Field - Error: ', num2str(sqrt(mean(abs(pred_field(:).^2 - field.z(:).^2))))]));
    colorbar;

    % var field - lower left quad
    h3=subplot(2,2,3);     
    %surf(var_field);
    imagesc(var_field'); % trans() because of the way imagesc works
    hold on;
    plot(u1.dest_pos(1), u1.dest_pos(2), 'bx', 'LineWidth', 2);
    hold off;
    colormap(h3,hot(8));
    colorbar;
    title('Variance Field');

    % traced field - lower right quad
    h4 = subplot(2,2,4);       
    plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5);
    hold on;
    plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['r',u1.cursor]), 'LineWidth', 2);
    hold off;
    set(h4,'Ydir','reverse');
    title(strcat(['Trace - Area Covered: ', num2str(area_covered * 100), '%']));
    % set the boundry of the frame
    axis([-1 width+1 -1 width+1]);

    pause(.01);
    
    saveas(return_fig, save_image_path, 'png');

    %%
    path_fig = figure('visible', 'off');
    plot(u1.s_loc(:,1),u1.s_loc(:,2), 'k-', 'LineWidth', 1.5);
    hold on;
    plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['r',u1.cursor]), 'LineWidth', 2);
    hold off;
    set(gca, 'Ydir', 'reverse');
    title(strcat(['Trace - Area Covered: ', num2str(area_covered * 100), '%']));
    % set the boundry of the frame
    axis([-1 width+1 -1 width+1]);
    
    saveas(gca, strcat(['path_',save_image_path]), 'png');

end

close(wb);

var_field_recs = var_field_recs(:,:,1:wps);
pred_field_recs = pred_field_recs(:,:,1:wps);
end

