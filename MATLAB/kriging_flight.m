% A software in the loop simulation of a demonstration of the autonomous
% path planning and prediction of an unknown field using the Kriging Method

% @author Sargis S Yonan
% @date August 2017
%
% @brief Simulation for Computer Engineering Masters Thesis
% 
% Department of Computer Engineering
% Autonomous Systems Lab
% University of California, Santa Cruz
%
% Last Revised October 2018


%% Get the environment setup

clc;
clear all;
close all;

%% Knobs

record_flight = false;
live_plot = true;

if (record_flight)
    % format the file name for the video to have the date and time in it
    [token, remain] = strtok(datestr(datetime), ' ');
    [token2, remain2] = strtok(remain, ' ');
    live_plot_video_filename = strcat(['video/', token, '-', token2, '-kriging_explore_sim']);
    
    fprintf('Recording flight --> %s\n', live_plot_video_filename)
end

%% load the required libraries

% load the libraries in the pwd
disp('Loading UAV libraries into path...')
disp('Loading Field libraries into path...')
addpath(genpath(pwd))

%% Vehicle/Field Dynamics Parameters

% square size of the field
width = 50;

% initial positon of the UAV
x0 = 1;
y0 = 1;

% the turning radius of the UAV
r = 0.001;

% prediction area
[xi, yi] = meshcoor([1 1], width, width);
        
%% Kriging Parameters
a = 100; % initial seed for fmincon's range search
c = 20; % initial seed for fmincon's sill search

%% Initial Conditions, UAV Creation, and Field Setup

fprintf('Creating a field size %dx%d\n\n', width, width)
% generate the field object to be observed
field = Field(width, width);
 
fprintf('allocating and initializing memory for the predicted field\n')
% the predicted field - initially nothing observed
pred_field = zeros(size(field.z));
fprintf('allocating and initializing memory for the variance field\n\n')
var_field = inf .* ones(size(field.z));

% create a UAV object with initial position x0,y0, radius=r
u1 = UAV([x0 y0]);
u1.set_radius(r);
u1.set_field(field);
u1.set_destination(floor([width, width])); 

% Only cover at most a percentage of the field
uav_fuel = .2 * width^2;

fprintf('UAV created:\n')
fprintf('\tDubins Vehicle radius: %.2f\n', u1.radius)
fprintf('\tStarting positon: (x0,y0)=(%f,%f)\n', u1.curr_pos(1), u1.curr_pos(2))
fprintf('\tInitial waypoint: (x,y)=(%f,%f)\n', u1.dest_pos(1), u1.dest_pos(2))

fprintf('enabling flight\n\n')
u1.start_flight()

if (live_plot)
    fprintf('starting live plot environment\n')
    live_plot_figure = figure();
    if (record_flight)
        fprintf('initializing flight recording object\n')
        
        writerObj = VideoWriter(live_plot_video_filename);
        
        writerObj.FrameRate = 10;
        open(writerObj);
    end
end

%% Simulation

% records
var_field_recs = zeros(width, width, 50);
wps = 1;

max_var = inf;
last_max_var = inf;
epsilon_var = 1;

pathfound = [];
fprintf('Simulation starting...\n')

if (live_plot)
    figure(live_plot_figure);

    % actual field - upper left quad
    h1=subplot(2,2,1);
    imagesc(field.z')
    title('Actual Field')
    colorbar;
    pause(0.01);
end

while (true)
    
    u1.update();
    
    area_covered = length(u1.s_loc)/(width^2);
    
    if (u1.waypoint_reached && isempty(pathfound))
        
        fprintf('waypoint at [%.2f, %.2f] reached:\n', u1.curr_pos(1), u1.curr_pos(2))
        
        fprintf('Computing semi-variogram\n')
        d = variogram(u1.s_loc, u1.samples(:), 'plot', false);
        fprintf('Fitting a continuous variogram\n')
        [a,c,n,vstruct] = variogramfit(d.distance,d.val,a,c,[],'plotit',false);
        
        % if you want unbounded predictions. Not suggested.
        % vstruct.type = '';
        
        % Run a "bounded" Kriging prediciton. This means that we are only
        % considering weights within our learned "range" distance.
        fprintf('Running Kriging prediction on [%d] selected points...\n', length(xi))
        [zi,s2zi] = kriging(vstruct, u1.s_loc(:,1),u1.s_loc(:,2), ...
        u1.samples(:), ...
        xi, yi, ...
        false, ...
        length(u1.samples));

        fprintf('Loading predictions and variances into field memory\n')
        for ix = 1:length(xi)
            pred_field(round(xi(ix)),round(yi(ix))) = zi(ix);
            var_field(round(xi(ix)),round(yi(ix))) = s2zi(ix);
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
        
        
        var_field_recs(:,:,wps) = var_field;   
        wps = wps + 1;
        
        [pathfound] = optimal_kriging_pathfinder(var_field, pred_field, u1.curr_pos, u1.s_loc, u1.samples(:), xi, yi, a, c);

        if uav_fuel - area_covered <= 0
            fprintf('No fuel remaining.\n')
            fprintf('Path planner terminated.\n')
            
            break;
        end
        
        u1.set_destination(pathfound(1,:));     
        if (length(pathfound) > 1)
            pathfound = pathfound(2:end, :);
        else
            pathfound = [];
        end
        
        fprintf('setting waypoint at [%.2f, %.2f]\n', u1.dest_pos(1), u1.dest_pos(2))
        
        fprintf('\n********************************************************************************\n\n')
        
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
            hold on
            plot(pathfound(:, 1), pathfound(: ,2), 'rx', 'LineWidth', 2)
            hold off
            set(h4,'Ydir','reverse');
            title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
            % set the boundry of the frame
            axis([-1 width+1 -1 width+1]);
            pause(.01)
            
            if (record_flight)
                frame = getframe(gcf);
                writeVideo(writerObj, frame);
            end
            
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
    plot(u1.curr_pos(1),u1.curr_pos(2), strcat(['r',u1.cursor]), 'LineWidth', 2)
    hold off
    set(h4,'Ydir','reverse');
    title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
    % set the boundry of the frame
    axis([-1 width+1 -1 width+1]);

    pause(.01)
end
            
if (record_flight)
    % write last frame and save
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
    fprintf('Closing video writing object\n');
    close(writerObj); % saves the movie
    fprintf('Video writing object closed\n\n')
end

current_rms_error = norm(pred_field - field.z);
fprintf('\nRMS Error: %.2f\n', current_rms_error)

arc_length = 0.0;
for pi = 2 : length(u1.s_loc)
    arc_length = arc_length + norm(u1.s_loc(pi,:) - u1.s_loc(pi-1,:));
end

fprintf('Arc length: %f\n', arc_length)


