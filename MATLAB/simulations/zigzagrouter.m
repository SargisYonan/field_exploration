function [ wps, pred_field_recs, var_field_recs, percentages ] = zigzagrouter( X0, field, wrap_spacing, max_area_covered_percentage, live_plot, save_image_path )

width = size(field.z, 1);

top = true;
bottom = false;
left = true;
right = false;
% initial positon of the UAV
x0 = X0(1);
y0 = X0(2);

% prediction area
[xi, yi] = meshgrid(1:width);
xi = xi(:);
yi = yi(:);

% the turning radius of the UAV
r = .00001;

wrap_radius = wrap_spacing;
%% Kriging Parameters
a = 40; % initial seed for fmincon's range search
c = 5; % initial seed for fmincon's sill search

pred_field = zeros(size(field.z));
var_field = inf .* ones(size(field.z));

% create a UAV object with initial position x0,y0, radius=r
u1 = UAV(X0);
u1.set_radius(r);
u1.set_field(field);
u1.set_destination((floor(width/2) * [1 1] - [round(wrap_radius) 0])); 

u1.start_flight()

if (live_plot)
    live_plot_figure = figure();
end

%% Simulation
var_field_recs = zeros(width, width, 100);
pred_field_recs = zeros(width, width, 100);

wraps = round((width/2) / wrap_radius);
u1.curr_pos = X0;

wps = 0;
percentages = [];

while (true)
    
    u1.update();
    
    area_covered = length(u1.s_loc)/(width^2);
    
    if (max_area_covered_percentage < 1 && area_covered >= max_area_covered_percentage)
        break;
    end
    
    if (u1.waypoint_reached)
       
       if (abs(ceil(u1.curr_pos(1)) - X0(1)) < wrap_radius/2 && abs(ceil(u1.curr_pos(2)) - X0(2)) < wrap_radius/2)
           break; 
       else
                    
           if (left && top)
                right = true;
                left = false;
                top = true;
                bottom = false;
                wraps = wraps - 1;

            elseif (right && top)
                bottom = true;
                right = true;
                top = false;
                left = false;
            elseif (right && bottom)
                left = true;
                bottom = true;
                top = false;
                right = false;
            elseif (left && bottom)
                top = true;
                left = true;
                right = false;
                bottom = false;
            end

            if (left)
                nx = (wraps * wrap_radius);
            else
                nx = width - (wraps * wrap_radius);
            end

            if (top)
                ny = (wraps * wrap_radius);
            else
                ny = width - (wraps * wrap_radius);
            end

            u1.set_destination([round(nx+1) round(ny+1)]);

                
            if ( length(u1.samples(:)) > width )
                d = variogram(u1.s_loc, u1.samples(:), 'plot', false);
                [a,c,n,vstruct] = variogramfit(d.distance,d.val,a,c,[],'plotit',false);

                [zi,s2zi] = kriging(vstruct, u1.s_loc(:,1),u1.s_loc(:,2), ...
                u1.samples(:), ...
                xi, yi, ...
                false, ...
                100);

                for ix = 1:width^2
                    pred_field(xi(ix),yi(ix)) = zi(ix);
                    var_field(xi(ix),yi(ix)) = s2zi(ix);
                end
                
                wps = wps + 1;

                var_field_recs(:,:,wps) = var_field;   
                pred_field_recs(:,:,wps) = pred_field;   
                percentages(end + 1) = area_covered;
            
            
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
                hold off
                set(h4,'Ydir','reverse');
                title(strcat(['UAV Trace - Area Covered: ', num2str(area_covered * 100), '%']))
                % set the boundry of the frame
                axis([-1 width+1 -1 width+1]);
                pause(.01)
            end
            end
        end
    end
end 
                

d = variogram(u1.s_loc, u1.samples(:), 'plot', false);
[a,c,n,vstruct] = variogramfit(d.distance,d.val,a,c,[],'plotit',false);

[zi,s2zi] = kriging(vstruct, u1.s_loc(:,1),u1.s_loc(:,2), ...
u1.samples(:), ...
xi, yi, ...
false, ...
100);

for ix = 1:width^2
    pred_field(xi(ix),yi(ix)) = zi(ix);
    var_field(xi(ix),yi(ix)) = s2zi(ix);
end

wps = wps + 1;

var_field_recs(:,:,wps) = var_field;   
pred_field_recs(:,:,wps) = pred_field;   
percentages(end + 1) = area_covered;
                
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
    plot(u1.s_loc(end,1),u1.s_loc(end,2), strcat(['r',u1.cursor]), 'LineWidth', 2)
    hold off
    set(h4,'Ydir','reverse');
    title(strcat(['Trace - Area Covered: ', num2str(area_covered * 100), '%']))
    % set the boundry of the frame
    axis([1 width+1 1 width+1]);

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
    plot(u1.s_loc(end,1),u1.s_loc(end,2), strcat(['r',u1.cursor]), 'LineWidth', 2);
    hold off;
    set(h4,'Ydir','reverse');
    title(strcat(['Trace - Area Covered: ', num2str(area_covered * 100), '%']));
    % set the boundry of the frame
    axis([-1 width+1 -1 width+1]);

    pause(.01);
    
    saveas(return_fig, save_image_path, 'png');
end

var_field_recs = var_field_recs(:,:,1:wps);
pred_field_recs = pred_field_recs(:,:,1:wps);

end

