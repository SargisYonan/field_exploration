function [ wps, pred_field_recs, var_field_recs, percentages ] = zigzag_explore( X0, field, percent_scan, live_plot, save_image_path )

Xm = [ceil(field.width/2) ceil(field.height/2)];

% prediction area
[xi, yi] = meshgrid(1:field.width);
xi = xi(:);
yi = yi(:);

%% Kriging Parameters
a = 40; % initial seed for fmincon's range search
c = 5; % initial seed for fmincon's sill search

pred_field = zeros(size(field.z));
var_field = inf .* ones(size(field.z));

% create a UAV object with initial position x0,y0, radius=r
u1 = UAV(X0);
u1.set_radius(0.000001);
u1.set_field(field);

u1.start_flight()

if (live_plot)
    live_plot_figure = figure();
end

%% Simulation
var_field_recs = zeros(field.width, field.width, 100);
pred_field_recs = zeros(field.width, field.width, 100);

u1.curr_pos = X0;

percentages = [];


% hard_coded_scan_area = norm(X0 - Xm) / (field_width * field_height);
r = 1 / percent_scan;

wraps = (field.height / 2) / r ;
K = ceil(4 * wraps) - 2;

Pzz = zeros(K, 2);

X = zeros(K, 1);
Y = zeros(K, 1);

% top down to middle movement is hardcoded as follows:

% top left of the middle point

Pi = [X0; [-r -r]; [r -r]; [r r]];
% initial location
X(1) = X0(1);
Y(1) = X0(2);
X(2) = -r;
Y(2) = -r;
X(3) = r;
Y(3) = -r;
X(4) = r;
Y(4) = r;

Pzz(1, :) = Pi(1, :);
Pzz(2:size(Pi, 1), :) = [Xm(1)*ones(size(Pi, 1)-1,1) Xm(2)*ones(size(Pi, 1)-1,1)] + [X(2:size(Pi, 1)) Y(2:size(Pi, 1))];

r_multiplier = 1;
x_sign = 1;
for k = size(Pi,1) + 1 : K
    if (mod(k-1, 4) == 0)
        r_multiplier = r_multiplier + 1;
    end
    
    if (mod(k+1, 2) == 0)
        x_sign = -1 * x_sign;
    end
    
    X(k) = x_sign * r_multiplier * r;
    Y(k) = X(k-1);
    
    Pzz(k,:) = Xm + [X(k) Y(k)];
    
    if (Pzz(k,1) > field.width)
        Pzz(k,1) = field.width;
    end
    if (Pzz(k,1) < 1)
        Pzz(k,1) = 1;
    end
    
    if (Pzz(k,2) > field.height)
        Pzz(k,2) = field.height;
    end
    if (Pzz(k,2) < 1)
        Pzz(k,2) = 1;
    end
end

k = 2;
wps = 0;

u1.set_destination(Pzz(k,:));
k = k + 1;
while (true)
    
    u1.update();
    
    if (u1.waypoint_reached)
       
        if (k > K) % ran through all points in the trajectory
            break;
        else
            u1.set_destination(Pzz(k,:));
            k = k + 1;

            % re-krig every n-th waypoint
            modN = 1;
            if (mod(k, modN) == 0)
                wps = wps + 1;
                d = variogram(u1.s_loc, u1.samples(:), 'plot', false);
                [a,c,~,vstruct] = variogramfit(d.distance,d.val,a,c,[],'plotit',false);

                [zi,s2zi] = kriging(vstruct, u1.s_loc(:,1),u1.s_loc(:,2), ...
                    u1.samples(:), ...
                    xi, yi, ...
                    false, ...
                    100);

                for ix = 1:field.width^2
                    pred_field(xi(ix),yi(ix)) = zi(ix);
                    var_field(xi(ix),yi(ix)) = s2zi(ix);
                end

                var_field_recs(:,:,wps) = var_field;   
                pred_field_recs(:,:,wps) = pred_field;   

                area_covered = length(u1.s_loc)/(field.width^2);
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
                    title(strcat(['Vehicle Trace - Area Covered: ', num2str(area_covered * 100), '%']))
                    % set the boundry of the frame
                    axis([-1 (field.width+1) -1 (field.width+1)]);
                    pause(.01)
                end
            end
        end        
    end
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
    axis([1 field.width+1 1 field.width+1]);

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
    axis([-1 field.width+1 -1 field.width+1]);

    pause(.01);
    
    saveas(return_fig, save_image_path, 'png');
end

var_field_recs = var_field_recs(:,:,1:wps);
pred_field_recs = pred_field_recs(:,:,1:wps);

end

