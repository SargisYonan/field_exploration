function [ best_path ] = mc_krigpathfind( var_field, predicted_field, curr_pos, sample_locations, samples, xi, yi,var_a, var_c )
%%
% Find your way using the kriging method...
%
% @param sampled_locations - an nx2 vector of all sampled locations.
%                            ex. [[x1, y1]; [x2, y2]; ... ]
% @param var_field The variance field generated using Kriging().
% @param curr_pos The current position of the vehicle.
%
% @return pathfound The vector of 2-vectors indicating which route
%         to take.
%         ex. [[dest_x1, dest_y1]; [dest_x2, dest_y2]; ... ]
%
 
% Monte Carlo endpoints
mck = 5;
% Random MC path per endpoint to calculate
random_paths_per_ep = 20;

rand_var_x = size(var_field, 1)/50;
rand_var_y = size(var_field, 1)/50;

best_mean = inf;

% Find the coordinates of the maximum variances.
maximum_variances = sort(max(var_field), 'descend');

if (length(maximum_variances) < mck)
    mck = length(maximum_variances);
end

maximum_variances = maximum_variances(1:mck);

for endpoint_index = 1 : mck

[ix, iy] = find(var_field == maximum_variances(endpoint_index));

% Find the maximum variance on the variance field closest
% to the current vehicle position.
closest_variance_coor = [ix(1), iy(1)];
closest_variance_dist = norm(closest_variance_coor - curr_pos);

alpha = 4 * max(sqrt([rand_var_x rand_var_y]));

for randpaths = 1 : random_paths_per_ep
    
% compute a trajectory given an initial location and final location
pn = ceil(closest_variance_dist / alpha);
calculated_path = zeros(pn, 2);

path_pos = curr_pos;
for px = 1 : pn    
    % use this for Monte Carlo later...
    angle = atan2d(closest_variance_coor(2)-path_pos(2), closest_variance_coor(1)-path_pos(1));
    
    if (randpaths == 1)
        path_pos = ceil(path_pos + alpha * [cosd(angle), sind(angle)]);    
    else
        path_pos = ceil(path_pos + alpha * [cosd(angle), sind(angle)] + [rand_var_x * randn, rand_var_y * randn]);
    end
    
    % clamp
    if path_pos(1) < 1
        path_pos(1) = 1;
    end
    if path_pos(2) < 1
        path_pos(2) = 1;
    end
    
    if path_pos(1) > size(var_field, 1)
        path_pos(1) = size(var_field, 1);
    end
    if path_pos(2) > size(var_field, 2)
        path_pos(2) = size(var_field, 2);
    end
 
    calculated_path(px,:) = path_pos;
end
calculated_path(pn,:) = closest_variance_coor;

new_samples = zeros(length(calculated_path), 1);
for pi = 1 : length(calculated_path)
    new_samples(pi) = predicted_field(calculated_path(pi,1), calculated_path(pi,2));
end
new_samples = [samples ; new_samples];

new_sample_locations = [sample_locations ; calculated_path];

d = variogram(new_sample_locations, new_samples, 'plot', false);
[~, ~, ~, vstruct] = variogramfit(d.distance, d.val, var_a, var_c, [], 'plotit', false);

[~, s2zi] = kriging(vstruct, ...
                    new_sample_locations(:,1), new_sample_locations(:,2), ...
                    new_samples, ...
                    xi, yi, ...
                    true, ...
                    length(new_samples));

for ix = 1:length(xi)
    new_var_field(round(xi(ix)),round(yi(ix))) = s2zi(ix);
end

new_var_mean = mean2(new_var_field);

if new_var_mean < best_mean
    best_mean = new_var_mean;
    best_path = calculated_path;
end

end % rands

end

end

