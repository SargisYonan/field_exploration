function [ pathfound ] = optimal_kriging_pathfinder( var_field, predicted_field, curr_pos, sample_locations, samples, xi, yi,var_a, var_c )

[h,w] = size(var_field);

possible_endpoints = [];

best_var = inf;

cx = round(curr_pos(1));
cy = round(curr_pos(2));

if (cx > w)
    cx = w;
end

if (cx < 1)
    cx = 1;
end

if (cy > h)
     cy = h;
end

if (cy < 1)
    cy = 1;
end

alpha = 1;

% check right
if (cx < w)
    possible_endpoints(end + 1, :) = [min(w, cx + alpha), cy]; % right
end

% check left
if (cx > 1)
    possible_endpoints(end + 1, :) = [max(1, cx - alpha), cy]; % left
end

% check down
if (cy < h)
     possible_endpoints(end + 1, :) = [cx, min(h, cy + alpha)]; % down
end

% check up
if (cy > 1)
    possible_endpoints(end + 1, :) = [cx, max(1, cy - alpha)]; % up
end

% check up and right
if (cx < w && cy > 1)
     possible_endpoints(end + 1, :) = [min(w, cx + alpha), max(1, cy - alpha)]; % up and right
end

% up and left
if (cx > 1 && cy > 1)
     possible_endpoints(end + 1, :) = [max(1, cx - alpha), max(1, cy - alpha)]; % up and left
end

% left and down
if (cx > 1 && cy < h)
     possible_endpoints(end + 1, :) = [max(1, cx - alpha), min(h, cy + alpha)]; % left and down
end

% up and right
if (cx < w && cy > 1)
     possible_endpoints(end + 1, :) = [min(w, cx + alpha), max(1, cy - alpha)]; % up and right
end

new_var_field = zeros(size(var_field));
for pi = 1 : length(possible_endpoints)
    px = possible_endpoints(pi,1);
    py = possible_endpoints(pi,2);
    
    new_samples = [samples ; predicted_field(px, py)];
    new_sample_locations = [sample_locations ; [px py]];

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
    this_var = mean2(new_var_field);

    if this_var < best_var
        best_var = this_var;
        pathfound = [px py];
    end

end

end

