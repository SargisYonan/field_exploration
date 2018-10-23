function [ best_path ] = nhv_krigpathfind( var_field )

% Find the coordinates of the maximum variances.
maximum_variances = sort(unique(max(var_field)), 'descend');
[ix, iy] = find(var_field == maximum_variances(1));

best_path = [ix(1), iy(1)];

end