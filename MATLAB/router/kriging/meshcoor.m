function [ xi, yi ] = meshcoor( center_coor, sq_radius, field_max )

% returns the coordinates [xi, yi] representing the coordinates on the
% field with the perimeter sq_radius around center_coor

cx = center_coor(1);
cy = center_coor(2);

field_min = 1;
xi = [];
yi = [];

for xx = max(field_min,cx-sq_radius):min(cx+sq_radius, field_max)
    for yy = max(field_min,cy-sq_radius):min(cy+sq_radius, field_max)
        xi = [xi; xx];
        yi = [yi; yy];
    end 
end

end

