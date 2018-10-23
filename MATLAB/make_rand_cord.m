close all;

width = 200;
start_coor = [100 50];
end_coors = [200 200; 5 175];

N = 2;
rand_cords = 15;

var = 10;

f = zeros(width,width);
alpha = 2 * var;

for n = 1:N

    end_coor = end_coors(n, :);
    plot(linspace(start_coor(1), end_coor(1), 10), linspace(start_coor(2), end_coor(2), 10), '-r', 'LineWidth', 3)
    hold on
    
    for rc = 1:rand_cords
    
        P = zeros(ceil(norm(end_coor - start_coor)/alpha) - 1, 2);
        P(1, :) = start_coor;
        for point = 2 : length(P)
            angle = atan2d(end_coor(2)-P(point - 1, 2), end_coor(1)-P(point - 1, 1));
            P(point, :) = P(point - 1, :) + alpha * [cosd(angle), sind(angle)] + sqrt(var) * randn(1,2);
        end
        P(end, :) = end_coor;

        plot(P(:,1), P(:,2), '-k', 'LineWidth', 0.5)
        hold on

    end
end

plot(start_coor(1), start_coor(2), 'go', 'LineWidth', 10)
hold on

axis([-1 width+1 -1 width+1]);



