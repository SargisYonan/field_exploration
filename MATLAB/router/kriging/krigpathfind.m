function [ pathfound, neighbors ] = krigpathfind( varnum, var_field, curr_pos )
% Find your way using the kriging method...

% @param varnum - the number of tesselations to create, i.e the number 
%                 of major variances to select for the pathfound

% @param var_field - the matrix which represents the variances of the
%                    gridded field

        % find a handful of the largest variances
        
        var_to_sort = var_field(var_field > 0);
        [var_to_sort, ~, ~] = unique(var_to_sort);
        [vars_sorted, original_pos] = sort( var_to_sort, 'descend' );
        vars_sorted = vars_sorted(1:varnum);
        
        target_vertex_locs = zeros(varnum, 2 + 1);
        neighbors = zeros(varnum, 2 + 1);
        search_var_field = var_field;
        for vi = 1:varnum
            [lx, ly] = find(search_var_field == (vars_sorted(vi)));
            target_vertex_locs(vi, :) = [lx(1) ly(1) vars_sorted(vi)];
            search_var_field(lx(1), ly(1)) = -1;
        end

        for vi = 1:varnum
            neigh = knnsearch(target_vertex_locs, target_vertex_locs(vi, :), 'k', 1);
            neighbors(neigh, :) = [target_vertex_locs(vi, :)];
        end
        
        dest_node = knnsearch(target_vertex_locs(:,1:2), target_vertex_locs(1, 1:2), 'k', 1);
        start_node = knnsearch(target_vertex_locs(:,1:2), floor(curr_pos), 'k', 1);
        
        Am = zeros(varnum, varnum);
        for ai = 1:varnum
            for aj = 1:varnum
                if (Am(ai,aj) == 0 && Am(aj,ai) == 0)
                    if (isempty(find(ai == knnsearch(target_vertex_locs(:,1:2), neighbors(aj,1:2), 'k', 4))))
                        Am(ai, aj) = 0;
                        Am(aj, ai) = 0;
                    else
                        Am(ai, aj) = 1 / (neighbors(ai,3) + neighbors(aj,3))^2;
                        Am(aj, ai) = Am(ai, aj);
                    end
                end
            end
        end
        
        G = graph(Am);
        [pathfound, D] = shortestpath(G, start_node, dest_node);
        
        max_var = max(max(var_field));
        if (D == inf)
            [nrow, ncol] = find(var_field == max_var);
            % calculate distance to the next within reach large var
            cx = curr_pos(1);
            cy = curr_pos(2);
            [dv, nextidx] = min(abs( (cx^2 + cy^2) - (nrow.^2 + ncol.^2) ));
            neighbors = [nrow(nextidx) ncol(nextidx)];
            pathfound = 1;
        end

end

