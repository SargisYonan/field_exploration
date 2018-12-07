function [ ap_mean_var, ap_mean_pred_err ] = apriori_errors( field, points )
    
    if (nargin == 1)
        points = 5;
    end
    
    s_locs = ones(points, 2);
    for p = 1:points
        s_locs(p,:) = p .* s_locs(p,:);
    end
    
    samples = zeros(length(s_locs),1);
    for si = 1 : length(s_locs)
       samples(si) = field.z(s_locs(si,1), s_locs(si,2)); 
    end

    d = variogram(s_locs, samples, 'plot', false);
    [~,~,~,vstruct] = variogramfit(d.distance,d.val,5,100,[],'plotit',false);
    [xi, yi] = meshcoor([1 1], field.width, field.width);

    % Running Kriging prediction
    [zi,s2zi] = kriging(vstruct, s_locs(:,1),s_locs(:,2), ...
    samples, ...
    xi, yi, ...
    false, ...
    100);

    ap_mean_pred_err = 0;

    for ix = 1:field.width^2
        ap_mean_pred_err = ap_mean_pred_err + sqrt((zi(ix) - field.z(xi(ix),yi(ix)))^2);
    end
    
    ap_mean_pred_err = ap_mean_pred_err / length(zi);
    ap_mean_var = mean(s2zi);
end

