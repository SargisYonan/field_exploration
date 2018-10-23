function [ avg_vars, avg_pred_errs ] = kriging_run_process_results( var_field_recs, pred_field_recs, actual_field )

avg_vars = zeros(1, size(var_field_recs, 3));
avg_pred_errs = zeros(1, size(pred_field_recs, 3));

for i = 1 : size(var_field_recs, 3)
    % average variance of each field
    avg_vars(i) = mean2(var_field_recs(:,:,i));
end

for i = 1 : size(pred_field_recs, 3)     
    % average element wise RMS errors
    pred_field = var_field_recs(:,:,i);
    avg_pred_errs(i) = sqrt(mean(abs(pred_field(:).^2 - actual_field(:).^2)));
    
end

end