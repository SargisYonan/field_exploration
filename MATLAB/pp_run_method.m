function [ ] = pp_run_method( method, rand_seed, field_size, percentage_scan, sigma_field )

% A script to run a sim of a given method
%
% @author Sargis S Yonan
% @date August 2017
%

% 
% Department of Computer Engineering
% Autonomous Systems Lab
% University of California, Santa Cruz
%
% Last Revised December 2018

%% load the libraries in the pwd
disp('Loading UAV libraries into path...')
disp('Loading Field libraries into path...')

addpath(genpath(pwd))

fprintf('Method: %s\n', method)
fprintf('Field Width: %d\n', field_size)
fprintf('Percentage Scan: %f\n', percentage_scan)
fprintf('Sigma Field: %d\n', sigma_field)

rng(rand_seed);
field_dir = 'fields/';

wps = 0;
perc = [];
pred_errs = [];
avg_vars = [];
    
% generate string to save images
filename = strcat(['_', num2str(100 * percentage_scan), ...
'p_' num2str(field_size), 'x', num2str(field_size), ...
'_sf_', num2str(sigma_field), '_seed_', num2str(rand_seed), '.png']);

% generate string to store data files
savename = strcat(['_', num2str(100 * percentage_scan), ...
'p_' num2str(field_size), 'x', num2str(field_size), ...
'_sf_', num2str(sigma_field), '_seed_', num2str(rand_seed), '_data']);

% load the pregenerated field
field_load_name = strcat([field_dir, 'field_', num2str(field_size), 'x', ...
    num2str(field_size), '_sf_', num2str(sigma_field), '_seed_', num2str(rand_seed)]);
field = load(field_load_name);
field = field.field;

[ap_vars, ap_errs] = apriori_errors(field);

if strcmp(method, 'zz')
    [wps, pred_field_recs, var_field_recs, perc] = zigzag_explore([1 1], field, percentage_scan, false, strcat([method, filename]));
    [avg_vars, pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:wps), pred_field_recs(:,:,1:wps), field.z);

else
    [wps, pred_field_recs, var_field_recs, perc] = kriging_field_explore([1 1], field, method, percentage_scan, true, strcat([method, filename]));
    [avg_vars, pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:wps), pred_field_recs(:,:,1:wps), field.z);
end

save(strcat([method, '_percs', savename]), 'perc');
save(strcat([method, '_pred_errs', savename]), 'pred_errs');
save(strcat([method, '_vars', savename]), 'avg_vars');
normalized = avg_vars ./ ap_vars;
save(strcat([method, '_normalized_vars', savename]), 'normalized');

exit
end

