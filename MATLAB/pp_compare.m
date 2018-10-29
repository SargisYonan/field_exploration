% A script to compare three custom path planners against a lawnmower
% explorer.
%
% @author Sargis S Yonan
% @date August 2017
%
% @brief Simulation for Computer Engineering Masters Thesis
% 
% Department of Computer Engineering
% Autonomous Systems Lab
% University of California, Santa Cruz
%
% Last Revised October 2018

clc;
clear all;
close all;

%% load the libraries in the pwd
disp('Loading UAV libraries into path...')
disp('Loading Field libraries into path...')
addpath(genpath(pwd))

%%

rand_seeds = [2] %[2, 3, 4];
field_sizes = [50]%[50, 100];
max_percentage_list =  [[0.10]]% 0.20]]%[[0.10, 0.20, 0.30]; 
                       %[0.10, 0.20, 0.30]];       

%%


%%
for fi = 1 : length(rand_seeds)
rng(rand_seeds(fi));
for field_size_ix = 1 : length(field_sizes)
field_size = field_sizes(field_size_ix);
autocorrelation_sigmas = [round(field_size/8)]% round(field_size/4) round(field_size/2) field_size];

for autocor_sigma_itor = 1 : length(autocorrelation_sigmas)
autocorrelation_sigma = autocorrelation_sigmas(autocor_sigma_itor);

for percentage_ix = 1:length(max_percentage_list)
    
%% Percent Scan Max
max_percentage = max_percentage_list(field_size_ix, percentage_ix);

%% Compare explorers
nhv_wps = 0;
nnhv_wps = 0;
mcpp_wps = 0;
zz_wps = 0;

nhv_perc = [];
nnhv_perc = [];
mcpp_perc = [];
zz_perc = [];

nhv_pred_errs = [];
nnhv_pred_errs = [];
mcpp_pred_errs = [];
zz_pred_errs = [];

nhv_avg_vars = [];
nnhv_avg_vars = [];
mcpp_avg_vars = [];
zz_avg_vars = [];

    
filename = strcat(['_', num2str(100 * max_percentage), ...
'p_' num2str(field_size), 'x', num2str(field_size), ...
'_sf_', num2str(autocorrelation_sigma), '_seed_', num2str(rand_seeds(fi)), '.png']);

field = Field(field_size, field_size, autocorrelation_sigma);

[nhv_wps, pred_field_recs, var_field_recs, nhv_perc] = kriging_field_explore([1 1], field, 'nhv', max_percentage, false, strcat(['nhv', filename]));
[nhv_avg_vars, nhv_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:nhv_wps), pred_field_recs(:,:,1:nhv_wps), field.z);

[nnhv_wps, pred_field_recs, var_field_recs, nnhv_perc] = kriging_field_explore([1 1], field, 'nnhv', max_percentage, false, strcat(['nnhv', filename]));
[nnhv_avg_vars, nnhv_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:nnhv_wps), pred_field_recs(:,:,1:nnhv_wps), field.z);

[mcpp_wps, pred_field_recs, var_field_recs, mcpp_perc] = kriging_field_explore([1 1], field, 'mc', max_percentage, false, strcat(['mc', filename]));
[mcpp_avg_vars, mcpp_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:mcpp_wps), pred_field_recs(:,:,1:mcpp_wps), field.z);

if (max_percentage == 0.5)
    r = (max_percentage * field_size) / (15);
elseif (max_percentage == 0.3)
    r = (max_percentage * field_size) / (5);
elseif (max_percentage == 0.2)
    r = (max_percentage * field_size) / (2);
elseif (max_percentage == 0.1)
    r = (max_percentage * field_size) / (0.5);
end
[zz_wps, pred_field_recs, var_field_recs, zz_perc] = zigzagrouter([field_size 1], field, r, max_percentage, false, strcat(['zz', filename]));
[zz_avg_vars, zz_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:zz_wps), pred_field_recs(:,:,1:zz_wps), field.z);

%%

[ap_vars, ap_errs] = apriori_errors(field);

%%
figure();

semilogy(zz_perc, zz_avg_vars ./ ap_vars, '-k', 'LineWidth', 2)
hold on
semilogy(nhv_perc, nhv_avg_vars ./ ap_vars, '-r', 'LineWidth', 2)
hold on
semilogy(nnhv_perc, nnhv_avg_vars ./ ap_vars, '-g', 'LineWidth', 2)
hold on
semilogy(mcpp_perc, mcpp_avg_vars ./ ap_vars, '-b', 'LineWidth', 2)

xlabel('Field Scanned Percentage', 'FontSize', 16, 'Interpreter', 'Latex')
ylabel('Normalized Mean Field Prediction Variance', 'FontSize', 16, 'Interpreter', 'Latex')

t = strcat(['Mean Variances (', num2str(100 * max_percentage), ...
    '\% Scan - ' num2str(field_size), '$\times$', num2str(field_size), ...
    ' Field - $\sigma_{field} = $ ', num2str(autocorrelation_sigma), ')']);
title(t, 'FontSize', 18, 'Interpreter', 'Latex')

legend(['Zig-Zag Method'], ['NHV'], ['N-NHV'], ['MCPP'])

xlim([min([zz_perc(1) nhv_perc(1) nnhv_perc(1) mcpp_perc(1)]) max_percentage]);
grid on

saveas(gcf, strcat(['vars_', num2str(100 * max_percentage), ...
    'p_' num2str(field_size), 'x', num2str(field_size), ...
    '_sf_', num2str(autocorrelation_sigma), '_seed_', num2str(rand_seeds(fi)), '.png']));

%% 
figure();

plot(zz_perc, zz_pred_errs, '-k', 'LineWidth', 2)
hold on
plot(nhv_perc, nhv_pred_errs, '-r', 'LineWidth', 2)
hold on
plot(nnhv_perc, nnhv_pred_errs, '-g', 'LineWidth', 2)
hold on
plot(mcpp_perc, mcpp_pred_errs, '-b', 'LineWidth', 2)

xlabel('Field Scanned Percentage', 'FontSize', 14, 'Interpreter', 'Latex')
ylabel('Mean RMS Error', 'FontSize', 14, 'Interpreter', 'Latex')

t = strcat(['Prediction Errors (', num2str(100 * max_percentage), ...
    '\% Scan - ' num2str(field_size), '$\times$', num2str(field_size), ...
    ' Field - $\sigma_{field} = $ ', num2str(autocorrelation_sigma), ')']);
title(t, 'FontSize', 18, 'Interpreter', 'Latex')

legend(['Zig-Zag Method'], ['NHV'], ['N-NHV'], ['MCPP'])

xlim([min([zz_perc(1) nhv_perc(1) nnhv_perc(1) mcpp_perc(1)]) max_percentage]);
grid on

saveas(gcf, strcat(['pred_errs_', num2str(100 * max_percentage), ...
    'p_' num2str(field_size), 'x', num2str(field_size), ...
    '_sf_', num2str(autocorrelation_sigma), '_seed_', num2str(rand_seeds(fi)), '.png']));

%%
fprintf('\n----------------------------------------------\n')
fprintf('Seed: %d\n', rand_seeds(fi))
fprintf('sigma field: %f\n', autocorrelation_sigma)
fprintf('Field size: %d\n', field_size)

fprintf('\nFinal Prediction Error -- Percentage: %.2f%%\n', max_percentage * 100)

m = zz_pred_errs;
fprintf('ZZ:\t%.4f\n', m(end))

m = nhv_pred_errs;
fprintf('NHV:\t%.4f\n', m(end))

m = nnhv_pred_errs;
fprintf('N-NHV:\t%.4f\n', m(end))

m = mcpp_pred_errs;
fprintf('MCPP:\t%.4f\n', m(end))

fprintf('\nFinal Variance -- Percentage: %.2f%%\n', max_percentage * 100)

m = zz_avg_vars;
fprintf('ZZ:\t%.4f\n', m(end) ./ ap_vars)

m = nhv_avg_vars;
fprintf('NHV:\t%.4f\n', m(end) ./ ap_vars)

m = nnhv_avg_vars;
fprintf('N-NHV:\t%.4f\n', m(end) ./ ap_vars)

m = mcpp_avg_vars;
fprintf('MCPP:\t%.4f\n', m(end) ./ ap_vars)

grid on

end
end
end
end
