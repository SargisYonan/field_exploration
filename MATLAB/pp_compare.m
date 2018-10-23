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
field_sizes = [50]; %[20, 50];
max_percentage_list =  [[.5, .75]];%[[0.20, 0.30, 0.40, 0.50, 0.75]; 
                       %[0.10, 0.20, 0.50, 0.75]];

for field_size_ix = 1 : length(field_sizes)
field_size = field_sizes(field_size_ix);
for percentage_ix = 1:length(max_percentage_list)
    
%% Percent Scan Max
max_percentage = max_percentage_list(field_size_ix, percentage_ix);

%% Generate n_fields number of fields of size field_size
n_fields = 5;

%% Compare explorers
nhv_wps = zeros(1, n_fields);
nnhv_wps = zeros(1, n_fields);
mcpp_wps = zeros(1, n_fields);
zz_wps = zeros(1, n_fields);


nhv_avg_vars = [];
nnhv_avg_vars = [];
mcpp_avg_vars = [];
zz_avg_vars = [];
for fi = 1 : n_fields
    field = Field(field_size, field_size);
    
    [nhv_wps(fi), pred_field_recs, var_field_recs] = kriging_field_explore([1 1], field, 'nhv', max_percentage, false, '');
    [nhv_avg_vars(fi,1:nhv_wps(fi)), nhv_pred_errs(fi,1:nhv_wps(fi))] = kriging_run_process_results(var_field_recs(:,:,1:nhv_wps(fi)), pred_field_recs(:,:,1:nhv_wps(fi)), field.z);
   
    [nnhv_wps(fi), pred_field_recs, var_field_recs] = kriging_field_explore([1 1], field, 'nnhv', max_percentage, false, '');
    [nnhv_avg_vars(fi,1:nnhv_wps(fi)), nnhv_pred_errs(fi,1:nnhv_wps(fi))] = kriging_run_process_results(var_field_recs(:,:,1:nnhv_wps(fi)), pred_field_recs(:,:,1:nnhv_wps(fi)), field.z);
    
    [mcpp_wps(fi), pred_field_recs, var_field_recs] = kriging_field_explore([1 1], field, 'mc', max_percentage, false, '');
    [mcpp_avg_vars(fi,1:mcpp_wps(fi)), mcpp_pred_errs(fi,1:mcpp_wps(fi))] = kriging_run_process_results(var_field_recs(:,:,1:mcpp_wps(fi)), pred_field_recs(:,:,1:mcpp_wps(fi)), field.z);
    
    [zz_wps(fi), pred_field_recs, var_field_recs] = zigzagrouter([field_size 1], field, (max_percentage * field_size) / 2, max_percentage, false);
    [zz_avg_vars(fi,1:zz_wps(fi)), zz_pred_errs(fi,1:zz_wps(fi))] = kriging_run_process_results(var_field_recs(:,:,1:zz_wps(fi)), pred_field_recs(:,:,1:zz_wps(fi)), field.z);
    
end

%%
figure();

semilogy(nonzero_means(zz_avg_vars) ./ zz_avg_vars(1), '-k', 'LineWidth', 2)
hold on
semilogy(nonzero_means(nhv_avg_vars) ./ nhv_avg_vars(1), '-r', 'LineWidth', 2)
hold on
semilogy(nonzero_means(nnhv_avg_vars) ./ nnhv_avg_vars(1), '-g', 'LineWidth', 2)
hold on
semilogy(nonzero_means(mcpp_avg_vars) ./ mcpp_avg_vars(1), '-b', 'LineWidth', 2)

xlabel('Replanning Iteration', 'FontSize', 16, 'Interpreter', 'Latex')
ylabel('Mean Field Prediction Variance', 'FontSize', 16, 'Interpreter', 'Latex')

t = strcat(['Variance Comparison (', num2str(100 * max_percentage), ...
    '\% Scan - ' num2str(field_size), 'x', num2str(field_size), ' Field)']);
title(t, 'FontSize', 20, 'Interpreter', 'Latex')

legend(['Zig-Zag Method'], ['NHV'], ['N-NHV'], ['MCPP'])

xlim([xlim] + [1 0]);

grid on
%% 
figure();

plot(nonzero_means(zz_pred_errs), '-k', 'LineWidth', 2)
hold on
plot(nonzero_means(nhv_pred_errs), '-r', 'LineWidth', 2)
hold on
plot(nonzero_means(nnhv_pred_errs), '-g', 'LineWidth', 2)
hold on
plot(nonzero_means(mcpp_pred_errs), '-b', 'LineWidth', 2)

xlabel('Replanning Iteration', 'FontSize', 14, 'Interpreter', 'Latex')
ylabel('Mean RMS Error', 'FontSize', 14, 'Interpreter', 'Latex')

t = strcat(['Error Between Actual and Predicted Fields (', num2str(100 * max_percentage), ...
    '\% Scan - ' num2str(field_size), 'x', num2str(field_size), ' Field)']);
title(t, 'FontSize', 15, 'Interpreter', 'Latex')

legend(['Zig-Zag Method'], ['NHV'], ['N-NHV'], ['MCPP'])

xlim([xlim] + [1 0]);

grid on

%%
fprintf('\n----------------------------------------------\n')

fprintf('\nFinal Prediction Error -- Percentage: %.2f%%\n', max_percentage * 100)

m = nonzero_last_means(zz_pred_errs);
fprintf('ZZ:\t%.4f\n', m(end))

m = nonzero_last_means(nhv_pred_errs);
fprintf('NHV:\t%.4f\n', m(end))

m = nonzero_last_means(nnhv_pred_errs);
fprintf('N-NHV:\t%.4f\n', m(end))

m = nonzero_last_means(mcpp_pred_errs);
fprintf('MCPP:\t%.4f\n', m(end))

fprintf('\nFinal Variance -- Percentage: %.2f%%\n', max_percentage * 100)

m = nonzero_last_means(zz_avg_vars);
fprintf('ZZ:\t%.4f\n', m(end))

m = nonzero_last_means(nhv_avg_vars);
fprintf('NHV:\t%.4f\n', m(end))

m = nonzero_last_means(nnhv_avg_vars);
fprintf('N-NHV:\t%.4f\n', m(end))

m = nonzero_last_means(mcpp_avg_vars);
fprintf('MCPP:\t%.4f\n', m(end))

grid on

end
end