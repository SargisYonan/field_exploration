function [ ] = pp_run( rand_seeds, field_sizes, max_percentage_list, autocorrelation_sigma_dividers )

% A script to compare three custom path planners against a lawnmower
% explorer.
%
% @author Sargis S Yonan
% @date August 2017
%

% 
% Department of Computer Engineering
% Autonomous Systems Lab
% University of California, Santa Cruz
%
% Last Revised October 2018

%% load the libraries in the pwd
disp('Loading UAV libraries into path...')
disp('Loading Field libraries into path...')
addpath(genpath(pwd))

%%
for fi = 1 : length(rand_seeds)
rng(rand_seeds(fi));
for field_size_ix = 1 : length(field_sizes)
field_size = field_sizes(field_size_ix);

autocorrelation_sigmas = zeros(length(autocorrelation_sigma_dividers), 1);
for sitor = 1 : length(autocorrelation_sigma_dividers)
    autocorrelation_sigmas(sitor) = field_size / autocorrelation_sigma_dividers(sitor);
end

for autocor_sigma_itor = 1 : length(autocorrelation_sigmas)
autocorrelation_sigma = autocorrelation_sigmas(autocor_sigma_itor);

for percentage_ix = 1:size(max_percentage_list,2)
    
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

savename = strcat(['_', num2str(100 * max_percentage), ...
'p_' num2str(field_size), 'x', num2str(field_size), ...
'_sf_', num2str(autocorrelation_sigma), '_seed_', num2str(rand_seeds(fi)), '_data']);

field = Field(field_size, field_size, autocorrelation_sigma);
save(strcat(['field_', num2str(field_size), 'x', num2str(field_size), ...
'_sf_', num2str(autocorrelation_sigma), '_seed_', num2str(rand_seeds(fi))]), ...
'field');

[ap_vars, ap_errs] = apriori_errors(field);

[nhv_wps, pred_field_recs, var_field_recs, nhv_perc] = kriging_field_explore([1 1], field, 'nhv', max_percentage, false, strcat(['nhv', filename]));
[nhv_avg_vars, nhv_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:nhv_wps), pred_field_recs(:,:,1:nhv_wps), field.z);
perc = nhv_perc;
save(strcat(['nhv', '_percs', savename]), 'perc');
pred_errs = nhv_pred_errs;
save(strcat(['nhv', '_pred_errs', savename]), 'pred_errs');
avg_vars = nhv_avg_vars;
save(strcat(['nhv', '_vars', savename]), 'avg_vars');
normalized = mcpp_avg_vars ./ ap_vars;
save(strcat(['nhv', '_normalized_vars', savename]), 'normalized');

[nnhv_wps, pred_field_recs, var_field_recs, nnhv_perc] = kriging_field_explore([1 1], field, 'nnhv', max_percentage, false, strcat(['nnhv', filename]));
[nnhv_avg_vars, nnhv_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:nnhv_wps), pred_field_recs(:,:,1:nnhv_wps), field.z);
perc = nnhv_perc;
save(strcat(['n-nhv', '_percs', savename]), 'perc');
pred_errs = nnhv_pred_errs;
save(strcat(['n-nhv', '_pred_errs', savename]), 'pred_errs');
avg_vars = nnhv_avg_vars;
save(strcat(['n-nhv', '_vars', savename]), 'avg_vars');
normalized = mcpp_avg_vars ./ ap_vars;
save(strcat(['n-nhv', '_normalized_vars', savename]), 'normalized');

[mcpp_wps, pred_field_recs, var_field_recs, mcpp_perc] = kriging_field_explore([1 1], field, 'mc', max_percentage, false, strcat(['mc', filename]));
[mcpp_avg_vars, mcpp_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:mcpp_wps), pred_field_recs(:,:,1:mcpp_wps), field.z);
perc = mcpp_perc;
save(strcat(['mc', '_percs', savename]), 'perc');
pred_errs = mcpp_pred_errs;
save(strcat(['mc', '_pred_errs', savename]), 'pred_errs');
avg_vars = mcpp_avg_vars;
save(strcat(['mc', '_vars', savename]), 'avg_vars');
normalized = mcpp_avg_vars ./ ap_vars;
save(strcat(['mc', '_normalized_vars', savename]), 'normalized');

[zz_wps, pred_field_recs, var_field_recs, zz_perc] = zigzag_explore([1 1], field, max_percentage, false, strcat(['zz', filename]));
[zz_avg_vars, zz_pred_errs] = kriging_run_process_results(var_field_recs(:,:,1:zz_wps), pred_field_recs(:,:,1:zz_wps), field.z);
perc = zz_perc;
save(strcat(['zz', '_percs', savename]), 'perc');
pred_errs = zz_pred_errs;
save(strcat(['zz', '_pred_errs', savename]), 'pred_errs');
avg_vars = zz_avg_vars;
save(strcat(['zz', '_vars', savename]), 'avg_vars');
normalized = zz_avg_vars ./ ap_vars;
save(strcat(['zz', '_normalized_vars', savename]), 'normalized');

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

legend(['Zig-Zag Method'], ['HV'], ['N-HV'], ['MCPP'])

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

legend(['Zig-Zag Method'], ['HV'], ['N-HV'], ['MCPP'])

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
fprintf('HV:\t%.4f\n', m(end))

m = nnhv_pred_errs;
fprintf('N-HV:\t%.4f\n', m(end))

m = mcpp_pred_errs;
fprintf('MCPP:\t%.4f\n', m(end))

fprintf('\nFinal Variance -- Percentage: %.2f%%\n', max_percentage * 100)

m = zz_avg_vars;
fprintf('ZZ:\t%.4f\n', m(end) ./ ap_vars)

m = nhv_avg_vars;
fprintf('HV:\t%.4f\n', m(end) ./ ap_vars)

m = nnhv_avg_vars;
fprintf('N-HV:\t%.4f\n', m(end) ./ ap_vars)

m = mcpp_avg_vars;
fprintf('MCPP:\t%.4f\n', m(end) ./ ap_vars)

grid on

end
end
end


exit
end

