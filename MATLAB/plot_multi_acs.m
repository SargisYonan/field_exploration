%cd test/

clear all;
close all;
clc;

save_plots = true;

addpath(genpath(pwd));

mat_file_dir = 'test/';
save_to_dir = 'paper_plots/';

index_offset = 1;
% data_file_prefixes = {'mc', 'nnhv', 'nhv', 'greedy', 'zz'};
%  data_file_prefixes = {'mc', 'gradient', 'gr', 'nhv', 'nnhv', 'zz', 'nbv'};
% data_file_prefixes = {'mc', 'gradient', 'gr', 'nhv', 'nnhv', 'zz'};

data_file_prefixes = {'mc', 'zz'};

scan_percentages = [10 20 30]; % lowest to highest

max_p = num2str(scan_percentages(end));
field_width = 100;
sigma_fields = [1 50 100];
seed = 2;

vars = figure(1);
errs = figure(2);

format long

for dix = 1 : size(data_file_prefixes,2)
   prefix = data_file_prefixes{dix};
   for sf_ix = 1 : length(sigma_fields)   
   % do all ps
   
   field = load(strcat(['fields/field_', ...
       num2str(field_width),'x',num2str(field_width), ...
       '_sf_', num2str(sigma_fields(sf_ix)), ...
       '_seed_', num2str(seed)]));
      
   if (length(sigma_fields) == 1)
        zz_r = 1 / max(scan_percentages);
        zz_r = zz_r / 100;
        initial_waypoint = ceil((field_width/2)-zz_r);

        apriori_points = initial_waypoint;
        [ap_mean_var, ap_mean_pred_err] = apriori_errors( field.field, apriori_points );
        %% todo
        ap_mean_pred_err = sqrt(ap_mean_var)
        
    else
        apriori_points = 0;
        ap_mean_var = 1;
        ap_mean_pred_err = 1;
        
        %% todo
        zz_r = 1 / max(scan_percentages);
        zz_r = zz_r / 100;
        initial_waypoint = ceil((field_width/2)-zz_r);

        apriori_points = initial_waypoint;
        [ap_mean_var, ap_mean_pred_err] = apriori_errors( field.field, apriori_points );
        ap_mean_pred_err = sqrt(ap_mean_var)
   end

   if (strcmp(prefix, 'zz'))
       for p = 1 : length(scan_percentages)
        this_p = num2str(scan_percentages(p));
        
        vars = load(strcat([mat_file_dir, prefix, ...
            '_vars_', this_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_data']));
        vars = vars.avg_vars;
        
        percentages = load(strcat([mat_file_dir, prefix, ...
            '_percs_', this_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_data']));   
        percentages = percentages.perc;
        
        errors = load(strcat([mat_file_dir, prefix, ...
            '_pred_errs_', this_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_data']));
        errors = errors.pred_errs;
        
        figure(1);
        if (length(sigma_fields) == 1)
            semilogy(100*percentages(index_offset:end), vars(index_offset:end) ./ ap_mean_var, '--', 'LineWidth', 1.8, ...
                'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}', '$']))
        else    
            semilogy(100*percentages(index_offset:end), vars(index_offset:end) ./ ap_mean_var, '--', 'LineWidth', 1.8, ...
                'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}',' - \sigma_{field}=', num2str(sigma_fields(sf_ix)), '$']))
        end
        
        hold on;
        
        figure(2);
        if (length(sigma_fields) == 1)
            semilogy(100*percentages(index_offset:end), errors(index_offset:end) ./ ap_mean_pred_err, '--','LineWidth', 1.8, ...
            'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}', '$']))    
        else
            semilogy(100*percentages(index_offset:end), errors(index_offset:end) ./ ap_mean_pred_err, '--','LineWidth', 1.8, ...
            'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}',' - \sigma_{field}=', num2str(sigma_fields(sf_ix)), '$']))
        end
        
        hold on;
       end
   else

        vars = load(strcat([mat_file_dir, prefix, ...
            '_vars_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_data']));
        vars = vars.avg_vars;

        percentages = load(strcat([mat_file_dir, prefix, ...
            '_percs_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_data']));   
        percentages = percentages.perc;

        errors = load(strcat([mat_file_dir, prefix, ...
            '_pred_errs_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_data']));   
        errors = errors.pred_errs;

        % map names
        if strcmp('nhv', prefix)
            plot_name = 'HV';
        elseif strcmp('nnhv', prefix)
            plot_name = '$N$-HV';
        elseif strcmp('gradient', prefix)
            plot_name = 'GA';
        elseif strcmp('gr', prefix)
            plot_name = 'RGA';
        elseif strcmp('nbv', prefix)
            plot_name = 'NBV';
        elseif strcmp('mc', prefix)
            plot_name = 'MCPP';
        else
            plot_name = upper(prefix);
        end

        figure(1);
        if (length(sigma_fields) == 1)
            semilogy(100*percentages(index_offset:end), vars(index_offset:end) ./ ap_mean_var, 'LineWidth', 1.8, ...
                'DisplayName', plot_name)
        else
            semilogy(100*percentages(index_offset:end), vars(index_offset:end) ./ ap_mean_var, 'LineWidth', 1.8, ...
                'DisplayName', strcat([plot_name,' - $\sigma_{field}=', num2str(sigma_fields(sf_ix)),'$']))
        end
        
        hold on;

        figure(2);
        
        if (length(sigma_fields) == 1)
            semilogy(100*percentages(index_offset:end), errors(index_offset:end) ./ ap_mean_pred_err, 'LineWidth', 1.8, ...
                'DisplayName', plot_name)
        else
            semilogy(100*percentages(index_offset:end), errors(index_offset:end) ./ ap_mean_pred_err, 'LineWidth', 1.8, ...
                'DisplayName', strcat([plot_name,' - $\sigma_{field}=', num2str(sigma_fields(sf_ix)),'$']))
        end
        
        hold on;
    end
   end
end

if (apriori_points == 0)
    normalized_ornot = '';
    normie = '';
else
    normalized_ornot = 'Normalized ';
    normie = 'normalized_';
end
figure(1);
leg = legend('show');
set(leg, 'Interpreter', 'Latex');
set(leg, 'FontSize', 12)  
set(leg, 'Orientation', 'vertical');

if (length(sigma_fields) == 1)
    title(strcat([normalized_ornot, 'Variance Comparison - $\sigma_{field} = ', num2str(sigma_fields(1)), '$']), 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, normie, 'variances_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_app_', num2str(apriori_points)]);
else
    title(strcat([normalized_ornot, 'Variance Comparisons for Varying Values of $\sigma_{field}$']), 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, normie, 'variances_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', 'all', ...
            '_seed_', num2str(seed) ,'_app_', num2str(apriori_points)]);
end

xlabel('Percent Scanned', 'FontSize', 18, 'Interpreter', 'Latex')
ylabel('$\log(\Sigma(\hat{Z}))$', 'FontSize', 18, 'Interpreter', 'Latex')
grid on;
axis([0 100*max(percentages) ylim]);    
if (save_plots)
    saveas(gcf, save_name, 'png')
end

figure(2);
leg = legend('show');
set(leg, 'Interpreter', 'Latex');
set(leg, 'FontSize', 12)  
set(leg, 'Orientation', 'vertical');

if (length(sigma_fields) == 1)
    title(strcat([normalized_ornot, 'Prediction Error Comparison - $\sigma_{field} = ', num2str(sigma_fields(1)), '$']), 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, normie, 'errors_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_app_', num2str(apriori_points)]);
else
    title(strcat([normalized_ornot, 'Prediction Error Comparisons for Varying Values of $\sigma_{field}$']), 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, normie, 'errors_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', 'all', ...
            '_seed_', num2str(seed) ,'_app_', num2str(apriori_points)]);
end

xlabel('Percent Scanned', 'FontSize', 18, 'Interpreter', 'Latex')
ylabel('$\log(E(Z,\hat{Z}))$', 'FontSize', 18, 'Interpreter', 'Latex')
grid on;
axis([0 100*max(percentages) ylim]);
if (save_plots)
    saveas(gcf, save_name, 'png')
end