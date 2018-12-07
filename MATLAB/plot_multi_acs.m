%cd test/

clear all;
close all;
clc;

save_plots = true;

addpath(genpath(pwd));

mat_file_dir = 'test/';
save_to_dir = 'paper_plots/';

data_file_prefixes = {'mc', 'nnhv', 'nhv', 'greedy', 'zz'};
apriori_points = 10;

scan_percentages = [10 20 30]; % lowest to highest
max_p = num2str(scan_percentages(end));
field_width = 100;
sigma_fields = [100]% 50 100];
seed = 1;

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
   [ap_mean_var, ap_mean_pred_err] = apriori_errors( field.field, apriori_points );
   
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
            semilogy(100*percentages, vars ./ ap_mean_var, '--', 'LineWidth', 1.5, ...
                'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}', '$']))
        else    
            semilogy(100*percentages, vars ./ ap_mean_var, '--', 'LineWidth', 1.5, ...
                'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}',' - \sigma_{field}=', num2str(sigma_fields(sf_ix)), '$']))
        end
        
        hold on;
        
        figure(2);
        if (length(sigma_fields) == 1)
            semilogy(100*percentages, errors ./ ap_mean_pred_err, '--','LineWidth', 1.5, ...
            'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}', '$']))    
        else
            semilogy(100*percentages, errors ./ ap_mean_pred_err, '--','LineWidth', 1.5, ...
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
        elseif strcmp('greedy', prefix)
            plot_name = 'NBV';
        elseif strcmp('mc', prefix)
            plot_name = 'MCPP';
        else
            plot_name = upper(prefix);
        end

        figure(1);
        if (length(sigma_fields) == 1)
            semilogy(100*percentages, vars ./ ap_mean_var, 'LineWidth', 1.5, ...
                'DisplayName', plot_name)
        else
            semilogy(100*percentages, vars ./ ap_mean_var, 'LineWidth', 1.5, ...
                'DisplayName', strcat([plot_name,' - $\sigma_{field}=', num2str(sigma_fields(sf_ix)),'$']))
        end
        
        hold on;

        figure(2);
        
        if (length(sigma_fields) == 1)
            semilogy(100*percentages, errors ./ ap_mean_pred_err, 'LineWidth', 1.5, ...
                'DisplayName', plot_name)
        else
            semilogy(100*percentages, errors ./ ap_mean_pred_err, 'LineWidth', 1.5, ...
                'DisplayName', strcat([plot_name,' - $\sigma_{field}=', num2str(sigma_fields(sf_ix)),'$']))
        end
        
        hold on;
    end
   end
end

figure(1);
leg = legend('show');
set(leg, 'Interpreter', 'Latex');
set(leg, 'FontSize', 12)  

if (length(sigma_fields) == 1)
    title(strcat(['Normalized Variance Comparison - $\sigma_{field} = ', num2str(sigma_fields(1)), '$']), 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, 'normalized_variances_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_app_', num2str(apriori_points)]);
else
    title('Normalized Variance Comparison', 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, 'normalized_variances_', max_p, 'p_', ...
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

if (length(sigma_fields) == 1)
    title(strcat(['Normalized Prediction Error Comparison - $\sigma_{field} = ', num2str(sigma_fields(1)), '$']), 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, 'normalized_errors_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sf_ix)), ...
            '_seed_', num2str(seed) ,'_app_', num2str(apriori_points)]);
else
    title('Normalized Prediction Error Comparison', 'Interpreter', 'Latex', 'FontSize', 18)
    save_name = strcat([save_to_dir, 'normalized_errors_', max_p, 'p_', ...
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