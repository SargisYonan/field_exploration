%cd test/

data_file_prefixes = {'mc', 'nnhv', 'nhv', 'greedy', 'zz'};

scan_percentages = [10 20 30]; % lowest to highest
max_p = num2str(scan_percentages(end));
field_width = 100;
sigma_fields = [100];
seed = 1;

vars = figure(1);
errs = figure(2);

for dix = 1 : size(data_file_prefixes,2)
   prefix = data_file_prefixes{dix};
   
   % do all ps
   if (strcmp(prefix, 'zz'))
       for p = 1 : length(scan_percentages)
        this_p = num2str(scan_percentages(p));
        
        normalized_vars = load(strcat([prefix, ...
            '_normalized_vars_', this_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sfix)), ...
            '_seed_', num2str(seed) ,'_data']));
        normalized_vars = normalized_vars.normalized;
        
        percentages = load(strcat([prefix, ...
            '_percs_', this_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sfix)), ...
            '_seed_', num2str(seed) ,'_data']));   
        percentages = percentages.perc;
        
        errors = load(strcat([prefix, ...
            '_pred_errs_', this_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sfix)), ...
            '_seed_', num2str(seed) ,'_data']));
        errors = errors.pred_errs;
        
        figure(1);
        semilogy(100*percentages, normalized_vars, 'LineWidth', 1.5, ...
            'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}',' - \sigma_{field}=', num2str(sigma_fields(sfix)), '$']))
        hold on;
        
        figure(2);
        plot(100*percentages, errors, 'LineWidth', 1.5, ...
            'DisplayName', strcat(['$',upper(prefix),'_{',num2str(this_p),'\%}',' - \sigma_{field}=', num2str(sigma_fields(sfix)), '$']))
        hold on;
       end
   else
       
   for sfix = 1 : length(sigma_fields)
        
        normalized_vars = load(strcat([prefix, ...
            '_normalized_vars_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sfix)), ...
            '_seed_', num2str(seed) ,'_data']));
        normalized_vars = normalized_vars.normalized;

        percentages = load(strcat([prefix, ...
            '_percs_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sfix)), ...
            '_seed_', num2str(seed) ,'_data']));   
        percentages = percentages.perc;
        
        errors = load(strcat([prefix, ...
            '_pred_errs_', max_p, 'p_', ...
            num2str(field_width),'x',num2str(field_width), ...
            '_sf_', num2str(sigma_fields(sfix)), ...
            '_seed_', num2str(seed) ,'_data']));   
        errors = errors.pred_errs;
        
        % map names
        if strcmp('nhv', prefix)
            plot_name = 'HV';
        elseif strcmp('nnhv', prefix)
            plot_name = '$N$-HV$';
        elseif strcmp('greedy', prefix)
            plot_name = 'NBV';
        else
            plot_name = upper(prefix);
        end
        
        figure(1);
        semilogy(100*percentages, normalized_vars, 'LineWidth', 1.5, ...
            'DisplayName', strcat(['$',plot_name,' - \sigma_{field}=', num2str(sigma_fields(sfix)),'$']))
        hold on;
        
        figure(2);
        plot(100*percentages, errors, 'LineWidth', 1.5, ...
            'DisplayName', strcat(['$',plot_name,' - \sigma_{field}=', num2str(sigma_fields(sfix)),'$']))
        hold on;
    end
    
   end
end

figure(1);
leg = legend('show');
set(leg, 'Interpreter', 'Latex');
set(leg, 'FontSize', 16)  
title('Normalized Variance Comparison', 'Interpreter', 'Latex', 'FontSize', 18)
xlabel('Percent Scanned', 'FontSize', 18, 'Interpreter', 'Latex')
ylabel('$\log(\Sigma(\hat{Z}))$', 'FontSize', 18, 'Interpreter', 'Latex')
grid on;
    
figure(2);
leg = legend('show');
set(leg, 'Interpreter', 'Latex');
set(leg, 'FontSize', 16)  
title('Normalized Variance Comparison', 'Interpreter', 'Latex', 'FontSize', 18)
xlabel('Percent Scanned', 'FontSize', 18, 'Interpreter', 'Latex')
ylabel('$E(Z,\hat{Z})$', 'FontSize', 18, 'Interpreter', 'Latex')
grid on;