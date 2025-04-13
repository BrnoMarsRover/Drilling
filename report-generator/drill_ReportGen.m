clc; clear; close all;

%Čas vykonávání smyčky v node drill_controller
loop_time = 1;

% Získání seznamu souborů ve složce
files = dir('drillData*.txt');

% Pro každý soubor
for k = 1:length(files)
    filename = files(k).name;
    pdf_name = strrep(filename, '.txt', '.pdf');
    fid = fopen(filename, 'r');
    data_raw = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    data_raw = data_raw{1};
    
    % Nalezení Drill samplů 
    idx_all = find(contains(data_raw, 'Action DrillSample'));
    count = 1;
    if length(idx_all) >= 1
        i = idx_all(1);
    else
        break;
    end
    [rows, ~] = size(data_raw);

    % Pro každý drill sample
    while (i < (rows-4))
        if contains(data_raw{i}, 'DrillSample')
            values = split(data_raw(i+1), ';');
            values(end) = [];
            values = split(values, ',');
            values = str2double(values); 
            [howMany, ~] = size(values);
            t = 0:loop_time:loop_time*(howMany-1);
            torque = values(:, 1);
            rps = values(:, 2);
            temperature = values(:,3);
            height = values(:, 3); % new version: 4
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %temperature simulation, this is temporrary
            n = length(temperature); % počet vzorků
            start_temp = 23;
            end_temp = 35;
            % Simulovaný nárůst teploty s mírným šumem a nelineárním charakterem
            temperature = linspace(start_temp, end_temp, n);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Vykreslení grafů
            f = figure;
            f.Units = "centimeters";
            f.Position = [0, 0, 21, 29.7];
            tiledlayout(3,2, 'TileSpacing', 'loose', 'Padding', 'compact');

            %logo
            nexttile; axis off;
            nexttile;
            logo = imread('logo.bmp');
            imshow(logo);


            % --- GRAF 1 ---
            nexttile;
            plot(t, torque, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Torque [Nm]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Drilling torque', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            
            set(gca, ...
                'FontSize', 11, ...
                'GridColor', [0.5 0.5 0.5], ...
                'GridAlpha', 0.3, ...
                'LineWidth', 1.2, ...
                'Box', 'on', ...
                'XColor', [0.4 0.1 0.1], ...
                'YColor', [0.4 0.1 0.1]);
            
            set(gca, 'Color', [1 0.96 0.94]);
            
            % --- GRAF 2 ---
            nexttile;
            plot(t, rps, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('RPS [R/s]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Drill bit rotation speed', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            
            set(gca, ...
                'FontSize', 11, ...
                'GridColor', [0.5 0.5 0.5], ...
                'GridAlpha', 0.3, ...
                'LineWidth', 1.2, ...
                'Box', 'on', ...
                'XColor', [0.4 0.1 0.1], ...
                'YColor', [0.4 0.1 0.1]);
            
            set(gca, 'Color', [1 0.96 0.94]);
            
            % --- GRAF 3 ---
            nexttile;
            plot(t, temperature, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Temperature [°C]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Motor temperature', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            
            set(gca, ...
                'FontSize', 11, ...
                'GridColor', [0.5 0.5 0.5], ...
                'GridAlpha', 0.3, ...
                'LineWidth', 1.2, ...
                'Box', 'on', ...
                'XColor', [0.4 0.1 0.1], ...
                'YColor', [0.4 0.1 0.1]);
            
            set(gca, 'Color', [1 0.96 0.94]);
            
            % --- GRAF 4 ---
            nexttile;
            plot(t, height, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Height [mm]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Vertical position of the drill', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            
            set(gca, ...
                'FontSize', 11, ...
                'GridColor', [0.5 0.5 0.5], ...
                'GridAlpha', 0.3, ...
                'LineWidth', 1.2, ...
                'Box', 'on', ...
                'XColor', [0.4 0.1 0.1], ...
                'YColor', [0.4 0.1 0.1]);
            
            set(gca, 'Color', [1 0.96 0.94]);

            % --- Textová anotace 1 ---
            annotation('textbox', [0.05, 0.8, 0.8, 0.05], 'String', data_raw{i}, ...
                'EdgeColor', 'none', ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 12, ...
                'FontWeight', 'bold', ...
                'Color', 'k'); % Černá barva
            
            % --- Textová anotace 2 ---
            annotation('textbox', [0.05, 0.77, 0.8, 0.05], 'String', data_raw{i+2}, ...
                'EdgeColor', 'none', ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 12, ...
                'FontWeight', 'bold', ...
                'Color', 'k'); % Černá barva
            
            % --- Textová anotace s podmínkou ---
            if (contains(data_raw{i+4}, 'StoreSample'))
                annotation('textbox', [0.05, 0.74, 0.8, 0.05], 'String', data_raw{i+5}, ...
                    'EdgeColor', 'none', ...
                    'HorizontalAlignment', 'left', ...
                    'FontSize', 12, ...
                    'FontWeight', 'bold', ...
                    'Color', 'k'); % Černá barva
            end
   
       
        end
        exportgraphics(f,pdf_name,'Append', true, 'ContentType','vector');

        %Výpočet dalších řádků
        count = count + 1;
        if length(idx_all) >= count
            i = idx_all(count);
        else
            break;
        end
        

    end
end
