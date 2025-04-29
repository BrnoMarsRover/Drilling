%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FILE:        drill_ReportGen.m
% AUTHOR:      Martin Kriz
% DATE:        2025-04-29
% VERSION:     1.2
%
% DESCRIPTION:
%   This script processes drill data logged from node drill_controller and 
%   generates visualizations including torque, rotational speed (RPS), 
%   temperature, and vertical position. Outputs are saved as vector-based 
%   PDF reports.
%   This script reads all files matching 'drillData*.txt' in the current
%   folder and generates one separate .pdf report for each file.
%   Each PDF will contain one or more pages, depending on how many
%   'DrillSample' actions are recorded in the file.
%
% REQUIREMENTS:
%   - Input files named 'drillData*.txt' in working directory
%   - 'logo.bmp' in working directory
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

%Loop time in drill_controller
loop_time = 1;

% Files with the right name
files = dir('drillData*.txt');

% For every file in the folder
for k = 1:length(files)
    filename = files(k).name;
    pdf_name = strrep(filename, '.txt', '.pdf');
    fid = fopen(filename, 'r');
    data_raw = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    data_raw = data_raw{1};
    
    % Find Action DrillSamples 
    idx_all = find(contains(data_raw, 'Action DrillSample'));
    count = 1;
    if length(idx_all) >= 1
        i = idx_all(1);
    else
        break;
    end
    [rows, ~] = size(data_raw);

    % For every drill sample
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
            
            % TEXT TO DELETE IN NEW VERSION
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %temperature simulation, this is temporrary
            n = length(temperature); % počet vzorků
            start_temp = 23;
            end_temp = 35;
            % Simulovaný nárůst teploty s mírným šumem a nelineárním charakterem
            temperature = linspace(start_temp, end_temp, n);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Charts
            f = figure;
            f.Units = "centimeters";
            f.Position = [0, 0, 21, 29.7];
            tiledlayout(3,2, 'TileSpacing', 'loose', 'Padding', 'compact');

            % --- Mars Rover logo ---
            nexttile; axis off;
            nexttile;
            logo = imread('logo.bmp');
            imshow(logo);


            % --- CHART 1 ---
            nexttile;
            plot(t, torque, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Torque [Nm]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Drilling torque', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            style_axes(gca);
            
            % --- CHART 2 ---
            nexttile;
            plot(t, rps, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('RPS [R/s]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Drill bit rotation speed', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            style_axes(gca);

            % --- CHART 3 ---
            nexttile;
            plot(t, temperature, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Temperature [°C]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Motor temperature', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            style_axes(gca);

            % --- CHART 4 ---
            nexttile;
            plot(t, height, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
            grid on;
            
            xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Height [mm]', 'FontSize', 12, 'FontWeight', 'bold');
            title('Vertical position of the drill', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0.2 0.1]);
            style_axes(gca);

            % --- Annotation ---
            add_annotations(data_raw, i);

   
       
        end
        % Add new page to .pdf file
        exportgraphics(f,pdf_name,'Append', true, 'ContentType','vector');

        %Calculates next drill action (if any)
        count = count + 1;
        if length(idx_all) >= count
            i = idx_all(count);
        else
            break;
        end
        

    end
end


% Settings for Mars style
function style_axes(ax)
    set(ax, ...
        'FontSize', 11, ...
        'GridColor', [0.5 0.5 0.5], ...
        'GridAlpha', 0.3, ...
        'LineWidth', 1.2, ...
        'Box', 'on', ...
        'XColor', [0.4 0.1 0.1], ...
        'YColor', [0.4 0.1 0.1], ...
        'Color', [1 0.96 0.94]);
end

% Text annotations for wight of the sample, slot and depth
function add_annotations(data_raw, i)
    % Annotation 1
    annotation('textbox', [0.05, 0.8, 0.8, 0.05], 'String', data_raw{i}, ...
        'EdgeColor', 'none', ...
        'HorizontalAlignment', 'left', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Color', 'k');

    % Annotation 2
    annotation('textbox', [0.05, 0.77, 0.8, 0.05], 'String', data_raw{i+2}, ...
        'EdgeColor', 'none', ...
        'HorizontalAlignment', 'left', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Color', 'k');

    % Annotation 3 only if store sample is after drill sample action
    if (contains(data_raw{i+4}, 'StoreSample'))
        annotation('textbox', [0.05, 0.74, 0.8, 0.05], 'String', data_raw{i+5}, ...
            'EdgeColor', 'none', ...
            'HorizontalAlignment', 'left', ...
            'FontSize', 12, ...
            'FontWeight', 'bold', ...
            'Color', 'k');
    end
end
