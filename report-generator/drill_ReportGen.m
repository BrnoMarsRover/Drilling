clc; clear; close all;

%Čas vykonávání smyčky v node drill_controller
loop_time = 0.1;

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
            height = values(:, 4);

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

            %1
            nexttile;
            plot(t, torque); 
            grid on
            xlabel('t [s]', 'FontSize', 10);
            ylabel('Torque [Nm]', 'FontSize', 10);
            title('Measured motor torque', 'FontSize', 10);

            %2
            nexttile;
            plot(t, rps); 
            grid on
            xlabel('t [s]', 'FontSize', 10);
            ylabel('RPS [R/s]', 'FontSize', 10);
            title('Measured rotates per second', 'FontSize', 10);

            %3
            nexttile;
            plot(t, temperature); 
            grid on
            xlabel('t [s]', 'FontSize', 10);
            ylabel('Temperature [mm]', 'FontSize', 10);
            title('Measured motor temperature', 'FontSize', 10);

            %4
            nexttile;
            plot(t, height); 
            grid on
            xlabel('t [s]', 'FontSize', 10);
            ylabel('Height [mm]', 'FontSize', 10);
            title('Measured height from encoders', 'FontSize', 10);

            annotation('textbox', [0.05, 0.8, 0.8, 0.05], 'String', data_raw(i), ...
           'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'FontSize', 12);

            if (contains(data_raw{i+4}, 'StoreSample'))
                annotation('textbox', [0.05, 0.77, 0.8, 0.05], 'String', strcat(data_raw(i+2), ", ", data_raw(i+5)), ...
                'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'FontSize', 12);


            else
                annotation('textbox', [0.05, 0.77, 0.8, 0.05], 'String', data_raw(i+2), ...
                'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'FontSize', 12);

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
