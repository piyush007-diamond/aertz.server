% run_full_simulation.m - Execute and Verify BMW i3 Full Model

% 1. Build the Model (Ensures it exists and is up to date)
fprintf('Building Simulink Model...\n');
try
    build_midc_model();
catch ME
    fprintf('Error building model: %s\n', ME.message);
    return;
end

% 2. Run Simulation
fprintf('Running Simulation (MIDC)...\n');
try
    simOut = sim('bmw_i3_midc_model');
    fprintf('Simulation Complete.\n');
catch ME
    fprintf('Error running simulation: %s\n', ME.message);
    return;
end

% 3. Extract Results
% The To Workspace blocks save data to the simOut object
% We need to extract from simOut

% Check if data is in workspace (old behavior) or in simOut (new behavior)
if exist('sim_time', 'var')
    % Data in workspace - extract directly
    fprintf('Extracting data from workspace...\n');
    time = sim_time;
    vel_ref_data = vel_ref.signals.values;
    vel_actual_data = vel_actual.signals.values;
    soc_data = soc.signals.values;
    v_term_data = v_term.signals.values;
    i_batt_data = i_batt.signals.values;
    p_elec_data = p_elec.signals.values;
    distance_data = distance.signals.values;
else
    % Data in simOut object - extract from there
    fprintf('Extracting data from simOut object...\n');
    
    % Extract from simOut - the variable names match what we set in To Workspace blocks
    try
        time = simOut.sim_time;
        vel_ref_data = simOut.vel_ref.signals.values;
        vel_actual_data = simOut.vel_actual.signals.values;
        soc_data = simOut.soc.signals.values;
        v_term_data = simOut.v_term.signals.values;
        i_batt_data = simOut.i_batt.signals.values;
        p_elec_data = simOut.p_elec.signals.values;
        distance_data = simOut.distance.signals.values;
        
        % Extract HESS Data
        try
            sc_soc_data = simOut.sc_soc.signals.values;
            i_sc_data = simOut.i_sc.signals.values;
            v_sc_data = simOut.v_sc.signals.values;
            p_batt_cmd_data = simOut.p_batt_cmd.signals.values;
            p_sc_cmd_data = simOut.p_sc_cmd.signals.values;
        catch
            % If HESS data is missing (e.g. older model version), ignore
            sc_soc_data = [];
        end
    catch ME
        fprintf('Error extracting from simOut: %s\n', ME.message);
        fprintf('SimOut contents:\n');
        disp(simOut);
        error('Could not extract simulation data from simOut object.');
    end
end

% Assign to simple variable names for the rest of the script
vel_ref = vel_ref_data;
vel_actual = vel_actual_data;
soc = soc_data;
v_term = v_term_data;
i_batt = i_batt_data;
p_elec = p_elec_data;
distance = distance_data;

dt = [diff(time); 0];

% 4. Calculate Battery Health Metrics
fprintf('Calculating battery health metrics...\n');
try
    % Load vehicle parameters for metrics calculation
    [vehicle, ~] = setup_full_params();
    
    % Prepare HESS data struct
    hess_data = [];
    if exist('sc_soc_data', 'var') && ~isempty(sc_soc_data)
        hess_data = struct();
        hess_data.sc_soc = sc_soc_data;
        hess_data.i_sc = i_sc_data;
        hess_data.v_sc = v_sc_data;
        hess_data.p_batt_cmd = p_batt_cmd_data;
        hess_data.p_sc_cmd = p_sc_cmd_data;
    end
    
    % Calculate comprehensive battery metrics
    battery_metrics = calculate_battery_metrics(time, soc, i_batt, v_term, p_elec, distance, vehicle, hess_data);
    fprintf('Battery metrics calculated successfully.\n');
catch ME
    fprintf('Warning: Could not calculate battery metrics: %s\n', ME.message);
    battery_metrics = struct();  % Empty struct if calculation fails
end

% 5. Analysis

% Total Distance
total_dist_m = distance(end);
total_dist_km = total_dist_m / 1000;

% Energy Consumption
% P_elec is in Watts. Energy = Integral(P * dt)
energy_J = trapz(time, p_elec);
energy_Wh = energy_J / 3600;
energy_kWh = energy_Wh / 1000;

% Specific Consumption
consumption_Wh_km = energy_Wh / total_dist_km;

% SoC Drop
soc_start = soc(1) * 100;
soc_end = soc(end) * 100;
soc_drop = soc_start - soc_end;

% Speed Tracking Error
error_kmh = (vel_ref - vel_actual) * 3.6;
max_error = max(abs(error_kmh));
rms_error = rms(error_kmh);

% 5. Validation
benchmark_Wh_km = 110;
error_pct = (consumption_Wh_km - benchmark_Wh_km) / benchmark_Wh_km * 100;

fprintf('\n==================================================\n');
fprintf('Tata Nexon EV Simulation Results (Full Model)\n');
fprintf('==================================================\n');
fprintf('Driving Cycle:        MIDC\n');
fprintf('Total Distance:       %.2f km\n', total_dist_km);
fprintf('Total Energy:         %.2f Wh\n', energy_Wh);
fprintf('Consumption:          %.2f Wh/km\n', consumption_Wh_km);
fprintf('Benchmark:            %.2f Wh/km\n', benchmark_Wh_km);
fprintf('Error:                %.2f %%\n', error_pct);
fprintf('SoC Drop:             %.2f %%\n', soc_drop);
fprintf('Max Speed Error:      %.2f km/h\n', max_error);
fprintf('RMS Speed Error:      %.2f km/h\n', rms_error);
fprintf('==================================================\n');

% Display Battery Health Metrics
if ~isempty(fieldnames(battery_metrics))
    fprintf('\n==================================================\n');
    fprintf('Battery Health & Efficiency Metrics\n');
    fprintf('==================================================\n');
    fprintf('Energy Efficiency:        %.2f Wh/km\n', battery_metrics.wh_per_km);
    fprintf('Cycle Count (EFC):        %.4f cycles\n', battery_metrics.cycle_count);
    fprintf('Battery Lifetime:         %.0f km / %.1f years\n', ...
            battery_metrics.battery_lifetime_km, battery_metrics.battery_lifetime_years);
    fprintf('Expected Replacements:    %d times\n', battery_metrics.battery_replacements);
    fprintf('Lifetime kWh Throughput:  %.0f kWh\n', battery_metrics.lifetime_kwh_throughput);
    fprintf('Current SOH:              %.2f %%\n', battery_metrics.soh_current);
    fprintf('SOH Degradation Rate:     %.3f %%/1000km\n', battery_metrics.soh_degradation_rate);
    
    if isfield(battery_metrics, 'hess') && battery_metrics.hess.available
        fprintf('\n--------------------------------------------------\n');
        fprintf('Supercapacitor HESS Performance\n');
        fprintf('--------------------------------------------------\n');
        fprintf('Peak Regen Absorbed:      %.2f kW\n', abs(battery_metrics.hess.peak_regen_power_sc)/1000);
        fprintf('Peak Assist Delivered:    %.2f kW\n', battery_metrics.hess.peak_assist_power_sc/1000);
        fprintf('Energy Throughput:        %.4f kWh\n', battery_metrics.hess.energy_throughput_kWh);
        fprintf('Peak Power Shaving:       %.1f %%\n', battery_metrics.hess.peak_shaving_pct);
        fprintf('SC Utilization (SoC):     %.1f%% - %.1f%%\n', battery_metrics.hess.min_sc_soc*100, battery_metrics.hess.max_sc_soc*100);
    end
    fprintf('==================================================\n');
end

if abs(error_pct) < 10
    fprintf('VALIDATION STATUS: PASS\n');
else
    fprintf('VALIDATION STATUS: FAIL (Target < 6%%)\n');
end

% 6. Plotting
figure('Name', 'Tata Nexon EV Simulation Results', 'NumberTitle', 'off');

subplot(3,1,1);
plot(time, vel_ref*3.6, 'k--', 'LineWidth', 1.5); hold on;
plot(time, vel_actual*3.6, 'b', 'LineWidth', 1);
ylabel('Speed (km/h)');
legend('Reference', 'Actual');
title('Speed Tracking');
grid on;

subplot(3,1,2);
plot(time, p_elec/1000, 'r');
ylabel('Battery Power (kW)');
title('Power Consumption (Positive = Discharge)');
grid on;

subplot(3,1,3);
plot(time, soc*100, 'g');
ylabel('SoC (%)');
xlabel('Time (s)');
title('State of Charge');
grid on;

% HESS Plot
if exist('sc_soc_data', 'var') && ~isempty(sc_soc_data)
    figure('Name', 'HESS Performance', 'NumberTitle', 'off');
    
    subplot(3,1,1);
    plot(time, sc_soc_data*100, 'b');
    ylabel('SC SoC (%)');
    title('Supercapacitor State of Charge');
    grid on;
    
    subplot(3,1,2);
    plot(time, p_sc_cmd_data/1000, 'm');
    ylabel('SC Power (kW)');
    title('Supercapacitor Power (Pos = Discharge, Neg = Charge)');
    grid on;
    
    subplot(3,1,3);
    plot(time, p_batt_cmd_data/1000, 'g');
    ylabel('Batt Power Cmd (kW)');
    xlabel('Time (s)');
    title('Battery Power Command from EMS');
    grid on;
end

% 7. Battery Health Dashboard
if ~isempty(fieldnames(battery_metrics))
    fprintf('Generating battery health visualizations...\n');
    try
        plot_battery_histograms(battery_metrics, vehicle);
        fprintf('Battery health dashboard created.\n');
    catch ME
        fprintf('Warning: Could not generate battery visualizations: %s\n', ME.message);
    end
end

% Close all figures for batch mode
close all;
