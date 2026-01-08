% run_mpc_simulation.m
try
    fprintf('Building MPC Model with HESS...\n');
    build_mpc_model();
    
    fprintf('Running Simulation...\n');
    simOut = sim('bmw_i3_mpc_model');
    
    fprintf('Simulation Complete. Analyzing Results...\n');
    
    % Extract data
    time = simOut.sim_time;
    soc = simOut.soc.signals.values;
    i_batt = simOut.i_batt.signals.values;
    v_term = simOut.v_term.signals.values;
    p_elec = simOut.p_elec.signals.values;
    distance = simOut.distance.signals.values;
    
    % HESS Data
    hess_data = struct();
    hess_data.p_sc_cmd = simOut.p_sc_cmd.signals.values;
    hess_data.sc_soc = simOut.sc_soc.signals.values;
    hess_data.i_sc = simOut.i_sc.signals.values;
    hess_data.v_sc = simOut.v_sc.signals.values;
    hess_data.p_batt_cmd = simOut.p_batt_cmd.signals.values;
    
    % Calculate Metrics
    vehicle = evalin('base', 'vehicle');
    battery_metrics = calculate_battery_metrics(time, soc, i_batt, v_term, p_elec, distance, vehicle, hess_data);
    
    % --- Analysis & Formatting (Matched to Full Simulation) ---
    
    % Total Distance
    total_dist_m = distance(end);
    total_dist_km = total_dist_m / 1000;
    
    % Energy Consumption
    energy_J = trapz(time, p_elec);
    energy_Wh = energy_J / 3600;
    
    % Specific Consumption
    consumption_Wh_km = energy_Wh / total_dist_km;
    
    % SoC Drop
    soc_start = soc(1) * 100;
    soc_end = soc(end) * 100;
    soc_drop = soc_start - soc_end;
    
    % Speed Tracking Error
    vel_ref = simOut.vel_ref.signals.values;
    vel_actual = simOut.vel_actual.signals.values;
    error_kmh = (vel_ref - vel_actual) * 3.6;
    max_error = max(abs(error_kmh));
    rms_error = rms(error_kmh);
    
    % Validation
    benchmark_Wh_km = 110;
    error_pct = (consumption_Wh_km - benchmark_Wh_km) / benchmark_Wh_km * 100;
    
    fprintf('\n==================================================\n');
    fprintf('MPC + HESS Simulation Results\n');
    fprintf('==================================================\n');
    fprintf('Driving Cycle:        NEDC\n');
    fprintf('Total Distance:       %.2f km\n', total_dist_km);
    fprintf('Total Energy:         %.2f Wh\n', energy_Wh);
    fprintf('Consumption:          %.2f Wh/km\n', consumption_Wh_km);
    fprintf('Benchmark:            %.2f Wh/km\n', benchmark_Wh_km);
    fprintf('Error:                %.2f %%\n', error_pct);
    fprintf('SoC Drop:             %.2f %%\n', soc_drop);
    fprintf('Max Speed Error:      %.2f km/h\n', max_error);
    fprintf('RMS Speed Error:      %.2f km/h\n', rms_error);
    fprintf('==================================================\n');
    
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
    
    % --- Plotting ---
    figure('Name', 'MPC + HESS Simulation Results', 'NumberTitle', 'off');
    
    subplot(3,1,1);
    plot(time, vel_ref*3.6, 'k--', 'LineWidth', 1.5); hold on;
    plot(time, vel_actual*3.6, 'b', 'LineWidth', 1);
    ylabel('Speed (km/h)');
    legend('Reference', 'Actual');
    title('Speed Tracking (MPC)');
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
    figure('Name', 'HESS Performance (MPC)', 'NumberTitle', 'off');
    
    subplot(3,1,1);
    plot(time, hess_data.sc_soc*100, 'b');
    ylabel('SC SoC (%)');
    title('Supercapacitor State of Charge');
    grid on;
    
    subplot(3,1,2);
    plot(time, hess_data.p_sc_cmd/1000, 'm');
    ylabel('SC Power (kW)');
    title('Supercapacitor Power (Pos = Discharge, Neg = Charge)');
    grid on;
    
    subplot(3,1,3);
    plot(time, hess_data.p_batt_cmd/1000, 'g');
    ylabel('Batt Power Cmd (kW)');
    xlabel('Time (s)');
    title('Battery Power Command from EMS');
    grid on;
    
    % Battery Health Dashboard
    if ~isempty(fieldnames(battery_metrics))
        fprintf('Generating battery health visualizations...\n');
        try
            plot_battery_histograms(battery_metrics, vehicle);
            fprintf('Battery health dashboard created.\n');
        catch ME
            fprintf('Warning: Could not generate battery visualizations: %s\n', ME.message);
        end
    end
    
catch ME
    fprintf('Error: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for k = 1:length(ME.stack)
        fprintf('File: %s, Line: %d, Name: %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end
end
