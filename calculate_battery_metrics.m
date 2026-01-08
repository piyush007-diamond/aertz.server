function metrics = calculate_battery_metrics(time, soc, i_batt, v_term, p_elec, distance, vehicle, hess_data)
    % calculate_battery_metrics - Comprehensive battery health and efficiency analysis
    %
    % Inputs:
    %   time      - Time vector (seconds)
    %   soc       - State of Charge vector (0-1)
    %   i_batt    - Battery current vector (A)
    %   v_term    - Terminal voltage vector (V)
    %   p_elec    - Electrical power vector (W)
    %   distance  - Distance traveled vector (m)
    %   vehicle   - Vehicle parameters struct
    %
    % Outputs:
    %   metrics   - Struct containing:
    %       .dod_data          - Depth of Discharge data for histogram
    %       .dod_histogram     - Histogram object data
    %       .cycle_count       - Equivalent Full Cycles (EFC)
    %       .wh_per_km         - Energy consumption per km
    %       .lifetime_kwh_throughput - Total kWh over vehicle life
    %       .battery_replacements    - Number of replacements needed
    %       .battery_lifetime_km     - Battery lifetime in km
    %       .battery_lifetime_years  - Battery lifetime in years
    %       .soh_current            - Current SOH (%)
    %       .soh_degradation_rate   - SOH degradation per 1000 km (%)
    
    %% 1. Basic Energy Metrics
    
    % Total energy consumed in this cycle
    energy_J = trapz(time, p_elec);
    energy_Wh = energy_J / 3600;
    energy_kWh = energy_Wh / 1000;
    
    % Distance traveled
    total_dist_m = distance(end);
    total_dist_km = total_dist_m / 1000;
    
    % Energy per km
    metrics.wh_per_km = energy_Wh / total_dist_km;
    
    %% 2. Depth of Discharge (DoD) Analysis
    
    % Convert SOC to percentage
    soc_pct = soc * 100;
    
    % Find local minima and maxima in SOC (turning points)
    % Use simple peak detection
    window_size = 10; % Smooth to avoid noise
    soc_smooth = movmean(soc_pct, window_size);
    
    % Find turning points
    maxima_idx = [];
    minima_idx = [];
    
    for i = 2:(length(soc_smooth)-1)
        if soc_smooth(i) > soc_smooth(i-1) && soc_smooth(i) > soc_smooth(i+1)
            maxima_idx = [maxima_idx, i];
        elseif soc_smooth(i) < soc_smooth(i-1) && soc_smooth(i) < soc_smooth(i+1)
            minima_idx = [minima_idx, i];
        end
    end
    
    % Calculate DoD for each discharge cycle
    % Pair each maximum with the next minimum
    metrics.dod_data = [];
    
    for i = 1:min(length(maxima_idx), length(minima_idx))
        if maxima_idx(i) < minima_idx(i)
            dod = soc_pct(maxima_idx(i)) - soc_pct(minima_idx(i));
            if dod > 0.5  % Filter out noise (DoD > 0.5%)
                metrics.dod_data = [metrics.dod_data, dod];
            end
        end
    end
    
    % If no clear cycles detected, use overall SOC swing
    if isempty(metrics.dod_data)
        overall_dod = max(soc_pct) - min(soc_pct);
        metrics.dod_data = [overall_dod];
    end
    
    % Create histogram bins (0-10%, 10-20%, ..., 90-100%)
    bin_edges = 0:10:100;
    [counts, edges] = histcounts(metrics.dod_data, bin_edges);
    metrics.dod_histogram.counts = counts;
    metrics.dod_histogram.edges = edges;
    metrics.dod_histogram.bin_centers = (edges(1:end-1) + edges(2:end)) / 2;
    
    %% 3. Cycle Counting (Equivalent Full Cycles - EFC)
    
    % Simplified cycle counting with DoD weighting
    % Each cycle is weighted by its DoD as a fraction of 100%
    % Example: 2 cycles at 50% DoD = 1 EFC
    
    cycle_weights = metrics.dod_data / 100;  % Normalize to 0-1
    metrics.cycle_count = sum(cycle_weights);
    
    % If cycle count is very small (< 0.001), use SOC drop as estimate
    if metrics.cycle_count < 0.001
        soc_drop = soc(1) - soc(end);
        metrics.cycle_count = abs(soc_drop);
    end
    
    %% 4. Battery Degradation and SOH Modeling
    
    % Get degradation parameters (with defaults if not available)
    if isfield(vehicle, 'battery') && isfield(vehicle.battery, 'soh_initial')
        soh_initial = vehicle.battery.soh_initial;
        soh_eol = vehicle.battery.soh_eol;
        k_cycle = vehicle.battery.cycle_aging_coeff;
        k_cal = vehicle.battery.calendar_aging_coeff;
    else
        % Default values
        soh_initial = 100;
        soh_eol = 80;
        k_cycle = 0.025;
        k_cal = 0.5;
    end
    
    % Average DoD for this cycle
    avg_dod = mean(metrics.dod_data) / 100;  % Normalize to 0-1
    
    % DoD stress factor (higher DoD = more degradation)
    % Using empirical relationship: stress increases with DoD
    dod_stress_factor = 1 + 0.5 * (avg_dod - 0.5);
    if dod_stress_factor < 0.5
        dod_stress_factor = 0.5;  % Minimum factor
    end
    
    % For current simulation (single cycle), SOH loss is minimal
    % Calculate per-cycle degradation
    % C-rate stress factor
    % High C-rates accelerate aging. HESS reduces peak C-rates.
    % Calculate peak C-rate
    if isfield(vehicle.battery, 'capacity_Ah')
        capacity_Ah = vehicle.battery.capacity_Ah * vehicle.battery.n_parallel;
    else
        capacity_Ah = 94.5; % Default
    end
    
    % ACTUAL peak battery current (WITH HESS)
    peak_current = max(abs(i_batt));
    peak_c_rate = peak_current / capacity_Ah;
    
    % Calculate THEORETICAL peak current WITHOUT HESS
    % This would be if battery handled all power directly
    avg_voltage = mean(v_term);
    if avg_voltage > 0
        theoretical_peak_current_no_hess = max(abs(p_elec)) / avg_voltage;
        peak_c_rate_no_hess = theoretical_peak_current_no_hess / capacity_Ah;
    else
        peak_c_rate_no_hess = peak_c_rate; % Fallback
    end
    
    % Get C-rate stress threshold from vehicle params (default 0.3C)
    if isfield(vehicle.battery, 'c_rate_stress_threshold')
        c_rate_threshold = vehicle.battery.c_rate_stress_threshold;
    else
        c_rate_threshold = 0.3; % Default: stress starts above 0.3C
    end
    
    % Stress factor WITH HESS: Based on actual observed C-rate
    c_rate_stress_factor = 1.0;
    if peak_c_rate > c_rate_threshold
        c_rate_stress_factor = 1.0 + (peak_c_rate - c_rate_threshold);
    end
    
    % Stress factor WITHOUT HESS: Based on theoretical C-rate if no SC
    c_rate_stress_factor_no_hess = 1.0;
    if peak_c_rate_no_hess > c_rate_threshold
        c_rate_stress_factor_no_hess = 1.0 + (peak_c_rate_no_hess - c_rate_threshold);
    end
    
    % Store both for later use
    metrics.peak_c_rate = peak_c_rate;
    metrics.peak_c_rate_no_hess = peak_c_rate_no_hess;
    metrics.c_rate_stress_factor = c_rate_stress_factor;
    metrics.c_rate_stress_factor_no_hess = c_rate_stress_factor_no_hess;
    
    % For current simulation (single cycle), SOH loss is minimal
    % Calculate per-cycle degradation
    soh_loss_per_cycle = k_cycle * 0.01 * dod_stress_factor * c_rate_stress_factor;  % % per cycle
    
    metrics.soh_current = soh_initial - (soh_loss_per_cycle * metrics.cycle_count);
    
    % Debug prints for stress factors
    fprintf('  > Peak C-rate WITH HESS: %.2f C (Stress Factor: %.2f)\n', peak_c_rate, c_rate_stress_factor);
    fprintf('  > Peak C-rate WITHOUT HESS: %.2f C (Stress Factor: %.2f)\n', peak_c_rate_no_hess, c_rate_stress_factor_no_hess);
    fprintf('  > Avg DoD: %.2f%% (Stress Factor: %.2f)\n', avg_dod*100, dod_stress_factor);
    fprintf('  > Cycle EFC: %.4f\n', metrics.cycle_count);
    
    %% 5. Lifetime Predictions
    
    % Get vehicle lifetime parameters
    if isfield(vehicle, 'lifetime')
        total_lifetime_km = vehicle.lifetime.total_km;
        lifetime_years = vehicle.lifetime.years;
    else
        % Default values
        total_lifetime_km = 200000;  % 200,000 km
        lifetime_years = 10;         % 10 years
    end
    
    % Calculate how many times this cycle would run over vehicle life
    cycles_per_lifetime = total_lifetime_km / total_dist_km;
    
    % Total EFC over vehicle life
    total_efc_lifetime = metrics.cycle_count * cycles_per_lifetime;
    
    % Total kWh throughput over lifetime
    metrics.lifetime_kwh_throughput = energy_kWh * cycles_per_lifetime;
    
    % Predict SOH degradation over lifetime
    % Cycle aging component
    soh_loss_cycle_lifetime = k_cycle * sqrt(total_efc_lifetime / 100) * dod_stress_factor * c_rate_stress_factor;
    
    % Calendar aging component (degradation even when not cycling)
    soh_loss_calendar_lifetime = k_cal * sqrt(lifetime_years);
    
    % Total SOH at end of life
    soh_at_eol = soh_initial - soh_loss_cycle_lifetime - soh_loss_calendar_lifetime;
    
    % Calculate battery lifetime (when SOH reaches 80%)
    % Using cycle aging + calendar aging model
    % We need to solve: soh_initial - k_cycle*sqrt(EFC) - k_cal*sqrt(years) = soh_eol
    
    % Iterative approach to find when battery reaches EOL
    % Need enough iterations for >200,000 km (approx 20,000 cycles)
    % Iterative approach to find when battery reaches EOL
    % Need enough iterations for >200,000 km (approx 20,000 cycles)
    % Increased to 500,000 to ensure we find the true EOL even for very long-lasting batteries
    max_iterations = 500000;
    soh_track = soh_initial;
    km_traveled = 0;
    years_elapsed = 0;
    efc_accumulated = 0;
    
    km_per_year = total_lifetime_km / lifetime_years;
    
    for iter = 1:max_iterations
        if soh_track <= soh_eol
            break;
        end
        
        % Increment distance
        km_traveled = km_traveled + total_dist_km;
        efc_accumulated = efc_accumulated + metrics.cycle_count;
        years_elapsed = km_traveled / km_per_year;
        
        % Calculate degradation
        % Note: k_cycle represents % SOH loss per sqrt(100 EFC)
        % So we scale EFC by dividing by 100 before taking sqrt
        soh_loss_cycle = k_cycle * sqrt(efc_accumulated / 100) * dod_stress_factor * c_rate_stress_factor;
        soh_loss_cal = k_cal * sqrt(years_elapsed);
        
        soh_track = soh_initial - soh_loss_cycle - soh_loss_cal;
    end
    
    metrics.battery_lifetime_km = km_traveled;
    metrics.battery_lifetime_years = years_elapsed;
    
    % Validation: Print final SOH to verify calculation
    fprintf('  > Battery EOL at: %.0f km / %.1f years\n', km_traveled, years_elapsed);
    fprintf('  > Final SOH: %.2f%% (Cycle loss: %.2f%%, Calendar loss: %.2f%%)\n', ...
        soh_track, soh_loss_cycle, soh_loss_cal);
    
    % Calculate BASELINE lifetime (WITHOUT HESS) for comparison
    soh_track_baseline = soh_initial;
    km_traveled_baseline = 0;
    efc_accumulated_baseline = 0;
    
    for iter = 1:max_iterations
        if soh_track_baseline <= soh_eol
            break;
        end
        
        km_traveled_baseline = km_traveled_baseline + total_dist_km;
        efc_accumulated_baseline = efc_accumulated_baseline + metrics.cycle_count;
        years_elapsed_baseline = km_traveled_baseline / km_per_year;
        
        % Use NO-HESS stress factor for baseline
        soh_loss_cycle_baseline = k_cycle * sqrt(efc_accumulated_baseline / 100) * dod_stress_factor * c_rate_stress_factor_no_hess;
        soh_loss_cal_baseline = k_cal * sqrt(years_elapsed_baseline);
        
        soh_track_baseline = soh_initial - soh_loss_cycle_baseline - soh_loss_cal_baseline;
    end
    
    metrics.battery_lifetime_km_baseline = km_traveled_baseline;
    metrics.battery_lifetime_years_baseline = km_traveled_baseline / km_per_year;
    
    % Calculate lifetime extension from HESS
    if km_traveled_baseline > 0
        metrics.lifetime_extension_pct = ((km_traveled - km_traveled_baseline) / km_traveled_baseline) * 100;
        metrics.lifetime_extension_km = km_traveled - km_traveled_baseline;
    else
        metrics.lifetime_extension_pct = 0;
        metrics.lifetime_extension_km = 0;
    end
    
    fprintf('  > Battery EOL WITHOUT HESS (Baseline): %.0f km / %.1f years\n', km_traveled_baseline, km_traveled_baseline / km_per_year);
    fprintf('  > HESS Lifetime Extension: +%.0f km (+%.1f%%)\n', metrics.lifetime_extension_km, metrics.lifetime_extension_pct);
    
    % Calculate degradation rate per 1000 km
    if metrics.battery_lifetime_km > 0
        total_degradation = soh_initial - soh_eol;
        metrics.soh_degradation_rate = (total_degradation / metrics.battery_lifetime_km) * 1000;
    else
        metrics.soh_degradation_rate = 0;
    end
    
    % Calculate number of battery replacements needed over vehicle life
    if metrics.battery_lifetime_km > 0
        metrics.battery_replacements = floor(total_lifetime_km / metrics.battery_lifetime_km);
    else
        metrics.battery_replacements = 0;
    end
    
    %% 6. Additional Metrics for Visualization
    
    % SOH projection - extend to calculated battery lifetime (plus buffer)
    % This shows when the battery reaches EOL, not just the vehicle target
    num_points = 100;
    max_km_projection = max(total_lifetime_km, metrics.battery_lifetime_km * 1.1);
    km_projection = linspace(0, max_km_projection, num_points);
    soh_projection = zeros(size(km_projection));
    
    for i = 1:num_points
        km = km_projection(i);
        years = km / km_per_year;
        efc = (km / total_dist_km) * metrics.cycle_count;
        
        soh_loss_cycle = k_cycle * sqrt(efc / 100) * dod_stress_factor * c_rate_stress_factor;
        soh_loss_cal = k_cal * sqrt(years);
        
        soh_projection(i) = soh_initial - soh_loss_cycle - soh_loss_cal;
        
        % Floor at EOL
        if soh_projection(i) < soh_eol
            soh_projection(i) = soh_eol;
        end
    end
    
    metrics.soh_projection.km = km_projection;
    metrics.soh_projection.soh = soh_projection;
    metrics.soh_projection.years = km_projection / km_per_year;
    
    % Cycle count breakdown by DoD range
    metrics.cycle_breakdown.dod_ranges = {'0-20%', '20-40%', '40-60%', '60-80%', '80-100%'};
    metrics.cycle_breakdown.counts = zeros(1, 5);
    
    for i = 1:length(metrics.dod_data)
        dod = metrics.dod_data(i);
        if dod <= 20
            metrics.cycle_breakdown.counts(1) = metrics.cycle_breakdown.counts(1) + (dod/100);
        elseif dod <= 40
            metrics.cycle_breakdown.counts(2) = metrics.cycle_breakdown.counts(2) + (dod/100);
        elseif dod <= 60
            metrics.cycle_breakdown.counts(3) = metrics.cycle_breakdown.counts(3) + (dod/100);
        elseif dod <= 80
            metrics.cycle_breakdown.counts(4) = metrics.cycle_breakdown.counts(4) + (dod/100);
        else
            metrics.cycle_breakdown.counts(5) = metrics.cycle_breakdown.counts(5) + (dod/100);
        end
    end
    
    %% 7. HESS Metrics (If available)
    if nargin > 7 && isstruct(hess_data) && isfield(hess_data, 'p_sc_cmd')
        metrics.hess.available = true;
        
        % Extract HESS data
        p_sc_cmd = hess_data.p_sc_cmd;
        sc_soc = hess_data.sc_soc;
        
        % 1. Peak Power Analysis
        % Max power absorbed by SC (Regen)
        metrics.hess.peak_regen_power_sc = min(p_sc_cmd); % Negative value
        
        % Max power delivered by SC (Assist)
        metrics.hess.peak_assist_power_sc = max(p_sc_cmd);
        
        % 2. Energy Throughput
        % Energy handled by SC
        energy_sc_J = trapz(time, abs(p_sc_cmd));
        metrics.hess.energy_throughput_kWh = energy_sc_J / 3600 / 1000;
        
        % 3. Battery Stress Reduction
        % Literature-standard definition: (Peak WITHOUT HESS - Peak WITH HESS) / Peak WITHOUT HESS
        % This measures the reduction in battery peak load due to SC, should match C-rate reduction
        
        % Peak power that battery WOULD handle without SC (= total load demand)
        peak_power_no_hess = max(abs(p_elec));
        
        % Peak power battery ACTUALLY handles with SC (= total load - SC contribution)
        % SC power is positive when discharging (assisting), negative when charging
        if ~isempty(p_sc_cmd)
            p_battery_with_hess = p_elec - p_sc_cmd;  % Battery handles remainder
            peak_power_with_hess = max(abs(p_battery_with_hess));
        else
            peak_power_with_hess = peak_power_no_hess;
        end
        
        % Peak shaving = reduction in battery peak power
        if peak_power_no_hess > 0
            metrics.hess.peak_shaving_pct = ((peak_power_no_hess - peak_power_with_hess) / peak_power_no_hess) * 100;
        else
            metrics.hess.peak_shaving_pct = 0;
        end
        
        % 4. SC Utilization
        metrics.hess.avg_sc_soc = mean(sc_soc);
        metrics.hess.min_sc_soc = min(sc_soc);
        metrics.hess.max_sc_soc = max(sc_soc);
        
        fprintf('HESS metrics calculated: Peak Shaving = %.1f%%\n', metrics.hess.peak_shaving_pct);
    else
        metrics.hess.available = false;
    end

    fprintf('Battery metrics calculated successfully.\n');
end
