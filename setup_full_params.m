function [vehicle, drive_cycle] = setup_full_params()
    % setup_full_params - Define ALL BMW i3 parameters for full simulation
    % Returns:
    %   vehicle: Struct containing all physical and electrical parameters
    %   drive_cycle: Struct containing time and velocity vectors
    
    %% 1. Physical Vehicle Parameters (Tata Nexon EV - Final Tune)
    vehicle.M_vehicle = 1500;      % Curb weight (kg)
    vehicle.Cd = 0.33;            % Drag coefficient (Optimized)
    vehicle.A_frontal = 2.54;      % Frontal area (m^2)
    vehicle.C_RR = 0.0095;          % Rolling resistance
    vehicle.r_tire = 0.344;        % Tire radius (m)
    vehicle.g = 9.81;              % Gravity (m/s^2)
    vehicle.rho_air = 1.225;       % Air density (kg/m^3)
    
    %% 2. Transmission
    vehicle.gear_ratio = 8.3;      % Single speed transmission
    vehicle.eta_trans = 0.97;      % Transmission efficiency (97%)
    
    %% 3. Electric Motor (95 kW PMSM)
    % Peak Torque: 215 Nm
    % Peak Power: 95 kW
    % Max Speed: 10500 RPM
    
    vehicle.motor.peak_torque = 215; % Nm
    
    % Generate Efficiency Map (Torque x Speed)
    % Speed range: 0 to 10500 RPM
    % Torque range: 0 to 215 Nm
    
    speed_vec = linspace(0, 10500, 25); % RPM
    torque_vec = linspace(0, 215, 25);  % Nm
    
    [S, T] = meshgrid(speed_vec, torque_vec);
    
    % Synthetic Efficiency Map Generation
    % Base efficiency: 95%
    % Peak region: 97%
    % Very flat map for NEDC (Low load efficiency)
    
    % Normalized coordinates for "sweet spot"
    s_norm = (S - 4200) / 4200; 
    t_norm = (T - 130) / 130;   
    dist = sqrt(s_norm.^2 + t_norm.^2);
    
    % Efficiency function: Peak - small_distance_penalty
    eff_map = 0.97 - 0.05 * dist; % Reduced penalty from 0.10 to 0.05
    eff_map(eff_map < 0.90) = 0.90; % Floor efficiency raised to 90%
    eff_map(eff_map > 0.98) = 0.98; % Cap
    
    vehicle.motor.speed_vec = speed_vec;   % RPM
    vehicle.motor.torque_vec = torque_vec; % Nm
    vehicle.motor.eff_map = eff_map;       % 0.0 to 1.0
    
    % Max Torque Curve
    % Constant 215 Nm until 4200 RPM, then constant power (P=T*w)
    max_torque = zeros(size(speed_vec));
    for i = 1:length(speed_vec)
        w = speed_vec(i) * 2 * pi / 60; % rad/s
        if speed_vec(i) < 4200
            max_torque(i) = 215;
        else
            % T = P / w
            if w > 0
                max_torque(i) = 95000 / w;
            else
                max_torque(i) = 215;
            end
        end
    end
    vehicle.motor.max_torque = max_torque;
    
    %% 4. Inverter
    % Simplified: 98% constant (SiC Inverter)
    vehicle.inverter.eff_map = 0.98 * ones(size(eff_map));
    
    %% 5. Battery Pack (100s6p, 94.5Ah cells, 30.2 kWh total)
    vehicle.battery.capacity_Ah = 15.75; % Per cell (6p = 94.5Ah total)
    vehicle.battery.n_series = 100;
    vehicle.battery.n_parallel = 6;
    
    % Thevenin Parameters (SoC dependent: 0% to 100%)
    soc_vec = 0:0.1:1; % 0 to 1
    
    % OCV Curve (Li-ion typical: 3.0V to 4.2V per cell)
    % Steep drop at end, flat middle, rise at top
    cell_ocv = [2.5, 3.0, 3.15, 3.20, 3.23, 3.26, 3.29, 3.31, 3.33, 3.35, 3.60];
    vehicle.battery.soc_vec = soc_vec;
    vehicle.battery.ocv_vec = cell_ocv * vehicle.battery.n_series; % Pack Voltage
    
    % Internal Resistance (R0) - Reduced for better efficiency
    vehicle.battery.r0_vec = 0.1 * [1.5, 1.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.2]; 
    
    % Polarization R1 and C1 (Simplified constant for now)
    vehicle.battery.r1_vec = 0.02 * ones(size(soc_vec));
    vehicle.battery.c1_vec = 2000 * ones(size(soc_vec));
    
    %% 6. Regenerative Braking
    vehicle.regen.max_power = 40000; % 40 kW limit (Boosted)
    vehicle.regen.max_decel = 8.0;   % 8.0 m/s^2 limit
    
    %% 7. Auxiliaries
    vehicle.aux_power = 350; % Watts (Minimal load)
    
    %% 8. Driver Model
    vehicle.driver.Kp = 45;  % Reduced for smoother driving
    vehicle.driver.Ki = 1.8;  % Reduced for better efficiency
    
    % MPC Parameters (NEW)
    vehicle.driver.mpc.w_track = 1000;   % Weight for speed tracking error
    vehicle.driver.mpc.w_energy = 0.1;   % Weight for energy consumption
    vehicle.driver.mpc.w_smooth = 10;    % Weight for control smoothness (jerk reduction)
    
    % Fuzzy Logic Controller Parameters (NEW)
    vehicle.driver.flc.input1_range = [-20 20]; % Velocity Error (m/s) - Expanded
    vehicle.driver.flc.input2_range = [-100 100]; % Change in Error (m/s^2) - Further Expanded
    vehicle.driver.flc.output_range = [-1 1];   % Accel/Brake Command
    
    %% 9. Battery Degradation Parameters
    % These parameters control how fast the battery ages
    % NOTE: These are BATTERY CHEMISTRY properties - do NOT change when adding HESS!
    % The HESS benefit comes from reduced C-rate stress, not changed coefficients.
    
    vehicle.battery.soh_initial = 100;           % Initial SOH (%)
    vehicle.battery.soh_eol = 80;                % End of life SOH (%)
    
    % Cycle aging: Coefficient for battery chemistry (NMC lithium-ion)
    % CALIBRATED to Tata Nexon EV warranty: 160,000 km / 8 years baseline
    % Interpolated: k_cycle=4.8→94.5k km, k_cycle=2.8→270k km, target 160k km
    vehicle.battery.cycle_aging_coeff = 3.8;     % Cycle aging coefficient (% per sqrt(100 EFC))
    
    % Calendar aging: Coefficient for battery chemistry
    % CALIBRATED: Interpolated for 8 year baseline life
    vehicle.battery.calendar_aging_coeff = 1.7;  % Calendar aging coefficient (%/sqrt(year))
    
    % C-rate stress threshold: Peak C-rate above this value accelerates aging
    % Lower threshold makes the model more sensitive to peak current reduction
    vehicle.battery.c_rate_stress_threshold = 0.3;  % C-rate threshold for stress calculation
    
    vehicle.battery.temp_avg = 25;               % Average operating temp (°C)
    
    %% 10. Vehicle Lifetime Assumptions
    vehicle.lifetime.total_km = 200000;          % Total lifetime distance (km)
    vehicle.lifetime.daily_km = 50;              % Average daily driving (km)
    vehicle.lifetime.years = 10;                 % Expected ownership period (years)

    %% 12. Supercapacitor Parameters (NEW)
    % Using RC equivalent circuit model
    % Sized for 40kW peak power for 10+ seconds
    vehicle.supercap.rated_capacitance = 3000;    % Farads per cell (Maxwell BCAP3000 style)
    vehicle.supercap.rated_voltage_cell = 2.7;    % V per cell
    vehicle.supercap.esr_cell = 0.00029;          % Ohms per cell (0.29 mΩ)
    vehicle.supercap.epr_cell = 10000;            % Ohms (leakage resistance)
    vehicle.supercap.n_series = 134;              % Cells in series (for ~360V bus)
    vehicle.supercap.n_parallel = 6;              % Cells in parallel (for capacity)
    vehicle.supercap.initial_soc = 0.75;           % Initial SOC (75% - literature midpoint)
    vehicle.supercap.soc_max = 0.90;              % Max SOC limit (literature: 85%)
    vehicle.supercap.soc_min = 0.60;              % Min SOC limit (literature: 65%)
    
    % Calculated pack parameters (derived)
    vehicle.supercap.pack_capacitance = (vehicle.supercap.rated_capacitance * ...
        vehicle.supercap.n_parallel) / vehicle.supercap.n_series;  % ~134 F pack
    vehicle.supercap.pack_voltage = vehicle.supercap.rated_voltage_cell * ...
        vehicle.supercap.n_series;  % ~362V
    vehicle.supercap.pack_esr = vehicle.supercap.esr_cell * ...
        vehicle.supercap.n_series / vehicle.supercap.n_parallel;  % ~6.5 mΩ
    vehicle.supercap.max_power = 60000;           % 60kW peak power capability
    vehicle.supercap.energy_capacity_Wh = 0.5 * vehicle.supercap.pack_capacitance * ...
        vehicle.supercap.pack_voltage^2 / 3600;   % ~2.4 kWh usable
    
    %% 13. DC-DC Converter Parameters (NEW)
    vehicle.dcdc.efficiency = 0.95;               % 95% efficiency
    vehicle.dcdc.max_power = 20000;               % 20kW max transfer rate
    vehicle.dcdc.min_power = 100;                 % 100W minimum for operation
    
    %% 14. Energy Management System (EMS) Parameters (NEW)
    vehicle.ems.regen_power_threshold = 5000;     % Power above this goes to supercap (W)
    vehicle.ems.accel_power_threshold = 15000;    % Supercap assists above this (W)
    vehicle.ems.battery_max_charge_rate = 1.0;    % Max 1C charge rate
    vehicle.ems.battery_max_discharge_rate = 2.0; % Max 2C discharge rate
    vehicle.ems.supercap_priority_factor = 0.8;   % 80% priority to supercap for bursts
    
    %% 15. Battery Management System (BMS) Parameters (NEW)
    vehicle.bms.soc_min_charge = 0.20;            % Start BMS charging at 20% SOC
    vehicle.bms.soc_max_charge = 0.80;            % Stop BMS charging at 80% SOC
    vehicle.bms.soc_charge_enable = true;         % Enable SOC-based charging limits
    vehicle.bms.current_limit_charge = 100;       % Max charge current (A)
    vehicle.bms.current_limit_discharge = 200;    % Max discharge current (A)
    
    %% 11. Driving Cycle (NEDC)
    % Generate standard NEDC cycle
    drive_cycle = generate_nedc_cycle();
    
end

function cycle = generate_nedc_cycle()
    % Helper to generate NEDC time/speed vectors
    
    % ECE-15 (Urban) - Repeated 4 times
    t_ece = []; v_ece = [];
    
    % One ECE-15 cycle (195s)
    % Idle 11s
    t_ece = [t_ece, 1:11]; v_ece = [v_ece, zeros(1,11)];
    % Accel to 15 km/h in 4s
    t_ece = [t_ece, 12:15]; v_ece = [v_ece, linspace(0,15,4)];
    % Cruise 15 km/h for 8s
    t_ece = [t_ece, 16:23]; v_ece = [v_ece, 15*ones(1,8)];
    % Decel to 0 in 5s
    t_ece = [t_ece, 24:28]; v_ece = [v_ece, linspace(15,0,5)];
    % Idle 21s
    t_ece = [t_ece, 29:49]; v_ece = [v_ece, zeros(1,21)];
    % Accel to 32 km/h in 12s
    t_ece = [t_ece, 50:61]; v_ece = [v_ece, linspace(0,32,12)];
    % Cruise 32 km/h for 24s
    t_ece = [t_ece, 62:85]; v_ece = [v_ece, 32*ones(1,24)];
    % Decel to 0 in 11s
    t_ece = [t_ece, 86:96]; v_ece = [v_ece, linspace(32,0,11)];
    % Idle 21s
    t_ece = [t_ece, 97:117]; v_ece = [v_ece, zeros(1,21)];
    % Accel to 50 km/h in 26s
    t_ece = [t_ece, 118:143]; v_ece = [v_ece, linspace(0,50,26)];
    % Cruise 50 km/h for 12s
    t_ece = [t_ece, 144:155]; v_ece = [v_ece, 50*ones(1,12)];
    % Decel to 35 km/h in 8s
    t_ece = [t_ece, 156:163]; v_ece = [v_ece, linspace(50,35,8)];
    % Cruise 35 km/h for 13s
    t_ece = [t_ece, 164:176]; v_ece = [v_ece, 35*ones(1,13)];
    % Decel to 0 in 12s
    t_ece = [t_ece, 177:188]; v_ece = [v_ece, linspace(35,0,12)];
    % Idle 7s
    t_ece = [t_ece, 189:195]; v_ece = [v_ece, zeros(1,7)];
    
    % Repeat 4 times
    full_v = [];
    for i=1:4
        full_v = [full_v, v_ece];
    end
    
    % EUDC (Extra Urban) - 400s
    v_eudc = zeros(1, 400);
    % Simplified EUDC construction for brevity (can be detailed if needed)
    % For now, let's use the previous simple EUDC logic or just append zeros if complex
    % Let's use a proper approximation:
    
    % Idle 20s
    idx = 1;
    v_eudc(idx:idx+19) = 0; idx=idx+20;
    % Accel to 70 in 41s
    v_eudc(idx:idx+40) = linspace(0,70,41); idx=idx+41;
    % Cruise 70 for 50s
    v_eudc(idx:idx+49) = 70; idx=idx+50;
    % Decel to 50 in 8s
    v_eudc(idx:idx+7) = linspace(70,50,8); idx=idx+8;
    % Cruise 50 for 69s
    v_eudc(idx:idx+68) = 50; idx=idx+69;
    % Accel to 70 in 13s
    v_eudc(idx:idx+12) = linspace(50,70,13); idx=idx+13;
    % Cruise 70 for 50s
    v_eudc(idx:idx+49) = 70; idx=idx+50;
    % Accel to 100 in 35s
    v_eudc(idx:idx+34) = linspace(70,100,35); idx=idx+35;
    % Cruise 100 for 30s
    v_eudc(idx:idx+29) = 100; idx=idx+30;
    % Accel to 120 in 20s
    v_eudc(idx:idx+19) = linspace(100,120,20); idx=idx+20;
    % Cruise 120 for 10s
    v_eudc(idx:idx+9) = 120; idx=idx+10;
    % Decel to 0 in 34s
    v_eudc(idx:idx+33) = linspace(120,0,34); idx=idx+34;
    % Idle 20s
    v_eudc(idx:end) = 0;
    
    full_v = [full_v, v_eudc];
    
    cycle.time = 0:(length(full_v)-1);
    cycle.velocity_kmh = full_v;
    cycle.velocity_ms = full_v / 3.6;
end

function cycle = generate_linear_ramp_cycle()
    % Helper to generate Linear Ramp driving cycle
    % Phase 1 (0-20s): Linear acceleration from 0 to 72 km/h (20 m/s)
    % Phase 2 (20-40s): Constant speed at 72 km/h
    % Total duration: 40 seconds
    
    % Phase 1: Linear ramp (0-20s)
    t_ramp = 0:20;
    v_ramp_kmh = linspace(0, 72, length(t_ramp));
    
    % Phase 2: Constant speed (21-40s)
    t_constant = 21:40;
    v_constant_kmh = 72 * ones(1, length(t_constant));
    
    % Combine
    full_v = [v_ramp_kmh, v_constant_kmh];
    
    cycle.time = 0:(length(full_v)-1);
    cycle.velocity_kmh = full_v;
    cycle.velocity_ms = full_v / 3.6;
end

function cycle = generate_midc_cycle()
    % Helper to generate Modified Indian Driving Cycle (MIDC)
    % MIDC is essentially NEDC with max speed limited to 90 km/h
    % Used for Type-1 testing in India (BSIV 4-wheeled vehicles)
    
    % ECE-15 (Urban) - Repeated 4 times (same as NEDC)
    t_ece = []; v_ece = [];
    
    % One ECE-15 cycle (195s)
    % Idle 11s
    t_ece = [t_ece, 1:11]; v_ece = [v_ece, zeros(1,11)];
    % Accel to 15 km/h in 4s
    t_ece = [t_ece, 12:15]; v_ece = [v_ece, linspace(0,15,4)];
    % Cruise 15 km/h for 8s
    t_ece = [t_ece, 16:23]; v_ece = [v_ece, 15*ones(1,8)];
    % Decel to 0 in 5s
    t_ece = [t_ece, 24:28]; v_ece = [v_ece, linspace(15,0,5)];
    % Idle 21s
    t_ece = [t_ece, 29:49]; v_ece = [v_ece, zeros(1,21)];
    % Accel to 32 km/h in 12s
    t_ece = [t_ece, 50:61]; v_ece = [v_ece, linspace(0,32,12)];
    % Cruise 32 km/h for 24s
    t_ece = [t_ece, 62:85]; v_ece = [v_ece, 32*ones(1,24)];
    % Decel to 0 in 11s
    t_ece = [t_ece, 86:96]; v_ece = [v_ece, linspace(32,0,11)];
    % Idle 21s
    t_ece = [t_ece, 97:117]; v_ece = [v_ece, zeros(1,21)];
    % Accel to 50 km/h in 26s
    t_ece = [t_ece, 118:143]; v_ece = [v_ece, linspace(0,50,26)];
    % Cruise 50 km/h for 12s
    t_ece = [t_ece, 144:155]; v_ece = [v_ece, 50*ones(1,12)];
    % Decel to 35 km/h in 8s
    t_ece = [t_ece, 156:163]; v_ece = [v_ece, linspace(50,35,8)];
    % Cruise 35 km/h for 13s
    t_ece = [t_ece, 164:176]; v_ece = [v_ece, 35*ones(1,13)];
    % Decel to 0 in 12s
    t_ece = [t_ece, 177:188]; v_ece = [v_ece, linspace(35,0,12)];
    % Idle 7s
    t_ece = [t_ece, 189:195]; v_ece = [v_ece, zeros(1,7)];
    
    % Repeat 4 times
    full_v = [];
    for i=1:4
        full_v = [full_v, v_ece];
    end
    
    % EUDC (Extra Urban) - Modified for MIDC (max 90 km/h instead of 120 km/h)
    v_eudc = zeros(1, 400);
    
    % Idle 20s
    idx = 1;
    v_eudc(idx:idx+19) = 0; idx=idx+20;
    % Accel to 70 in 41s
    v_eudc(idx:idx+40) = linspace(0,70,41); idx=idx+41;
    % Cruise 70 for 50s
    v_eudc(idx:idx+49) = 70; idx=idx+50;
    % Decel to 50 in 8s
    v_eudc(idx:idx+7) = linspace(70,50,8); idx=idx+8;
    % Cruise 50 for 69s
    v_eudc(idx:idx+68) = 50; idx=idx+69;
    % Accel to 70 in 13s
    v_eudc(idx:idx+12) = linspace(50,70,13); idx=idx+13;
    % Cruise 70 for 50s
    v_eudc(idx:idx+49) = 70; idx=idx+50;
    % Accel to 90 in 35s (MIDC limit: 90 km/h instead of 100 km/h)
    v_eudc(idx:idx+34) = linspace(70,90,35); idx=idx+35;  
    % Cruise 90 for 30s (MIDC limit: 90 km/h instead of 100 km/h)
    v_eudc(idx:idx+29) = 90; idx=idx+30;
    % No further acceleration (MIDC max is 90 km/h, NEDC goes to 120 km/h)
    % Decel directly to 0 in 54s (longer decel from 90 instead of 120)
    v_eudc(idx:idx+53) = linspace(90,0,54); idx=idx+54;
    % Idle 20s
    v_eudc(idx:end) = 0;
    
    full_v = [full_v, v_eudc];
    
    cycle.time = 0:(length(full_v)-1);
    cycle.velocity_kmh = full_v;
    cycle.velocity_ms = full_v / 3.6;
end

