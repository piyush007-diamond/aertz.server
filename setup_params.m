function [vehicle, drive_cycle] = setup_params()
    % setup_params - Define BMW i3 parameters and NEDC driving cycle
    
    %% Vehicle Parameters (BMW i3 2014 approx)
    vehicle.M_vehicle = 1195 + 75; % Curb weight + driver (kg)
    vehicle.Cd = 0.29;             % Drag coefficient
    vehicle.A_frontal = 2.38;      % Frontal area (m^2)
    vehicle.C_RR = 0.01;           % Rolling resistance coefficient (base)
    vehicle.r_tire = 0.35;         % Tire radius (m) (approx for 155/70 R19)
    vehicle.gear_ratio = 9.7;      % Transmission gear ratio
    vehicle.eta_trans = 0.98;      % Transmission efficiency
    vehicle.eta_motor_avg = 0.89;  % Average motor efficiency (tuned)
    vehicle.eta_inverter_avg = 0.92; % Average inverter efficiency (tuned)
    vehicle.eta_battery = 0.90;    % Battery efficiency (charge/discharge) (tuned)
    vehicle.aux_power = 300;       % Auxiliary load (W) - NEDC
    vehicle.regen_fraction = 0.6;  % Fraction of braking energy recovered (simplified logic)
    
    % Constants
    vehicle.g = 9.81;
    vehicle.rho_air = 1.225;
    
    %% Driving Cycle: NEDC (New European Driving Cycle)
    % Constructed from standard segments
    
    dt = 1; % Time step (s)
    
    % ECE-15 Urban Cycle (195 seconds)
    % Time (s), Speed (km/h) points
    ece_time = [0, 11, 15, 23, 28, 49, 54, 64, 69, 78, 83, 96, 101, 112, 117, 133, 138, 143, 155, 163, 178, 188, 195];
    ece_speed = [0, 0, 15, 15, 32, 32, 0, 0, 50, 50, 35, 35, 0, 0, 32, 32, 10, 10, 0, 0, 32, 32, 0]; % Simplified/Approx points
    % Note: The above is a rough approximation. A more accurate way is to define the specific acceleration/cruise/decel phases.
    % For this task, we will use a standard generated NEDC profile if available, or construct a piecewise linear one.
    % Let's use a more detailed construction for better accuracy.
    
    t_ece = [];
    v_ece = [];
    
    % Helper to append segment
    function add_segment(duration, v_start, v_end)
        t_seg = 1:duration;
        v_seg = linspace(v_start, v_end, duration);
        if ~isempty(v_ece)
            v_ece = [v_ece, v_seg];
        else
            v_ece = v_seg;
        end
    end

    % ECE-15 Definition (repeated 4 times)
    % 1. Idle 11s
    % 2. Accel to 15 km/h in 4s
    % 3. Cruise 15 km/h for 8s
    % 4. Decel to 0 in 5s
    % 5. Idle 21s
    % 6. Accel to 32 km/h in 12s
    % 7. Cruise 32 km/h for 24s
    % 8. Decel to 0 in 11s
    % 9. Idle 21s
    % 10. Accel to 50 km/h in 26s
    % 11. Cruise 50 km/h for 12s
    % 12. Decel to 35 km/h in 8s
    % 13. Cruise 35 km/h for 13s
    % 14. Decel to 0 in 12s
    % 15. Idle 7s
    
    one_ece_v = [];
    one_ece_v = [one_ece_v, zeros(1, 11)]; % Idle
    one_ece_v = [one_ece_v, linspace(0, 15, 4)]; % Accel
    one_ece_v = [one_ece_v, 15 * ones(1, 8)]; % Cruise
    one_ece_v = [one_ece_v, linspace(15, 0, 5)]; % Decel
    one_ece_v = [one_ece_v, zeros(1, 21)]; % Idle
    one_ece_v = [one_ece_v, linspace(0, 32, 12)]; % Accel
    one_ece_v = [one_ece_v, 32 * ones(1, 24)]; % Cruise
    one_ece_v = [one_ece_v, linspace(32, 0, 11)]; % Decel
    one_ece_v = [one_ece_v, zeros(1, 21)]; % Idle
    one_ece_v = [one_ece_v, linspace(0, 50, 26)]; % Accel
    one_ece_v = [one_ece_v, 50 * ones(1, 12)]; % Cruise
    one_ece_v = [one_ece_v, linspace(50, 35, 8)]; % Decel
    one_ece_v = [one_ece_v, 35 * ones(1, 13)]; % Cruise
    one_ece_v = [one_ece_v, linspace(35, 0, 12)]; % Decel
    one_ece_v = [one_ece_v, zeros(1, 7)]; % Idle
    
    % EUDC Definition (Extra Urban)
    % 1. Idle 20s
    % 2. Accel to 70 km/h in 41s
    % 3. Cruise 70 km/h for 50s
    % 4. Decel to 50 km/h in 8s
    % 5. Cruise 50 km/h for 69s
    % 6. Accel to 70 km/h in 13s
    % 7. Cruise 70 km/h for 50s
    % 8. Accel to 100 km/h in 35s
    % 9. Cruise 100 km/h for 30s
    % 10. Accel to 120 km/h in 20s
    % 11. Cruise 120 km/h for 10s
    % 12. Decel to 0 in 34s
    % 13. Idle 20s
    
    eudc_v = [];
    eudc_v = [eudc_v, zeros(1, 20)];
    eudc_v = [eudc_v, linspace(0, 70, 41)];
    eudc_v = [eudc_v, 70 * ones(1, 50)];
    eudc_v = [eudc_v, linspace(70, 50, 8)];
    eudc_v = [eudc_v, 50 * ones(1, 69)];
    eudc_v = [eudc_v, linspace(50, 70, 13)];
    eudc_v = [eudc_v, 70 * ones(1, 50)];
    eudc_v = [eudc_v, linspace(70, 100, 35)];
    eudc_v = [eudc_v, 100 * ones(1, 30)];
    eudc_v = [eudc_v, linspace(100, 120, 20)];
    eudc_v = [eudc_v, 120 * ones(1, 10)];
    eudc_v = [eudc_v, linspace(120, 0, 34)];
    eudc_v = [eudc_v, zeros(1, 20)];
    
    % Combine: 4x ECE + 1x EUDC
    full_cycle_v = [one_ece_v, one_ece_v, one_ece_v, one_ece_v, eudc_v];
    
    % Create time vector
    drive_cycle.time = 0:(length(full_cycle_v)-1);
    drive_cycle.velocity_kmh = full_cycle_v;
    drive_cycle.velocity_ms = full_cycle_v / 3.6;
    
end
