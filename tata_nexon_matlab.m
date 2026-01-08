% run_simulation.m - Execute BMW i3 Simulation (Quasi-Static)

% 1. Setup Parameters
[vehicle, cycle] = setup_params();

fprintf('Starting Simulation: BMW i3 on NEDC Cycle...\n');
fprintf('Total Time: %d s\n', length(cycle.time));
fprintf('Total Distance: %.2f km\n', sum(cycle.velocity_ms) * 1 / 1000);

% 2. Initialize State Variables
N = length(cycle.time);
P_batt = zeros(1, N);
E_cum = zeros(1, N);
SoC = zeros(1, N);
SoC(1) = 100; % Initial SoC %
battery_capacity_kWh = 18.8; % BMW i3 (60 Ah) usable
battery_energy_J = battery_capacity_kWh * 3600 * 1000;

% 3. Simulation Loop
for i = 1:N-1
    v = cycle.velocity_ms(i);       % Current speed (m/s)
    v_next = cycle.velocity_ms(i+1); % Next speed (m/s)
    a = (v_next - v) / 1;           % Acceleration (m/s^2) (dt=1)
    
    % Forces
    F_aero = 0.5 * vehicle.rho_air * vehicle.A_frontal * vehicle.Cd * v^2;
    F_roll = vehicle.C_RR * vehicle.M_vehicle * vehicle.g; % Simplified (flat road)
    F_inertia = vehicle.M_vehicle * a;
    F_grade = 0; % Flat road
    
    F_tractive = F_aero + F_roll + F_inertia + F_grade;
    
    % Power at Wheels
    P_wheels = F_tractive * v; % Watts
    
    % Powertrain
    if P_wheels >= 0
        % Traction Mode
        P_motor_out = P_wheels / vehicle.eta_trans;
        P_motor_in = P_motor_out / vehicle.eta_motor_avg;
        P_inverter_in = P_motor_in / vehicle.eta_inverter_avg;
        P_batt_load = P_inverter_in + vehicle.aux_power;
        
        % Battery Discharge
        P_batt(i) = P_batt_load / vehicle.eta_battery;
        
    else
        % Regenerative Braking Mode
        % Limit regen (simplified)
        P_regen_avail = abs(P_wheels) * vehicle.eta_trans * vehicle.eta_motor_avg * vehicle.eta_inverter_avg;
        
        % Apply regen fraction (not all braking is regen)
        P_regen_captured = P_regen_avail * vehicle.regen_fraction;
        
        % Net Battery Power (Negative = Charging)
        P_batt_load = -P_regen_captured + vehicle.aux_power;
        
        if P_batt_load < 0
            % Charging
            P_batt(i) = P_batt_load * vehicle.eta_battery;
        else
            % Discharging (Aux > Regen)
            P_batt(i) = P_batt_load / vehicle.eta_battery;
        end
    end
    
    % Energy Accumulation
    E_cum(i+1) = E_cum(i) + P_batt(i) * 1; % Joules (dt=1)
    
    % SoC Update
    SoC(i+1) = SoC(i) - (P_batt(i) * 1 / battery_energy_J) * 100;
end

% 4. Results
total_energy_J = E_cum(end);
total_energy_kWh = total_energy_J / (3600 * 1000);
total_dist_km = sum(cycle.velocity_ms) * 1 / 1000;

energy_consumption_Wh_km = (total_energy_kWh * 1000) / total_dist_km;

fprintf('--------------------------------------------------\n');
fprintf('Simulation Complete.\n');
fprintf('Total Distance: %.2f km\n', total_dist_km);
fprintf('Total Energy Consumed: %.2f kWh\n', total_energy_kWh);
fprintf('Estimated Energy Consumption: %.2f Wh/km\n', energy_consumption_Wh_km);
fprintf('Final SoC: %.2f %%\n', SoC(end));
fprintf('--------------------------------------------------\n');

% Validation Check
expected_val = 135; % Wh/km (approx NEDC for i3)
error_pct = (energy_consumption_Wh_km - expected_val) / expected_val * 100;
fprintf('Validation vs Benchmark (%.0f Wh/km): Error = %.2f %%\n', expected_val, error_pct);

if abs(error_pct) < 6
    fprintf('Status: PASS\n');
else
    fprintf('Status: FAIL (Check parameters)\n');
end
