% analyze_results.m - Detailed Energy Analysis of Simulink Model

fprintf('\n==================================================\n');
fprintf('DETAILED ENERGY ANALYSIS\n');
fprintf('==================================================\n');

% Check for data
if ~exist('simOut', 'var') && ~exist('vel_actual', 'var')
    fprintf('Error: No simulation data found. Run "run_full_simulation" first.\n');
    return;
end

% Extract data if needed
if exist('simOut', 'var')
    try
        time = simOut.sim_time;
        vel = simOut.vel_actual.signals.values; % m/s
        p_elec = simOut.p_elec.signals.values; % Watts
        soc = simOut.soc.signals.values;
        % We need to reconstruct forces since they aren't logged directly
        % Or we can use the parameters to estimate them
    catch
        fprintf('Error extracting data from simOut.\n');
        return;
    end
else
    % Use workspace variables
    vel = vel_actual;
    % time, p_elec should be there
end

% Ensure time is column vector
if size(time, 2) > 1; time = time'; end
if size(vel, 2) > 1; vel = vel'; end
if size(p_elec, 2) > 1; p_elec = p_elec'; end

dt = [diff(time); 0];

% 1. Calculate Propulsion vs Regen Energy
p_prop = p_elec;
p_prop(p_prop < 0) = 0;
e_prop_Wh = trapz(time, p_prop) / 3600;

p_regen = p_elec;
p_regen(p_regen > 0) = 0;
e_regen_Wh = trapz(time, p_regen) / 3600;

e_net_Wh = e_prop_Wh + e_regen_Wh; % Regen is negative

fprintf('Electrical Energy Breakdown:\n');
fprintf('  Propulsion Energy:  %.2f Wh\n', e_prop_Wh);
fprintf('  Regen Energy:       %.2f Wh\n', e_regen_Wh);
fprintf('  Net Energy:         %.2f Wh\n', e_net_Wh);
fprintf('  Regen Recovery %%:   %.1f %%\n', abs(e_regen_Wh / e_prop_Wh) * 100);

% 2. Estimate Mechanical Loads (Based on parameters)
% We need to load vehicle struct
if ~exist('vehicle', 'var')
    fprintf('Warning: "vehicle" struct not found. Cannot calculate mechanical loads.\n');
    setup_full_params; % Try to load default
end

mass = vehicle.M_vehicle;
g = vehicle.g;
rho = vehicle.rho_air;
Cd = vehicle.Cd;
A = vehicle.A_frontal;
Crr = vehicle.C_RR;

% Aero Force
f_aero = 0.5 * rho * Cd * A * vel.^2;
p_aero = f_aero .* vel;
e_aero_Wh = trapz(time, p_aero) / 3600;

% Rolling Resistance
f_roll = Crr * mass * g;
p_roll = f_roll .* vel; % Only when moving
p_roll(vel < 0.1) = 0;
e_roll_Wh = trapz(time, p_roll) / 3600;

% Inertial Power (M*a*v)
accel = [0; diff(vel) ./ diff(time)];
f_inertial = mass * 1.05 * accel; % 1.05 for rotational inertia
p_inertial = f_inertial .* vel;

% Split Inertial into Accel (Positive) and Decel (Negative)
p_accel = p_inertial; p_accel(p_accel < 0) = 0;
e_accel_Wh = trapz(time, p_accel) / 3600;

p_decel = p_inertial; p_decel(p_decel > 0) = 0;
e_decel_Wh = trapz(time, p_decel) / 3600;

fprintf('\nMechanical Load Estimates (at Wheels):\n');
fprintf('  Aero Energy:        %.2f Wh\n', e_aero_Wh);
fprintf('  Rolling Energy:     %.2f Wh\n', e_roll_Wh);
fprintf('  Kinetic Energy (+): %.2f Wh\n', e_accel_Wh);
fprintf('  Kinetic Energy (-): %.2f Wh\n', e_decel_Wh);
fprintf('  Total Mech Required:%.2f Wh (Aero + Roll + Kinetic+)\n', e_aero_Wh + e_roll_Wh + e_accel_Wh);

% 3. Efficiency Analysis
% Avg Powertrain Efficiency = Mech Required / Elec Propulsion
mech_req = e_aero_Wh + e_roll_Wh + e_accel_Wh;
eff_prop = mech_req / e_prop_Wh * 100;

fprintf('\nEfficiency Analysis:\n');
fprintf('  Est. Powertrain Eff:%.1f %%\n', eff_prop);

% 4. Braking Analysis
% Energy available for regen = Kinetic Energy (-) - Aero (during decel) - Roll (during decel)
% This is rough, but gives an idea.
% A better check: Compare Electrical Regen to Kinetic Decel
regen_eff = abs(e_regen_Wh / e_decel_Wh) * 100;
fprintf('  Braking Capture Eff:%.1f %% (Regen Elec / Kinetic Decel)\n', regen_eff);

fprintf('\nDIAGNOSIS:\n');
if regen_eff < 50
    fprintf('  [CRITICAL] Regen efficiency is very low (<50%%). \n');
    fprintf('  Likely Causes:\n');
    fprintf('  1. Regen speed threshold too high (cutting off at low speeds).\n');
    fprintf('  2. Battery SoC limit or Power limit reached.\n');
    fprintf('  3. Friction brakes engaging too early.\n');
end

if eff_prop < 80
    fprintf('  [CRITICAL] Propulsion efficiency is low (<80%%).\n');
    fprintf('  Likely Causes:\n');
    fprintf('  1. Motor/Inverter efficiency map poor.\n');
    fprintf('  2. Gear ratio causing inefficient operating points.\n');
end

fprintf('==================================================\n');
