% build_simulink_model.m - Create BMW i3 Simulink Model (Simplified Visualization)

% Close any existing models with the same name
try
    close_system('bmw_i3_model', 0);
catch
    % Model doesn't exist yet, that's fine
end

% Create new model
model_name = 'bmw_i3_model';
new_system(model_name);
open_system(model_name);

% Load vehicle parameters
[vehicle, cycle] = setup_params();

% Configure simulation parameters
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '1');
set_param(model_name, 'StopTime', num2str(length(cycle.time)-1));

%% Create a minimal working model for visualization

% 1. Clock source
add_block('simulink/Sources/Clock', [model_name '/Clock']);
set_param([model_name '/Clock'], 'Position', [30, 100, 60, 130]);

% 2. Velocity Lookup Table (1D)
add_block('simulink/Lookup Tables/1-D Lookup Table', [model_name '/Velocity_Profile']);
set_param([model_name '/Velocity_Profile'], 'Position', [120, 90, 200, 140]);

% 3. Unit conversion (km/h to m/s)
add_block('simulink/Math Operations/Gain', [model_name '/kmh_to_ms']);
set_param([model_name '/kmh_to_ms'], 'Gain', '1/3.6');
set_param([model_name '/kmh_to_ms'], 'Position', [250, 100, 290, 130]);

% 4. Scope for speed visualization
add_block('simulink/Sinks/Scope', [model_name '/Speed_Scope']);
set_param([model_name '/Speed_Scope'], 'Position', [350, 95, 390, 135]);

% 5. Display for speed
add_block('simulink/Sinks/Display', [model_name '/Speed_Display']);
set_param([model_name '/Speed_Display'], 'Position', [350, 160, 450, 190]);

% 6. To Workspace for logging speed data
add_block('simulink/Sinks/To Workspace', [model_name '/Log_Speed']);
set_param([model_name '/Log_Speed'], 'VariableName', 'speed_log');
set_param([model_name '/Log_Speed'], 'Position', [350, 220, 450, 250]);
set_param([model_name '/Log_Speed'], 'SaveFormat', 'Array');

%% Additional blocks for energy calculation (simplified)

% 7. Constant for vehicle mass
add_block('simulink/Sources/Constant', [model_name '/Vehicle_Mass']);
set_param([model_name '/Vehicle_Mass'], 'Value', num2str(vehicle.M_vehicle));
set_param([model_name '/Vehicle_Mass'], 'Position', [30, 300, 100, 330]);

% 8. Display for mass
add_block('simulink/Sinks/Display', [model_name '/Mass_Display']);
set_param([model_name '/Mass_Display'], 'Position', [150, 295, 250, 335]);

% 9. Constant for auxiliary power
add_block('simulink/Sources/Constant', [model_name '/Aux_Power']);
set_param([model_name '/Aux_Power'], 'Value', num2str(vehicle.aux_power));
set_param([model_name '/Aux_Power'], 'Position', [30, 380, 100, 410]);

% 10. Display for auxiliary power
add_block('simulink/Sinks/Display', [model_name '/Aux_Display']);
set_param([model_name '/Aux_Display'], 'Position', [150, 375, 250, 415]);

%% Connect the blocks
add_line(model_name, 'Clock/1', 'Velocity_Profile/1');
add_line(model_name, 'Velocity_Profile/1', 'kmh_to_ms/1');
add_line(model_name, 'kmh_to_ms/1', 'Speed_Scope/1');
add_line(model_name, 'kmh_to_ms/1', 'Speed_Display/1');
add_line(model_name, 'kmh_to_ms/1', 'Log_Speed/1');
add_line(model_name, 'Vehicle_Mass/1', 'Mass_Display/1');
add_line(model_name, 'Aux_Power/1', 'Aux_Display/1');

%% Save the model
save_system(model_name);

fprintf('==========================================================\n');
fprintf('Simulink model "%s.slx" created successfully!\n', model_name);
fprintf('==========================================================\n');
fprintf('Model location: %s\n', pwd);
fprintf('\nModel structure:\n');
fprintf('  - Clock -> Velocity Profile (NEDC Cycle Lookup Table)\n');
fprintf('  - Velocity -> Unit Conversion (km/h to m/s)\n');
fprintf('  - Velocity -> Scope (for visualization)\n');
fprintf('  - Velocity -> Display & Data Logging\n');
fprintf('  - Vehicle Parameters displayed as constants\n');
fprintf('\nNext step: Run "run_simulink_simulation.m" to execute!\n');
fprintf('==========================================================\n');
