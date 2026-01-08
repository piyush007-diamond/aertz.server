% run_simulink_simulation.m - Execute the BMW i3 Simulink Model

% Load vehicle parameters and driving cycle
[vehicle, cycle] = setup_params();

% Prepare lookup table data for velocity profile
assignin('base', 'velocity_breakpoints', cycle.time);
assignin('base', 'velocity_table', cycle.velocity_kmh);

% Load the model
load_system('bmw_i3_model');

% Configure the lookup table
set_param('bmw_i3_model/Velocity_Profile', 'BreakpointsForDimension1', 'velocity_breakpoints');
set_param('bmw_i3_model/Velocity_Profile', 'Table', 'velocity_table');

fprintf('==========================================================\n');
fprintf('Running Simulink simulation: bmw_i3_model\n');
fprintf('==========================================================\n');
fprintf('Driving Cycle: NEDC\n');
fprintf('Simulation time: %d seconds (%.1f minutes)\n', cycle.time(end), cycle.time(end)/60);
fprintf('Total distance: %.2f km\n', sum(cycle.velocity_ms) * 1 / 1000);
fprintf('--------------------------------------------------\n');

% Run the simulation
sim_out = sim('bmw_i3_model');

fprintf('Simulation completed successfully!\n');
fprintf('==========================================================\n');

% Open the model for viewing
open_system('bmw_i3_model');

fprintf('\n');
fprintf('==========================================================\n');
fprintf('       The Simulink model is now open in MATLAB!        \n');
fprintf('==========================================================\n');
fprintf('\n');
fprintf('🚗 BMW i3 Energy Consumption Simulation\n');
fprintf('-------------------------------------------\n');
fprintf('What you can do:\n');
fprintf('\n');
fprintf('1. VIEW THE MODEL:\n');
fprintf('   - See the complete NEDC driving cycle flow\n');
fprintf('   - Observe the signal routing and blocks\n');
fprintf('   - Check vehicle parameters\n');
fprintf('\n');
fprintf('2. VISUALIZE THE RESULTS:\n');
fprintf('   - Double-click "Speed_Scope" to see velocity profile\n');
fprintf('   - View the "Speed_Display" for current speed (m/s)\n');
fprintf('   - Check vehicle mass and auxiliary power displays\n');
fprintf('\n');
fprintf('3. INTERACT WITH THE MODEL:\n');
fprintf('   - Click the ▶ Run button to re-simulate\n');
fprintf('   - Modify parameters in any block (double-click)\n');
fprintf('   - Save the model (Ctrl+S) if you make changes\n');
fprintf('\n');
fprintf('4. ANALYZE THE DATA:\n');
fprintf('   - Logged speed data is in "speed_log" variable\n');
fprintf('   - Type "plot(speed_log)" to plot speed over time\n');
fprintf('\n');
fprintf('==========================================================\n');
