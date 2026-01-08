function build_fuzzy_model()
    % build_fuzzy_model - Programmatically construct the BMW i3 EV Model with Fuzzy Logic Driver
    
    model_name = 'bmw_i3_fuzzy_model';
    
    % Close if open
    try close_system(model_name, 0); catch; end
    
    % Create new model
    new_system(model_name);
    open_system(model_name);
    
    % Setup Model Configuration
    set_param(model_name, 'Solver', 'ode45');
    set_param(model_name, 'RelTol', '1e-6'); 
    set_param(model_name, 'StopTime', '1180'); % NEDC duration
    
    % Load Parameters
    fprintf('Loading parameters...\n');
    [vehicle, cycle] = setup_full_params();
    assignin('base', 'vehicle', vehicle);
    assignin('base', 'cycle', cycle);
    
    %% 1. Build Subsystems
    fprintf('Building Subsystems...\n');
    
    build_fuzzy_driver_subsystem(model_name, vehicle); % NEW: FLC Driver
    build_regen_subsystem(model_name);      % Reused
    build_motor_subsystem(model_name);      % Reused
    build_transmission_subsystem(model_name); % Reused
    build_dynamics_subsystem(model_name);   % Reused
    build_battery_subsystem(model_name);    % Reused
    build_aux_subsystem(model_name);        % Reused
    
    % HESS Subsystems (Placeholders - to be added)
    build_supercapacitor_subsystem(model_name);
    build_dcdc_controller_subsystem(model_name);
    build_ems_subsystem(model_name);
    build_bms_subsystem(model_name);
    
    %% 2. Connect Subsystems
    fprintf('Connecting Subsystems...\n');
    connect_all_subsystems(model_name);
    
    %% 3. Add Logging
    add_logging(model_name);
    
    %% 4. Save
    save_system(model_name);
    fprintf('Model "%s" created successfully!\n', model_name);
end

%% Subsystem Builders

function build_fuzzy_driver_subsystem(model_name, vehicle)
    % Fuzzy Logic Driver Model
    % Inputs: Vel_Ref, Vel_Actual
    % Outputs: Accel_Cmd, Brake_Cmd
    
    sub_name = [model_name '/Driver_Model'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [50, 50, 150, 150]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    % Inputs
    add_block('simulink/Sources/In1', [sub_name '/Vel_Ref'], 'Position', [20, 50, 50, 70]);
    add_block('simulink/Sources/In1', [sub_name '/Vel_Actual'], 'Position', [20, 100, 50, 120]);
    
    % Calculate Error and dError
    add_block('simulink/Math Operations/Subtract', [sub_name '/Error_Calc'], 'Position', [100, 60, 130, 90]);
    add_line(sub_name, 'Vel_Ref/1', 'Error_Calc/1');
    add_line(sub_name, 'Vel_Actual/1', 'Error_Calc/2');
    
    add_block('simulink/Continuous/Derivative', [sub_name '/dError_Calc'], 'Position', [150, 100, 180, 130]);
    add_line(sub_name, 'Error_Calc/1', 'dError_Calc/1');
    
    % --- Programmatic FIS Generation ---
    fprintf('Generating Fuzzy Inference System...\n');
    fis = mamfis('Name', 'Driver_FIS');
    
    % Input 1: Velocity Error - Wide overlapping MFs
    fis = addInput(fis, vehicle.driver.flc.input1_range, 'Name', 'Error');
    fis = addMF(fis, 'Error', 'trapmf', [-40 -20 -3 -1], 'Name', 'NB'); % Negative Big
    fis = addMF(fis, 'Error', 'trimf', [-3 -1.5 0], 'Name', 'NS');       % Negative Small
    fis = addMF(fis, 'Error', 'trimf', [-1.5 0 1.5], 'Name', 'ZE');      % Zero
    fis = addMF(fis, 'Error', 'trimf', [0 1.5 3], 'Name', 'PS');         % Positive Small
    fis = addMF(fis, 'Error', 'trapmf', [1 3 20 40], 'Name', 'PB');      % Positive Big
    
    % Input 2: Change in Error - Wide overlapping MFs
    fis = addInput(fis, vehicle.driver.flc.input2_range, 'Name', 'dError');
    fis = addMF(fis, 'dError', 'trapmf', [-200 -100 -10 -2], 'Name', 'NB'); % Expanded NB
    fis = addMF(fis, 'dError', 'trimf', [-10 -3 0], 'Name', 'NS');
    fis = addMF(fis, 'dError', 'trimf', [-3 0 3], 'Name', 'ZE');
    fis = addMF(fis, 'dError', 'trimf', [0 3 10], 'Name', 'PS');
    fis = addMF(fis, 'dError', 'trapmf', [2 10 100 200], 'Name', 'PB'); % Expanded PB
    
    % Output: Command (-1 to 1)
    fis = addOutput(fis, vehicle.driver.flc.output_range, 'Name', 'Command');
    fis = addMF(fis, 'Command', 'trapmf', [-1.5 -1 -0.6 -0.3], 'Name', 'Brake_Hard');
    fis = addMF(fis, 'Command', 'trimf', [-0.6 -0.3 0], 'Name', 'Brake_Soft');
    fis = addMF(fis, 'Command', 'trimf', [-0.1 0 0.1], 'Name', 'Maintain');
    fis = addMF(fis, 'Command', 'trimf', [0 0.3 0.6], 'Name', 'Accel_Soft');
    fis = addMF(fis, 'Command', 'trapmf', [0.3 0.6 1 1.5], 'Name', 'Accel_Hard');
    
    % FULL 25-RULE FLC MATRIX
    % Error: NB(1)=too fast, NS(2)=slightly fast, ZE(3)=on target, PS(4)=slightly slow, PB(5)=too slow
    % dError: NB(1)=error decreasing fast, NS(2)=decreasing, ZE(3)=stable, PS(4)=increasing, PB(5)=increasing fast
    % Command: Brake_Hard(1), Brake_Soft(2), Maintain(3), Accel_Soft(4), Accel_Hard(5)
    %
    % Rule Matrix Design:
    %              dError
    %          NB    NS    ZE    PS    PB
    % Error NB  2     1     1     1     1   (too fast - brake, but ease if correcting)
    %       NS  3     2     2     1     1   (slightly fast)
    %       ZE  4     4     3     2     2   (on target - damping based on dError)
    %       PS  5     5     4     4     3   (slightly slow)
    %       PB  5     5     5     5     4   (too slow - accel, but ease if correcting)
    
    ruleList = [
        % Error=NB (too fast - need to brake)
        1 1 2 1 1; % NB,NB -> Brake_Soft (already correcting fast)
        1 2 1 1 1; % NB,NS -> Brake_Hard
        1 3 1 1 1; % NB,ZE -> Brake_Hard
        1 4 1 1 1; % NB,PS -> Brake_Hard (getting worse)
        1 5 1 1 1; % NB,PB -> Brake_Hard (getting much worse)
        
        % Error=NS (slightly too fast)
        2 1 3 1 1; % NS,NB -> Maintain (correcting well)
        2 2 2 1 1; % NS,NS -> Brake_Soft
        2 3 2 1 1; % NS,ZE -> Brake_Soft
        2 4 1 1 1; % NS,PS -> Brake_Hard (getting worse)
        2 5 1 1 1; % NS,PB -> Brake_Hard (getting much worse)
        
        % Error=ZE (on target - use dError for damping)
        3 1 4 1 1; % ZE,NB -> Accel_Soft (prevent undershoot)
        3 2 4 1 1; % ZE,NS -> Accel_Soft (prevent undershoot)
        3 3 3 1 1; % ZE,ZE -> Maintain
        3 4 2 1 1; % ZE,PS -> Brake_Soft (prevent overshoot)
        3 5 2 1 1; % ZE,PB -> Brake_Soft (prevent overshoot)
        
        % Error=PS (slightly too slow)
        4 1 5 1 1; % PS,NB -> Accel_Hard (getting much worse)
        4 2 5 1 1; % PS,NS -> Accel_Hard (getting worse)
        4 3 4 1 1; % PS,ZE -> Accel_Soft
        4 4 4 1 1; % PS,PS -> Accel_Soft
        4 5 3 1 1; % PS,PB -> Maintain (correcting well)
        
        % Error=PB (too slow - need to accelerate)
        5 1 5 1 1; % PB,NB -> Accel_Hard (getting much worse)
        5 2 5 1 1; % PB,NS -> Accel_Hard (getting worse)
        5 3 5 1 1; % PB,ZE -> Accel_Hard
        5 4 5 1 1; % PB,PS -> Accel_Hard
        5 5 4 1 1; % PB,PB -> Accel_Soft (already correcting fast)
    ];
    fis = addRule(fis, ruleList);
    
    % Save FIS to workspace for Simulink to access
    assignin('base', 'fis_driver', fis);

    % --- FLC Implementation with Saturation ---
    
    % Saturation Blocks for Inputs (Protection)
    add_block('simulink/Commonly Used Blocks/Saturation', [sub_name '/Sat_Error'], 'Position', [180, 50, 210, 80]);
    set_param([sub_name '/Sat_Error'], 'UpperLimit', '20', 'LowerLimit', '-20');
    
    add_block('simulink/Commonly Used Blocks/Saturation', [sub_name '/Sat_dError'], 'Position', [180, 90, 210, 120]);
    set_param([sub_name '/Sat_dError'], 'UpperLimit', '100', 'LowerLimit', '-100');

    % Mux Inputs for FLC
    add_block('simulink/Commonly Used Blocks/Mux', [sub_name '/Mux'], 'Position', [230, 60, 235, 110], 'Inputs', '2');
    
    % Fuzzy Logic Controller Block (Double space in name is required)
    add_block('fuzblock/Fuzzy Logic  Controller', [sub_name '/FLC'], 'Position', [260, 60, 360, 110]);
    set_param([sub_name '/FLC'], 'FIS', 'fis_driver');
    
    % Connect Inputs -> Saturation -> Mux -> FLC
    add_line(sub_name, 'Error_Calc/1', 'Sat_Error/1');
    add_line(sub_name, 'dError_Calc/1', 'Sat_dError/1');
    add_line(sub_name, 'Sat_Error/1', 'Mux/1');
    add_line(sub_name, 'Sat_dError/1', 'Mux/2');
    add_line(sub_name, 'Mux/1', 'FLC/1');
    
    % Output Processing (Accel/Brake Split)
    add_block('simulink/Discontinuities/Saturation', [sub_name '/Sat_Accel'], 'Position', [400, 50, 430, 80]);
    set_param([sub_name '/Sat_Accel'], 'LowerLimit', '0', 'UpperLimit', '1');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Negate'], 'Position', [380, 100, 410, 130]);
    set_param([sub_name '/Negate'], 'Gain', '-1');
    
    add_block('simulink/Discontinuities/Saturation', [sub_name '/Sat_Brake'], 'Position', [430, 100, 460, 130]);
    set_param([sub_name '/Sat_Brake'], 'LowerLimit', '0', 'UpperLimit', '1');
    
    add_line(sub_name, 'FLC/1', 'Sat_Accel/1');
    add_line(sub_name, 'FLC/1', 'Negate/1');
    add_line(sub_name, 'Negate/1', 'Sat_Brake/1');
    
    % Outputs
    add_block('simulink/Sinks/Out1', [sub_name '/Accel_Cmd'], 'Position', [500, 60, 530, 80]);
    add_block('simulink/Sinks/Out1', [sub_name '/Brake_Cmd'], 'Position', [500, 110, 530, 130]);
    
    add_line(sub_name, 'Sat_Accel/1', 'Accel_Cmd/1');
    add_line(sub_name, 'Sat_Brake/1', 'Brake_Cmd/1');
end

% --- REUSED SUBSYSTEM BUILDERS (Standard) ---

function build_regen_subsystem(model_name)
    % Regenerative Braking Controller
    sub_name = [model_name '/Regen_Controller'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [200, 200, 300, 300]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    % Inputs
    add_block('simulink/Sources/In1', [sub_name '/Brake_Cmd'], 'Position', [20, 50, 50, 70]);
    add_block('simulink/Sources/In1', [sub_name '/Velocity'], 'Position', [20, 150, 50, 170]);
    add_block('simulink/Sources/In1', [sub_name '/SoC'], 'Position', [20, 250, 50, 270]);
    
    % Constants
    add_block('simulink/Math Operations/Gain', [sub_name '/Calc_F_Demand'], 'Position', [100, 45, 150, 75]);
    set_param([sub_name '/Calc_F_Demand'], 'Gain', 'vehicle.M_vehicle*vehicle.g*0.8'); 
    add_line(sub_name, 'Brake_Cmd/1', 'Calc_F_Demand/1');
    
    % Calculate Max Regen Force Available
    add_block('simulink/Sources/Constant', [sub_name '/P_regen_max'], 'Position', [100, 120, 140, 140]);
    set_param([sub_name '/P_regen_max'], 'Value', 'vehicle.regen.max_power');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Div_P_by_V'], 'Position', [200, 130, 230, 160]);
    set_param([sub_name '/Div_P_by_V'], 'Inputs', '*/');
    
    % Protect against divide by zero
    add_block('simulink/Discontinuities/Saturation', [sub_name '/Min_Vel'], 'Position', [100, 150, 130, 170]);
    set_param([sub_name '/Min_Vel'], 'LowerLimit', '0.1', 'UpperLimit', 'inf');
    add_line(sub_name, 'Velocity/1', 'Min_Vel/1');
    
    add_line(sub_name, 'P_regen_max/1', 'Div_P_by_V/1');
    add_line(sub_name, 'Min_Vel/1', 'Div_P_by_V/2');
    
    % Speed Fade
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/Speed_Factor'], 'Position', [200, 180, 250, 210]);
    set_param([sub_name '/Speed_Factor'], 'Table', '[0 0 1 1]', 'BreakpointsForDimension1', '[0 0.3 1.5 100]');
    add_line(sub_name, 'Velocity/1', 'Speed_Factor/1');
    
    % SoC Fade
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/SoC_Factor'], 'Position', [200, 250, 250, 280]);
    set_param([sub_name '/SoC_Factor'], 'Table', '[1 1 0 0]', 'BreakpointsForDimension1', '[0 0.92 0.99 1]');
    add_line(sub_name, 'SoC/1', 'SoC_Factor/1');
    
    % Combine Limits
    add_block('simulink/Math Operations/Product', [sub_name '/Calc_F_Avail'], 'Position', [300, 150, 330, 200]);
    set_param([sub_name '/Calc_F_Avail'], 'Inputs', '3');
    add_line(sub_name, 'Div_P_by_V/1', 'Calc_F_Avail/1');
    add_line(sub_name, 'Speed_Factor/1', 'Calc_F_Avail/2');
    add_line(sub_name, 'SoC_Factor/1', 'Calc_F_Avail/3');
    
    % Logic: Min(F_Demand, F_Avail)
    add_block('simulink/Math Operations/MinMax', [sub_name '/Min_Force'], 'Position', [400, 60, 430, 120]);
    set_param([sub_name '/Min_Force'], 'Function', 'min', 'Inputs', '2');
    add_line(sub_name, 'Calc_F_Demand/1', 'Min_Force/1');
    add_line(sub_name, 'Calc_F_Avail/1', 'Min_Force/2');
    
    % Calculate Friction
    add_block('simulink/Math Operations/Subtract', [sub_name '/Sub_Friction'], 'Position', [500, 50, 530, 80]);
    add_line(sub_name, 'Calc_F_Demand/1', 'Sub_Friction/1');
    add_line(sub_name, 'Min_Force/1', 'Sub_Friction/2');
    
    % Convert F_Regen to Motor Torque
    add_block('simulink/Math Operations/Gain', [sub_name '/Force_to_Torque'], 'Position', [500, 100, 550, 130]);
    set_param([sub_name '/Force_to_Torque'], 'Gain', '-vehicle.r_tire / vehicle.gear_ratio / vehicle.eta_trans');
    add_line(sub_name, 'Min_Force/1', 'Force_to_Torque/1');
    
    % Outputs
    add_block('simulink/Sinks/Out1', [sub_name '/T_regen_cmd'], 'Position', [650, 105, 680, 125]);
    add_block('simulink/Sinks/Out1', [sub_name '/F_friction'], 'Position', [650, 55, 680, 75]);
    
    add_line(sub_name, 'Force_to_Torque/1', 'T_regen_cmd/1');
    add_line(sub_name, 'Sub_Friction/1', 'F_friction/1');
end

function build_motor_subsystem(model_name)
    sub_name = [model_name '/Motor_Drive'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [350, 50, 450, 150]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/T_demand'], 'Position', [20, 50, 50, 70]);
    add_block('simulink/Sources/In1', [sub_name '/Speed_rads'], 'Position', [20, 150, 50, 170]);
    
    add_block('simulink/Math Operations/Gain', [sub_name '/To_RPM'], 'Position', [100, 145, 140, 175]);
    set_param([sub_name '/To_RPM'], 'Gain', '60/(2*pi)');
    add_line(sub_name, 'Speed_rads/1', 'To_RPM/1');
    
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/Max_Torque_Map'], 'Position', [200, 200, 250, 230]);
    set_param([sub_name '/Max_Torque_Map'], 'Table', 'vehicle.motor.max_torque', 'BreakpointsForDimension1', 'vehicle.motor.speed_vec');
    add_line(sub_name, 'To_RPM/1', 'Max_Torque_Map/1');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Neg_Max'], 'Position', [300, 220, 330, 250]);
    set_param([sub_name '/Neg_Max'], 'Gain', '-1');
    add_line(sub_name, 'Max_Torque_Map/1', 'Neg_Max/1');
    
    add_block('simulink/Discontinuities/Saturation Dynamic', [sub_name '/Torque_Limit'], 'Position', [400, 50, 430, 90]);
    add_line(sub_name, 'T_demand/1', 'Torque_Limit/1');
    add_line(sub_name, 'Max_Torque_Map/1', 'Torque_Limit/2');
    add_line(sub_name, 'Neg_Max/1', 'Torque_Limit/3');
    
    add_block('simulink/Math Operations/Abs', [sub_name '/Abs_Torque'], 'Position', [450, 120, 480, 150]);
    add_line(sub_name, 'Torque_Limit/1', 'Abs_Torque/1');
    
    add_block('simulink/Lookup Tables/2-D Lookup Table', [sub_name '/Eff_Map'], 'Position', [550, 130, 600, 170]);
    set_param([sub_name '/Eff_Map'], 'Table', 'vehicle.motor.eff_map', ...
        'BreakpointsForDimension1', 'vehicle.motor.speed_vec', ...
        'BreakpointsForDimension2', 'vehicle.motor.torque_vec');
    add_line(sub_name, 'To_RPM/1', 'Eff_Map/1');
    add_line(sub_name, 'Abs_Torque/1', 'Eff_Map/2');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Calc_P_mech'], 'Position', [500, 40, 530, 70]);
    add_line(sub_name, 'Torque_Limit/1', 'Calc_P_mech/1');
    add_line(sub_name, 'Speed_rads/1', 'Calc_P_mech/2');
    
    add_block('simulink/Sources/Constant', [sub_name '/Inv_Eff'], 'Position', [550, 200, 580, 220]);
    set_param([sub_name '/Inv_Eff'], 'Value', '0.95');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Total_Eff'], 'Position', [650, 150, 680, 180]);
    add_line(sub_name, 'Eff_Map/1', 'Total_Eff/1');
    add_line(sub_name, 'Inv_Eff/1', 'Total_Eff/2');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/Is_Motoring'], 'Position', [600, 10, 630, 40]);
    set_param([sub_name '/Is_Motoring'], 'Operator', '>');
    
    add_block('simulink/Sources/Constant', [sub_name '/Zero_Ref_M'], 'Position', [550, 80, 570, 100]);
    set_param([sub_name '/Zero_Ref_M'], 'Value', '0');
    
    add_line(sub_name, 'Calc_P_mech/1', 'Is_Motoring/1');
    add_line(sub_name, 'Zero_Ref_M/1', 'Is_Motoring/2');
    
    add_block('simulink/Signal Routing/Switch', [sub_name '/Power_Switch'], 'Position', [750, 50, 780, 90]);
    
    add_block('simulink/Math Operations/Product', [sub_name '/Div_Eff'], 'Position', [700, 40, 730, 70]);
    set_param([sub_name '/Div_Eff'], 'Inputs', '*/');
    add_line(sub_name, 'Calc_P_mech/1', 'Div_Eff/1');
    add_line(sub_name, 'Total_Eff/1', 'Div_Eff/2');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Mult_Eff'], 'Position', [700, 90, 730, 120]);
    add_line(sub_name, 'Calc_P_mech/1', 'Mult_Eff/1');
    add_line(sub_name, 'Total_Eff/1', 'Mult_Eff/2');
    
    add_line(sub_name, 'Is_Motoring/1', 'Power_Switch/2');
    add_line(sub_name, 'Div_Eff/1', 'Power_Switch/1');
    add_line(sub_name, 'Mult_Eff/1', 'Power_Switch/3');
    
    add_block('simulink/Sinks/Out1', [sub_name '/T_actual'], 'Position', [850, 20, 880, 40]);
    add_block('simulink/Sinks/Out1', [sub_name '/P_elec'], 'Position', [850, 70, 880, 90]);
    
    add_line(sub_name, 'Torque_Limit/1', 'T_actual/1');
    add_line(sub_name, 'Power_Switch/1', 'P_elec/1');
end

function build_transmission_subsystem(model_name)
    sub_name = [model_name '/Transmission'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [500, 50, 600, 150]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/Motor_Torque'], 'Position', [20, 50, 50, 70]);
    add_block('simulink/Sources/In1', [sub_name '/Veh_Speed'], 'Position', [20, 150, 50, 170]);
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Calc_Force'], 'Position', [150, 45, 250, 75]);
    set_param([sub_name '/Calc_Force'], 'Gain', 'vehicle.gear_ratio * vehicle.eta_trans / vehicle.r_tire');
    add_line(sub_name, 'Motor_Torque/1', 'Calc_Force/1');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Calc_Motor_Speed'], 'Position', [150, 145, 250, 175]);
    set_param([sub_name '/Calc_Motor_Speed'], 'Gain', 'vehicle.gear_ratio / vehicle.r_tire');
    add_line(sub_name, 'Veh_Speed/1', 'Calc_Motor_Speed/1');
    
    add_block('simulink/Sinks/Out1', [sub_name '/Tractive_Force'], 'Position', [350, 55, 380, 75]);
    add_block('simulink/Sinks/Out1', [sub_name '/Motor_Speed'], 'Position', [350, 155, 380, 175]);
    
    add_line(sub_name, 'Calc_Force/1', 'Tractive_Force/1');
    add_line(sub_name, 'Calc_Motor_Speed/1', 'Motor_Speed/1');
end

function build_dynamics_subsystem(model_name)
    sub_name = [model_name '/Vehicle_Dynamics'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [650, 50, 750, 150]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/F_Tractive'], 'Position', [20, 50, 50, 70]);
    add_block('simulink/Sources/In1', [sub_name '/F_Brake'], 'Position', [20, 100, 50, 120]);
    
    add_block('simulink/Math Operations/Product', [sub_name '/V_Squared'], 'Position', [200, 200, 230, 230]);
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Aero_Coeff'], 'Position', [250, 200, 350, 230]);
    set_param([sub_name '/Aero_Coeff'], 'Gain', '0.5 * vehicle.rho_air * vehicle.Cd * vehicle.A_frontal');
    add_line(sub_name, 'V_Squared/1', 'Aero_Coeff/1');
    
    add_block('simulink/Sources/Constant', [sub_name '/Roll_Res'], 'Position', [250, 250, 300, 280]);
    set_param([sub_name '/Roll_Res'], 'Value', 'vehicle.C_RR * vehicle.M_vehicle * vehicle.g');
    
    add_block('simulink/Math Operations/Add', [sub_name '/Sum_Forces'], 'Position', [400, 50, 430, 150]);
    set_param([sub_name '/Sum_Forces'], 'Inputs', '+---');
    
    add_line(sub_name, 'F_Tractive/1', 'Sum_Forces/1');
    add_line(sub_name, 'F_Brake/1', 'Sum_Forces/2');
    add_line(sub_name, 'Aero_Coeff/1', 'Sum_Forces/3');
    add_line(sub_name, 'Roll_Res/1', 'Sum_Forces/4');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Inv_Mass'], 'Position', [450, 85, 500, 115]);
    set_param([sub_name '/Inv_Mass'], 'Gain', '1/(vehicle.M_vehicle * 1.05)'); 
    add_line(sub_name, 'Sum_Forces/1', 'Inv_Mass/1');
    
    add_block('simulink/Continuous/Integrator', [sub_name '/Integrator_Vel'], 'Position', [550, 85, 580, 115]);
    set_param([sub_name '/Integrator_Vel'], 'InitialCondition', '0');
    add_line(sub_name, 'Inv_Mass/1', 'Integrator_Vel/1');
    
    add_line(sub_name, 'Integrator_Vel/1', 'V_Squared/1');
    add_line(sub_name, 'Integrator_Vel/1', 'V_Squared/2');
    
    add_block('simulink/Continuous/Integrator', [sub_name '/Integrator_Dist'], 'Position', [650, 150, 680, 180]);
    set_param([sub_name '/Integrator_Dist'], 'InitialCondition', '0');
    add_line(sub_name, 'Integrator_Vel/1', 'Integrator_Dist/1');
    
    add_block('simulink/Sinks/Out1', [sub_name '/Velocity'], 'Position', [750, 90, 780, 110]);
    add_block('simulink/Sinks/Out1', [sub_name '/Distance'], 'Position', [750, 160, 780, 180]);
    
    add_line(sub_name, 'Integrator_Vel/1', 'Velocity/1');
    add_line(sub_name, 'Integrator_Dist/1', 'Distance/1');
end

function build_battery_subsystem(model_name)
    sub_name = [model_name '/Battery_Pack'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [350, 200, 450, 300]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/P_elec'], 'Position', [20, 50, 50, 70]);
    add_block('simulink/Sources/In1', [sub_name '/P_aux'], 'Position', [20, 100, 50, 120]);
    
    add_block('simulink/Math Operations/Add', [sub_name '/Sum_Power'], 'Position', [100, 60, 130, 90]);
    add_line(sub_name, 'P_elec/1', 'Sum_Power/1');
    add_line(sub_name, 'P_aux/1', 'Sum_Power/2');
    
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/V_oc_Map'], 'Position', [200, 150, 250, 180]);
    set_param([sub_name '/V_oc_Map'], 'Table', 'vehicle.battery.ocv_vec', 'BreakpointsForDimension1', 'vehicle.battery.soc_vec');
    
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/R0_Map'], 'Position', [200, 200, 250, 230]);
    set_param([sub_name '/R0_Map'], 'Table', 'vehicle.battery.r0_vec', 'BreakpointsForDimension1', 'vehicle.battery.soc_vec');
    
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/R1_Map'], 'Position', [200, 250, 250, 280]);
    set_param([sub_name '/R1_Map'], 'Table', 'vehicle.battery.r1_vec', 'BreakpointsForDimension1', 'vehicle.battery.soc_vec');
    
    add_block('simulink/Lookup Tables/1-D Lookup Table', [sub_name '/C1_Map'], 'Position', [200, 300, 250, 330]);
    set_param([sub_name '/C1_Map'], 'Table', 'vehicle.battery.c1_vec', 'BreakpointsForDimension1', 'vehicle.battery.soc_vec');
    
    add_block('simulink/Signal Routing/Mux', [sub_name '/Mux_Current'], 'Position', [350, 80, 355, 200]);
    set_param([sub_name '/Mux_Current'], 'Inputs', '4', 'DisplayOption', 'bar');
    
    add_line(sub_name, 'Sum_Power/1', 'Mux_Current/1');
    add_line(sub_name, 'V_oc_Map/1', 'Mux_Current/2');
    add_line(sub_name, 'R0_Map/1', 'Mux_Current/3');
    
    add_block('simulink/User-Defined Functions/Fcn', [sub_name '/Calc_Current'], 'Position', [400, 120, 500, 160]);
    expr = '((u(2)-u(4)) - sqrt((u(2)-u(4))^2 - 4*u(3)*u(1))) / (2*u(3))';
    set_param([sub_name '/Calc_Current'], 'Expr', expr);
    
    add_line(sub_name, 'Mux_Current/1', 'Calc_Current/1');
    
    add_block('simulink/Signal Routing/Mux', [sub_name '/Mux_dV1'], 'Position', [550, 250, 555, 350]);
    set_param([sub_name '/Mux_dV1'], 'Inputs', '4', 'DisplayOption', 'bar');
    
    add_line(sub_name, 'Calc_Current/1', 'Mux_dV1/1');
    add_line(sub_name, 'R1_Map/1', 'Mux_dV1/3');
    add_line(sub_name, 'C1_Map/1', 'Mux_dV1/4');
    
    add_block('simulink/User-Defined Functions/Fcn', [sub_name '/Calc_dV1'], 'Position', [600, 290, 660, 310]);
    set_param([sub_name '/Calc_dV1'], 'Expr', '-u(2)/(u(3)*u(4)) + u(1)/u(4)');
    
    add_line(sub_name, 'Mux_dV1/1', 'Calc_dV1/1');
    
    add_block('simulink/Continuous/Integrator', [sub_name '/Integrator_V1'], 'Position', [680, 290, 710, 320]);
    set_param([sub_name '/Integrator_V1'], 'InitialCondition', '0');
    add_line(sub_name, 'Calc_dV1/1', 'Integrator_V1/1');
    
    add_line(sub_name, 'Integrator_V1/1', 'Mux_Current/4');
    add_line(sub_name, 'Integrator_V1/1', 'Mux_dV1/2');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Calc_dSoC'], 'Position', [550, 80, 600, 110]);
    set_param([sub_name '/Calc_dSoC'], 'Gain', '-1/(vehicle.battery.capacity_Ah * vehicle.battery.n_parallel * 3600)');
    add_line(sub_name, 'Calc_Current/1', 'Calc_dSoC/1');
    
    add_block('simulink/Continuous/Integrator', [sub_name '/Integrator_SoC'], 'Position', [650, 80, 680, 110]);
    set_param([sub_name '/Integrator_SoC'], 'InitialCondition', '0.95');
    add_line(sub_name, 'Calc_dSoC/1', 'Integrator_SoC/1');
    
    add_line(sub_name, 'Integrator_SoC/1', 'V_oc_Map/1');
    add_line(sub_name, 'Integrator_SoC/1', 'R0_Map/1');
    add_line(sub_name, 'Integrator_SoC/1', 'R1_Map/1');
    add_line(sub_name, 'Integrator_SoC/1', 'C1_Map/1');
    
    add_block('simulink/Math Operations/Add', [sub_name '/Calc_V_term'], 'Position', [800, 150, 830, 200]);
    set_param([sub_name '/Calc_V_term'], 'Inputs', '+--');
    
    add_line(sub_name, 'V_oc_Map/1', 'Calc_V_term/1');
    
    add_block('simulink/Math Operations/Product', [sub_name '/I_R0'], 'Position', [750, 180, 780, 210]);
    add_line(sub_name, 'Calc_Current/1', 'I_R0/1');
    add_line(sub_name, 'R0_Map/1', 'I_R0/2');
    add_line(sub_name, 'I_R0/1', 'Calc_V_term/2');
    
    add_line(sub_name, 'Integrator_V1/1', 'Calc_V_term/3');
    
    add_block('simulink/Sinks/Out1', [sub_name '/SoC'], 'Position', [900, 90, 930, 110]);
    add_block('simulink/Sinks/Out1', [sub_name '/V_term'], 'Position', [900, 170, 930, 190]);
    add_block('simulink/Sinks/Out1', [sub_name '/I_batt'], 'Position', [900, 50, 930, 70]);
    
    add_line(sub_name, 'Integrator_SoC/1', 'SoC/1');
    add_line(sub_name, 'Calc_V_term/1', 'V_term/1');
    add_line(sub_name, 'Calc_Current/1', 'I_batt/1');
end

function build_aux_subsystem(model_name)
    sub_name = [model_name '/Auxiliaries'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [200, 200, 300, 300]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/Constant', [sub_name '/Aux_Load'], 'Position', [50, 50, 100, 80]);
    set_param([sub_name '/Aux_Load'], 'Value', 'vehicle.aux_power');
    
    add_block('simulink/Sinks/Out1', [sub_name '/P_aux'], 'Position', [200, 55, 230, 75]);
    add_line(sub_name, 'Aux_Load/1', 'P_aux/1');
end

% --- HESS SUBSYSTEMS ---

function build_supercapacitor_subsystem(model_name)
    % Supercapacitor Subsystem (RC Model)
    sub_name = [model_name '/Supercapacitor'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [100, 350, 250, 450]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/P_demand'], 'Position', [50, 100, 80, 120]);
    
    add_block('simulink/Sources/Constant', [sub_name '/C_pack'], 'Position', [100, 200, 140, 220]);
    set_param([sub_name '/C_pack'], 'Value', 'vehicle.supercap.pack_capacitance');
    
    add_block('simulink/Sources/Constant', [sub_name '/R_esr'], 'Position', [100, 250, 140, 270]);
    set_param([sub_name '/R_esr'], 'Value', 'vehicle.supercap.pack_esr');
    
    add_block('simulink/Sources/Constant', [sub_name '/V_max'], 'Position', [100, 300, 140, 320]);
    set_param([sub_name '/V_max'], 'Value', 'vehicle.supercap.pack_voltage');
    
    add_block('simulink/Continuous/Integrator', [sub_name '/Energy_J'], 'Position', [400, 100, 430, 130]);
    set_param([sub_name '/Energy_J'], 'InitialCondition', ...
        '0.5 * vehicle.supercap.pack_capacitance * (vehicle.supercap.pack_voltage * vehicle.supercap.initial_soc)^2');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Negate'], 'Position', [200, 100, 230, 130]);
    set_param([sub_name '/Negate'], 'Gain', '-1');
    add_line(sub_name, 'P_demand/1', 'Negate/1');
    add_line(sub_name, 'Negate/1', 'Energy_J/1');
    
    add_block('simulink/Math Operations/Gain', [sub_name '/Two_over_C'], 'Position', [500, 100, 550, 130]);
    set_param([sub_name '/Two_over_C'], 'Gain', '2 / vehicle.supercap.pack_capacitance');
    add_line(sub_name, 'Energy_J/1', 'Two_over_C/1');
    
    add_block('simulink/Math Operations/Sqrt', [sub_name '/Sqrt'], 'Position', [600, 100, 630, 130]);
    add_line(sub_name, 'Two_over_C/1', 'Sqrt/1');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Calc_I'], 'Position', [700, 150, 730, 180]);
    set_param([sub_name '/Calc_I'], 'Inputs', '*/');
    add_line(sub_name, 'P_demand/1', 'Calc_I/1');
    add_line(sub_name, 'Sqrt/1', 'Calc_I/2');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Calc_SoC'], 'Position', [700, 250, 730, 280]);
    set_param([sub_name '/Calc_SoC'], 'Inputs', '*/');
    add_line(sub_name, 'Sqrt/1', 'Calc_SoC/1');
    add_line(sub_name, 'V_max/1', 'Calc_SoC/2');
    
    add_block('simulink/Sinks/Out1', [sub_name '/V_term'], 'Position', [800, 100, 830, 120]);
    add_block('simulink/Sinks/Out1', [sub_name '/I_sc'], 'Position', [800, 160, 830, 180]);
    add_block('simulink/Sinks/Out1', [sub_name '/SoC'], 'Position', [800, 260, 830, 280]);
    
    add_line(sub_name, 'Sqrt/1', 'V_term/1');
    add_line(sub_name, 'Calc_I/1', 'I_sc/1');
    add_line(sub_name, 'Calc_SoC/1', 'SoC/1');
end

function build_dcdc_controller_subsystem(model_name)
    % DC-DC Controller Subsystem
    sub_name = [model_name '/DCDC_Controller'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [300, 350, 450, 450]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/P_cmd'], 'Position', [50, 100, 80, 120]);
    
    add_block('simulink/Sources/Constant', [sub_name '/Eff'], 'Position', [100, 200, 140, 220]);
    set_param([sub_name '/Eff'], 'Value', 'vehicle.dcdc.efficiency');
    
    add_block('simulink/Sinks/Out1', [sub_name '/P_batt_side'], 'Position', [600, 100, 630, 120]);
    add_line(sub_name, 'P_cmd/1', 'P_batt_side/1');
    
    add_block('simulink/Signal Routing/Switch', [sub_name '/Eff_Switch'], 'Position', [300, 150, 330, 190]);
    set_param([sub_name '/Eff_Switch'], 'Threshold', '0');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Div_Eff'], 'Position', [200, 130, 230, 160]);
    set_param([sub_name '/Div_Eff'], 'Inputs', '*/');
    add_line(sub_name, 'P_cmd/1', 'Div_Eff/1');
    add_line(sub_name, 'Eff/1', 'Div_Eff/2');
    
    add_block('simulink/Math Operations/Product', [sub_name '/Mult_Eff'], 'Position', [200, 230, 230, 260]);
    add_line(sub_name, 'P_cmd/1', 'Mult_Eff/1');
    add_line(sub_name, 'Eff/1', 'Mult_Eff/2');
    
    add_line(sub_name, 'Div_Eff/1', 'Eff_Switch/1');
    add_line(sub_name, 'P_cmd/1', 'Eff_Switch/2');
    add_line(sub_name, 'Mult_Eff/1', 'Eff_Switch/3');
    
    add_block('simulink/Sinks/Out1', [sub_name '/P_sc_side'], 'Position', [600, 170, 630, 190]);
    add_line(sub_name, 'Eff_Switch/1', 'P_sc_side/1');
end

function build_ems_subsystem(model_name)
    % Energy Management System (EMS)
    sub_name = [model_name '/EMS'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [300, 200, 450, 300]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/P_load'], 'Position', [50, 50, 80, 70]);
    add_block('simulink/Sources/In1', [sub_name '/SC_SoC'], 'Position', [50, 150, 80, 170]);
    add_block('simulink/Sources/In1', [sub_name '/Batt_SoC'], 'Position', [50, 250, 80, 270]);
    
    add_block('simulink/Sources/Constant', [sub_name '/Regen_Thresh'], 'Position', [100, 300, 140, 320]);
    set_param([sub_name '/Regen_Thresh'], 'Value', '-vehicle.ems.regen_power_threshold');
    
    add_block('simulink/Sources/Constant', [sub_name '/Accel_Thresh'], 'Position', [100, 350, 140, 370]);
    set_param([sub_name '/Accel_Thresh'], 'Value', 'vehicle.ems.accel_power_threshold');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/Is_High_Regen'], 'Position', [200, 50, 230, 80]);
    set_param([sub_name '/Is_High_Regen'], 'Operator', '<');
    add_line(sub_name, 'P_load/1', 'Is_High_Regen/1');
    add_line(sub_name, 'Regen_Thresh/1', 'Is_High_Regen/2');
    
    add_block('simulink/Sources/Constant', [sub_name '/SC_Max'], 'Position', [100, 150, 130, 170]);
    set_param([sub_name '/SC_Max'], 'Value', '0.95');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/SC_Has_Room'], 'Position', [200, 140, 230, 170]);
    set_param([sub_name '/SC_Has_Room'], 'Operator', '<');
    add_line(sub_name, 'SC_SoC/1', 'SC_Has_Room/1');
    add_line(sub_name, 'SC_Max/1', 'SC_Has_Room/2');
    
    add_block('simulink/Logic and Bit Operations/Logical Operator', [sub_name '/Regen_Cond'], 'Position', [280, 90, 310, 120]);
    set_param([sub_name '/Regen_Cond'], 'Operator', 'AND');
    add_line(sub_name, 'Is_High_Regen/1', 'Regen_Cond/1');
    add_line(sub_name, 'SC_Has_Room/1', 'Regen_Cond/2');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/Is_High_Accel'], 'Position', [200, 250, 230, 280]);
    set_param([sub_name '/Is_High_Accel'], 'Operator', '>');
    add_line(sub_name, 'P_load/1', 'Is_High_Accel/1');
    add_line(sub_name, 'Accel_Thresh/1', 'Is_High_Accel/2');
    
    add_block('simulink/Sources/Constant', [sub_name '/SC_Min'], 'Position', [100, 200, 130, 220]);
    set_param([sub_name '/SC_Min'], 'Value', '0.20');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/SC_Has_Charge'], 'Position', [200, 300, 230, 330]);
    set_param([sub_name '/SC_Has_Charge'], 'Operator', '>');
    add_line(sub_name, 'SC_SoC/1', 'SC_Has_Charge/1');
    add_line(sub_name, 'SC_Min/1', 'SC_Has_Charge/2');
    
    add_block('simulink/Logic and Bit Operations/Logical Operator', [sub_name '/Accel_Cond'], 'Position', [280, 270, 310, 300]);
    set_param([sub_name '/Accel_Cond'], 'Operator', 'AND');
    add_line(sub_name, 'Is_High_Accel/1', 'Accel_Cond/1');
    add_line(sub_name, 'SC_Has_Charge/1', 'Accel_Cond/2');
    
    add_block('simulink/Sources/Constant', [sub_name '/Zero'], 'Position', [350, 200, 380, 220]);
    set_param([sub_name '/Zero'], 'Value', '0');
    
    add_block('simulink/Math Operations/Add', [sub_name '/Calc_Assist'], 'Position', [350, 350, 380, 380]);
    set_param([sub_name '/Calc_Assist'], 'Inputs', '+-');
    add_line(sub_name, 'P_load/1', 'Calc_Assist/1');
    add_line(sub_name, 'Accel_Thresh/1', 'Calc_Assist/2');
    
    add_block('simulink/Signal Routing/Switch', [sub_name '/Sw_SC_Regen'], 'Position', [500, 150, 530, 200]);
    set_param([sub_name '/Sw_SC_Regen'], 'Threshold', '0.5');
    
    add_block('simulink/Signal Routing/Switch', [sub_name '/Sw_SC_Accel'], 'Position', [420, 250, 450, 300]);
    set_param([sub_name '/Sw_SC_Accel'], 'Threshold', '0.5');
    
    add_line(sub_name, 'Calc_Assist/1', 'Sw_SC_Accel/1');
    add_line(sub_name, 'Accel_Cond/1', 'Sw_SC_Accel/2');
    add_line(sub_name, 'Zero/1', 'Sw_SC_Accel/3');
    
    add_line(sub_name, 'P_load/1', 'Sw_SC_Regen/1');
    add_line(sub_name, 'Regen_Cond/1', 'Sw_SC_Regen/2');
    add_line(sub_name, 'Sw_SC_Accel/1', 'Sw_SC_Regen/3');
    
    add_block('simulink/Math Operations/Add', [sub_name '/Calc_Batt'], 'Position', [600, 70, 630, 100]);
    set_param([sub_name '/Calc_Batt'], 'Inputs', '+-');
    add_line(sub_name, 'P_load/1', 'Calc_Batt/1');
    add_line(sub_name, 'Sw_SC_Regen/1', 'Calc_Batt/2');
    
    add_block('simulink/Sinks/Out1', [sub_name '/P_batt_cmd'], 'Position', [700, 70, 730, 90]);
    add_block('simulink/Sinks/Out1', [sub_name '/P_sc_cmd'], 'Position', [700, 170, 730, 190]);
    
    add_line(sub_name, 'Calc_Batt/1', 'P_batt_cmd/1');
    add_line(sub_name, 'Sw_SC_Regen/1', 'P_sc_cmd/1');
end

function build_bms_subsystem(model_name)
    % Battery Management System (BMS)
    sub_name = [model_name '/BMS'];
    add_block('simulink/Ports & Subsystems/Subsystem', sub_name, 'Position', [500, 350, 600, 450]);
    Simulink.SubSystem.deleteContents(sub_name);
    
    add_block('simulink/Sources/In1', [sub_name '/P_req'], 'Position', [50, 100, 80, 120]);
    add_block('simulink/Sources/In1', [sub_name '/SoC'], 'Position', [50, 200, 80, 220]);
    
    add_block('simulink/Sources/Constant', [sub_name '/SoC_Max'], 'Position', [100, 250, 140, 270]);
    set_param([sub_name '/SoC_Max'], 'Value', 'vehicle.bms.soc_max_charge');
    
    add_block('simulink/Sources/Constant', [sub_name '/Zero'], 'Position', [100, 300, 130, 320]);
    set_param([sub_name '/Zero'], 'Value', '0');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/Is_Charge'], 'Position', [200, 100, 230, 130]);
    set_param([sub_name '/Is_Charge'], 'Operator', '<');
    add_line(sub_name, 'P_req/1', 'Is_Charge/1');
    add_line(sub_name, 'Zero/1', 'Is_Charge/2');
    
    add_block('simulink/Logic and Bit Operations/Relational Operator', [sub_name '/SoC_High'], 'Position', [200, 200, 230, 230]);
    set_param([sub_name '/SoC_High'], 'Operator', '>');
    add_line(sub_name, 'SoC/1', 'SoC_High/1');
    add_line(sub_name, 'SoC_Max/1', 'SoC_High/2');
    
    add_block('simulink/Logic and Bit Operations/Logical Operator', [sub_name '/Block_Charge'], 'Position', [280, 150, 310, 180]);
    set_param([sub_name '/Block_Charge'], 'Operator', 'AND');
    add_line(sub_name, 'Is_Charge/1', 'Block_Charge/1');
    add_line(sub_name, 'SoC_High/1', 'Block_Charge/2');
    
    add_block('simulink/Signal Routing/Switch', [sub_name '/Sw_Out'], 'Position', [350, 100, 380, 150]);
    set_param([sub_name '/Sw_Out'], 'Threshold', '0.5');
    add_line(sub_name, 'Zero/1', 'Sw_Out/1');
    add_line(sub_name, 'Block_Charge/1', 'Sw_Out/2');
    add_line(sub_name, 'P_req/1', 'Sw_Out/3');
    
    add_block('simulink/Sinks/Out1', [sub_name '/P_allowed'], 'Position', [450, 120, 480, 140]);
    add_line(sub_name, 'Sw_Out/1', 'P_allowed/1');
end

function connect_all_subsystems(model_name)
    add_block('simulink/Sources/Clock', [model_name '/Clock'], 'Position', [20, 50, 40, 70]);
    
    add_block('simulink/Lookup Tables/1-D Lookup Table', [model_name '/Drive_Cycle'], 'Position', [80, 40, 130, 80]);
    set_param([model_name '/Drive_Cycle'], 'Table', 'cycle.velocity_ms', 'BreakpointsForDimension1', 'cycle.time');
    add_line(model_name, 'Clock/1', 'Drive_Cycle/1');
    
    add_line(model_name, 'Drive_Cycle/1', 'Driver_Model/1');
    
    add_line(model_name, 'Driver_Model/2', 'Regen_Controller/1');
    
    add_block('simulink/Math Operations/Gain', [model_name '/Accel_Gain'], 'Position', [250, 60, 300, 90]);
    set_param([model_name '/Accel_Gain'], 'Gain', 'vehicle.motor.peak_torque'); 
    add_line(model_name, 'Driver_Model/1', 'Accel_Gain/1');
    
    add_block('simulink/Math Operations/Add', [model_name '/Sum_Torque'], 'Position', [320, 60, 340, 90]);
    add_line(model_name, 'Accel_Gain/1', 'Sum_Torque/1');
    add_line(model_name, 'Regen_Controller/1', 'Sum_Torque/2'); 
    
    add_line(model_name, 'Sum_Torque/1', 'Motor_Drive/1');
    add_line(model_name, 'Transmission/2', 'Motor_Drive/2');
    add_line(model_name, 'Motor_Drive/1', 'Transmission/1');
    add_line(model_name, 'Transmission/1', 'Vehicle_Dynamics/1');
    add_line(model_name, 'Regen_Controller/2', 'Vehicle_Dynamics/2');
    
    add_block('simulink/Signal Routing/Goto', [model_name '/Goto_Vel'], 'Position', [800, 90, 840, 110]);
    set_param([model_name '/Goto_Vel'], 'GotoTag', 'Vel');
    add_line(model_name, 'Vehicle_Dynamics/1', 'Goto_Vel/1');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_Vel_Driver'], 'Position', [20, 100, 60, 120]);
    set_param([model_name '/From_Vel_Driver'], 'GotoTag', 'Vel');
    add_line(model_name, 'From_Vel_Driver/1', 'Driver_Model/2');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_Vel_Regen'], 'Position', [150, 240, 190, 260]);
    set_param([model_name '/From_Vel_Regen'], 'GotoTag', 'Vel');
    add_line(model_name, 'From_Vel_Regen/1', 'Regen_Controller/2');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_Vel_Trans'], 'Position', [450, 100, 490, 120]);
    set_param([model_name '/From_Vel_Trans'], 'GotoTag', 'Vel');
    add_line(model_name, 'From_Vel_Trans/1', 'Transmission/2');
    
    add_block('simulink/Signal Routing/Goto', [model_name '/Goto_SoC'], 'Position', [500, 220, 540, 240]);
    set_param([model_name '/Goto_SoC'], 'GotoTag', 'Batt_SoC');
    add_line(model_name, 'Battery_Pack/1', 'Goto_SoC/1');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_SoC_Regen'], 'Position', [150, 280, 190, 300]);
    set_param([model_name '/From_SoC_Regen'], 'GotoTag', 'Batt_SoC');
    add_line(model_name, 'From_SoC_Regen/1', 'Regen_Controller/3');
    
    % HESS Connections (EMS, BMS, SC, DCDC)
    add_line(model_name, 'Motor_Drive/2', 'EMS/1');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_SC_SoC_EMS'], 'Position', [250, 230, 290, 250]);
    set_param([model_name '/From_SC_SoC_EMS'], 'GotoTag', 'SC_SoC');
    add_line(model_name, 'From_SC_SoC_EMS/1', 'EMS/2');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_Batt_SoC_EMS'], 'Position', [250, 260, 290, 280]);
    set_param([model_name '/From_Batt_SoC_EMS'], 'GotoTag', 'Batt_SoC');
    add_line(model_name, 'From_Batt_SoC_EMS/1', 'EMS/3');
    
    add_line(model_name, 'EMS/1', 'BMS/1');
    add_line(model_name, 'EMS/2', 'DCDC_Controller/1');
    
    add_block('simulink/Signal Routing/From', [model_name '/From_Batt_SoC_BMS'], 'Position', [450, 400, 490, 420]);
    set_param([model_name '/From_Batt_SoC_BMS'], 'GotoTag', 'Batt_SoC');
    add_line(model_name, 'From_Batt_SoC_BMS/1', 'BMS/2');
    
    add_line(model_name, 'BMS/1', 'Battery_Pack/1');
    add_line(model_name, 'DCDC_Controller/2', 'Supercapacitor/1');
    add_line(model_name, 'Auxiliaries/1', 'Battery_Pack/2');
    
    add_block('simulink/Signal Routing/Goto', [model_name '/Goto_SC_SoC'], 'Position', [300, 430, 340, 450]);
    set_param([model_name '/Goto_SC_SoC'], 'GotoTag', 'SC_SoC');
    add_line(model_name, 'Supercapacitor/3', 'Goto_SC_SoC/1');
end

function add_logging(model_name)
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Vel_Ref'], 'Position', [150, 10, 210, 30]);
    set_param([model_name '/Log_Vel_Ref'], 'VariableName', 'vel_ref', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Drive_Cycle/1', 'Log_Vel_Ref/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Vel_Act'], 'Position', [800, 120, 860, 140]);
    set_param([model_name '/Log_Vel_Act'], 'VariableName', 'vel_actual', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Vehicle_Dynamics/1', 'Log_Vel_Act/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_SoC'], 'Position', [500, 250, 560, 270]);
    set_param([model_name '/Log_SoC'], 'VariableName', 'soc', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Battery_Pack/1', 'Log_SoC/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_V_term'], 'Position', [500, 280, 560, 300]);
    set_param([model_name '/Log_V_term'], 'VariableName', 'v_term', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Battery_Pack/2', 'Log_V_term/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_I_batt'], 'Position', [500, 310, 560, 330]);
    set_param([model_name '/Log_I_batt'], 'VariableName', 'i_batt', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Battery_Pack/3', 'Log_I_batt/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_T_motor'], 'Position', [480, 10, 540, 30]);
    set_param([model_name '/Log_T_motor'], 'VariableName', 't_motor', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Motor_Drive/1', 'Log_T_motor/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_P_elec'], 'Position', [480, 160, 540, 180]);
    set_param([model_name '/Log_P_elec'], 'VariableName', 'p_elec', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Motor_Drive/2', 'Log_P_elec/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Dist'], 'Position', [800, 160, 860, 180]);
    set_param([model_name '/Log_Dist'], 'VariableName', 'distance', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Vehicle_Dynamics/2', 'Log_Dist/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Time'], 'Position', [60, 10, 120, 30]);
    set_param([model_name '/Log_Time'], 'VariableName', 'sim_time', 'SaveFormat', 'Array');
    add_line(model_name, 'Clock/1', 'Log_Time/1');
    
    % HESS Logging
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_SC_SoC'], 'Position', [300, 400, 360, 420]);
    set_param([model_name '/Log_SC_SoC'], 'VariableName', 'sc_soc', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Supercapacitor/3', 'Log_SC_SoC/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_I_sc'], 'Position', [300, 370, 360, 390]);
    set_param([model_name '/Log_I_sc'], 'VariableName', 'i_sc', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Supercapacitor/2', 'Log_I_sc/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_V_sc'], 'Position', [300, 340, 360, 360]);
    set_param([model_name '/Log_V_sc'], 'VariableName', 'v_sc', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'Supercapacitor/1', 'Log_V_sc/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_P_batt_cmd'], 'Position', [550, 120, 610, 140]);
    set_param([model_name '/Log_P_batt_cmd'], 'VariableName', 'p_batt_cmd', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'EMS/1', 'Log_P_batt_cmd/1');
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_P_sc_cmd'], 'Position', [550, 170, 610, 190]);
    set_param([model_name '/Log_P_sc_cmd'], 'VariableName', 'p_sc_cmd', 'SaveFormat', 'Structure with Time');
    add_line(model_name, 'EMS/2', 'Log_P_sc_cmd/1');
end
