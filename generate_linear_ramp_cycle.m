function cycle = generate_linear_ramp_cycle()
    % generate_linear_ramp_cycle - Generate Linear Ramp driving cycle
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
