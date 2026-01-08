function cycle = generate_midc_cycle()
    % generate_midc_cycle - Generate Modified Indian Driving Cycle (MIDC)
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
