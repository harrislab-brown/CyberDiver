function CyberDiver_write_config_v1(config, file)

desc_str_len = 1024;
experimental_sequence_max_len = 1024;
calib_poly_max_order = 10;
mode_names = {'Idle', 'Current control', 'Force control', 'Position control', 'Simulated structure'};

% file writing

if (config.firmware_version ~= writer_version)
    error('Writer and firmware version do not match.')
end

fileID = fopen(file, 'w');

fwrite(fileID, config.firmware_version, "uint32");
fwrite(fileID, padarray(double(config.experiment_description), [0, desc_str_len - length(config.experiment_description)], 'post'), "uint8");

fwrite(fileID, find(strcmp(config.idle_controller_state.mode, mode_names)) - 1, "uint32");
fwrite(fileID, config.idle_controller_state.setpoint, "float32");
fwrite(fileID, config.idle_controller_state.led_status, "uint16");
fwrite(fileID, config.idle_controller_state.is_logging, "uint16");

fwrite(fileID, find(strcmp(config.armed_controller_state.mode, mode_names)) - 1, "uint32");
fwrite(fileID, config.armed_controller_state.setpoint, "float32");
fwrite(fileID, config.armed_controller_state.led_status, "uint16");
fwrite(fileID, config.armed_controller_state.is_logging, "uint16");

fwrite(fileID, config.staging_time_us, "uint32");
fwrite(fileID, config.running_time_us, "uint32");
fwrite(fileID, config.trigger_accel_threshold, "float32");



for i = 1:experimental_sequence_max_len
    fwrite(fileID, find(strcmp(config.controller_config.experimental_sequence.sequence.Mode(i), mode_names)) - 1, "uint32");
    fwrite(fileID, config.controller_config.experimental_sequence.sequence.Setpoint(i), "float32");
    fwrite(fileID, config.controller_config.experimental_sequence.sequence.LED(i), "uint16");
    fwrite(fileID, config.controller_config.experimental_sequence.sequence.Logging(i), "uint16");
end

fwrite(fileID, config.controller_config.experimental_sequence.len, "uint32");

for i = 1:experimental_sequence_max_len
    fwrite(fileID, config.controller_config.experimental_sequence.sequence.("Time (us)")(i), "uint32");
end

fwrite(fileID, config.controller_config.experimental_sequence.is_looping, "uint32");



fwrite(fileID, config.controller_config.pos_pid_config.Kp, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.Ki, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.Kd, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.Kff, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.tau, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.lim_min, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.lim_max, "float32");
fwrite(fileID, config.controller_config.pos_pid_config.T, "float32");

fwrite(fileID, config.controller_config.poly_coil.order, "uint32");
fwrite(fileID, config.controller_config.poly_coil.coeffs, "float32");
fwrite(fileID, config.controller_config.poly_coil.input_min, "float32");
fwrite(fileID, config.controller_config.poly_coil.input_max, "float32");

fwrite(fileID, config.controller_config.poly_passive.order, "uint32");
fwrite(fileID, config.controller_config.poly_passive.coeffs, "float32");
fwrite(fileID, config.controller_config.poly_passive.input_min, "float32");
fwrite(fileID, config.controller_config.poly_passive.input_max, "float32");

fwrite(fileID, config.controller_config.poly_structure.order, "uint32");
fwrite(fileID, config.controller_config.poly_structure.coeffs, "float32");
fwrite(fileID, config.controller_config.poly_structure.input_min, "float32");
fwrite(fileID, config.controller_config.poly_structure.input_max, "float32");

fwrite(fileID, config.controller_config.neutral_position_offset_mm, "float32");


fwrite(fileID, config.logger_config.decimation, "uint32");


fwrite(fileID, config.power_board_config.Kp, "float32");
fwrite(fileID, config.power_board_config.Ki, "float32");
fwrite(fileID, config.power_board_config.Kd, "float32");
fwrite(fileID, config.power_board_config.Kff, "float32");
fwrite(fileID, config.power_board_config.tau, "float32");
fwrite(fileID, config.power_board_config.lim_min, "float32");
fwrite(fileID, config.power_board_config.lim_max, "float32");
fwrite(fileID, config.power_board_config.T, "float32");

fwrite(fileID, config.power_board_config.Kff_vel, "float32");

fwrite(fileID, config.accelerometer_config.g_range, "uint32");

fclose(fileID);
end