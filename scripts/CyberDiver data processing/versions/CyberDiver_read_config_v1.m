function config = CyberDiver_read_config_v1(fileID)

desc_str_len = 1024;
experimental_sequence_max_len = 1024;
calib_poly_max_order = 10;
mode_names = {'Idle', 'Current control', 'Force control', 'Position control', 'Simulated structure'};

% parsing

config = struct;
config.firmware_version = 1;

config.experiment_description = extractBefore(char(fread(fileID, [1, desc_str_len], "char")), char(0));

config.idle_controller_state = struct;
config.idle_controller_state.mode = mode_names{fread(fileID, 1, "uint32") + 1};
config.idle_controller_state.setpoint = fread(fileID, 1, "float32");
config.idle_controller_state.led_status = fread(fileID, 1, "uint16");
config.idle_controller_state.is_logging = fread(fileID, 1, "uint16");

config.armed_controller_state = struct;
config.armed_controller_state.mode = mode_names{fread(fileID, 1, "uint32") + 1};
config.armed_controller_state.setpoint = fread(fileID, 1, "float32");
config.armed_controller_state.led_status = fread(fileID, 1, "uint16");
config.armed_controller_state.is_logging = fread(fileID, 1, "uint16");

config.staging_time_us = fread(fileID, 1, "uint32");
config.running_time_us = fread(fileID, 1, "uint32");
config.trigger_accel_threshold = fread(fileID, 1, "float32");


config.controller_config = struct;
config.controller_config.experimental_sequence = struct;
config.controller_config.experimental_sequence.sequence = table('Size', ...
    [experimental_sequence_max_len, 5], 'VariableTypes', {'uint32', 'string', 'double', 'double', 'double'}, ...
    'VariableNames', {'Time (us)', 'Mode', 'Setpoint', 'LED', 'Logging'});


for i = 1:experimental_sequence_max_len
    config.controller_config.experimental_sequence.sequence.Mode(i) = mode_names(fread(fileID, 1, "uint32") + 1);
    config.controller_config.experimental_sequence.sequence.Setpoint(i) = fread(fileID, 1, "float32");
    config.controller_config.experimental_sequence.sequence.LED(i) = fread(fileID, 1, "uint16");
    config.controller_config.experimental_sequence.sequence.Logging(i) = fread(fileID, 1, "uint16");
end

config.controller_config.experimental_sequence.len = fread(fileID, 1, "uint32");

for i = 1:experimental_sequence_max_len
    config.controller_config.experimental_sequence.sequence.("Time (us)")(i) = fread(fileID, 1, "uint32");
end

config.controller_config.experimental_sequence.is_looping = fread(fileID, 1, "uint32");


config.controller_config.pos_pid_config = struct;
config.controller_config.pos_pid_config.Kp = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.Ki = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.Kd = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.Kff = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.tau = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.lim_min = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.lim_max = fread(fileID, 1, "float32");
config.controller_config.pos_pid_config.T = fread(fileID, 1, "float32");

config.controller_config.poly_coil = struct;
config.controller_config.poly_coil.order = fread(fileID, 1, "uint32");
config.controller_config.poly_coil.coeffs = fread(fileID, [1, calib_poly_max_order + 1], "float32");
config.controller_config.poly_coil.input_min = fread(fileID, 1, "float32");
config.controller_config.poly_coil.input_max = fread(fileID, 1, "float32");

config.controller_config.poly_passive = struct;
config.controller_config.poly_passive.order = fread(fileID, 1, "uint32");
config.controller_config.poly_passive.coeffs = fread(fileID, [1, calib_poly_max_order + 1], "float32");
config.controller_config.poly_passive.input_min = fread(fileID, 1, "float32");
config.controller_config.poly_passive.input_max = fread(fileID, 1, "float32");

config.controller_config.poly_structure = struct;
config.controller_config.poly_structure.order = fread(fileID, 1, "uint32");
config.controller_config.poly_structure.coeffs = fread(fileID, [1, calib_poly_max_order + 1], "float32");
config.controller_config.poly_structure.input_min = fread(fileID, 1, "float32");
config.controller_config.poly_structure.input_max = fread(fileID, 1, "float32");


config.controller_config.neutral_position_offset_mm = fread(fileID, 1, "float32");


config.logger_config.decimation = fread(fileID, 1, "uint32");


config.power_board_config = struct;
config.power_board_config.Kp = fread(fileID, 1, "float32");
config.power_board_config.Ki = fread(fileID, 1, "float32");
config.power_board_config.Kd = fread(fileID, 1, "float32");
config.power_board_config.Kff = fread(fileID, 1, "float32");
config.power_board_config.tau = fread(fileID, 1, "float32");
config.power_board_config.lim_min = fread(fileID, 1, "float32");
config.power_board_config.lim_max = fread(fileID, 1, "float32");
config.power_board_config.T = fread(fileID, 1, "float32");

config.power_board_config.Kff_vel = fread(fileID, 1, "float32");


config.accelerometer_config = struct;
config.accelerometer_config.g_range = fread(fileID, 1, "uint32");

end
