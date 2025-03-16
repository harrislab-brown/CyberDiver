function CyberDiver_write_config_v2(config, file)


desc_str_len = 1024;
experimental_sequence_max_len = 1024;
calib_poly_max_order = 100;
mode_names = {'idle', 'current_control', 'force_control', 'position_control', 'simulated_structure'};
logger_loc_names = {'mcu', 'sdram'};

% file writing

fileID = fopen(file, 'w');

fwrite(fileID, config.firmware_version, "uint32");
fwrite(fileID, padarray(double(config.experiment_description), [0, desc_str_len - length(config.experiment_description)], 'post'), "uint8");


mode = find(strcmp(config.idle_controller_state.mode, mode_names)) - 1;
status = bitor(mode, bitshift(config.idle_controller_state.led, 31)); 
fwrite(fileID, status, "uint32");
fwrite(fileID, config.idle_controller_state.setpoint, "float32");


mode = find(strcmp(config.armed_controller_state.mode, mode_names)) - 1;
status = bitor(mode, bitshift(config.armed_controller_state.led, 31)); 
fwrite(fileID, status, "uint32");
fwrite(fileID, config.armed_controller_state.setpoint, "float32");


fwrite(fileID, config.staging_time_us, "uint32");
fwrite(fileID, config.running_time_us, "uint32");
fwrite(fileID, config.trigger_accel_threshold, "float32");


fwrite(fileID, config.encoder_config.neutral_position_offset_mm, "float32");


for i = 1:experimental_sequence_max_len
    mode = find(strcmp(config.controller_config.experimental_sequence.sequence.mode(i), mode_names)) - 1;
    status = bitor(mode, bitshift(config.controller_config.experimental_sequence.sequence.led(i), 31)); 
    fwrite(fileID, status, "uint32");
    fwrite(fileID, config.controller_config.experimental_sequence.sequence.setpoint(i), "float32");
end

fwrite(fileID, config.controller_config.experimental_sequence.len, "uint32");

for i = 1:experimental_sequence_max_len
    fwrite(fileID, config.controller_config.experimental_sequence.sequence.("time (us)")(i), "uint32");
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
fwrite(fileID, config.controller_config.poly_coil.fast_compute, "uint32");

fwrite(fileID, config.controller_config.poly_passive.order, "uint32");
fwrite(fileID, config.controller_config.poly_passive.coeffs, "float32");
fwrite(fileID, config.controller_config.poly_passive.input_min, "float32");
fwrite(fileID, config.controller_config.poly_passive.input_max, "float32");
fwrite(fileID, config.controller_config.poly_passive.fast_compute, "uint32");

fwrite(fileID, config.controller_config.poly_structure.order, "uint32");
fwrite(fileID, config.controller_config.poly_structure.coeffs, "float32");
fwrite(fileID, config.controller_config.poly_structure.input_min, "float32");
fwrite(fileID, config.controller_config.poly_structure.input_max, "float32");
fwrite(fileID, config.controller_config.poly_structure.fast_compute, "uint32");


fwrite(fileID, config.controller_config.damping_npmmps, "float32");



fwrite(fileID, config.logger_config.decimation, "uint32");
fwrite(fileID, find(strcmp(config.logger_config.location, logger_loc_names)) - 1, "uint32");
fwrite(fileID, config.logger_config.packet_size, "uint32");



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
fwrite(fileID, config.accelerometer_config.lpf, "uint32");
fwrite(fileID, config.accelerometer_config.usr_offset_x, "float32");
fwrite(fileID, config.accelerometer_config.usr_offset_y, "float32");
fwrite(fileID, config.accelerometer_config.usr_offset_z, "float32");



fclose(fileID);


end