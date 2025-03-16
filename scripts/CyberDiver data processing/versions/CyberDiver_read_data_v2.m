function data = CyberDiver_read_data_v2(fileID)

types = {'uint32', 'float32', 'float32', 'float32', 'float32', 'float32', ...
    'float32', 'float32', 'float32', 'float32', 'uint32'};

sizes = 4 * ones(1, 11);

raw = cell(1, numel(types));

pos = ftell(fileID);  % get the starting position of the file pointer

for i = 1:numel(types)
    fseek(fileID, sum(sizes(1:i-1)) + pos, 'bof'); 
    raw{i} = fread(fileID, Inf, ['*' types{i}], sum(sizes) - sizes(i)); 
end

status = raw{11};
raw{11} = raw{10};
raw{12} = bitshift(uint32(status), -31);  % led 
raw{10} = bitand(uint32(status), bitcmp(bitshift(uint32(1), 31)));  % mode

mode_names = {'idle', 'current_control', 'force_control', 'position_control', 'simulated_structure'};

raw{10} = string(mode_names(raw{10} + 1))';

l = length(raw{1});
c = cell(l, 12);

table_types = {'uint32', 'double', 'double', 'double', 'double', ...
    'double', 'double', 'double', 'double', 'string', 'double', 'uint32'};

table_names = {'time_us', 'current_A', 'force_N', 'position_mm', 'velocity_mmps', ...
    'duty_cycle', 'accel_x_g', 'accel_y_g', 'accel_z_g', 'mode', 'setpoint', 'led'};

data = table('Size', ...
    [length(raw{1}), 12], 'VariableTypes', table_types, ...
    'VariableNames', table_names);

for i = 1:12
    data.(i) = raw{i};
end

end