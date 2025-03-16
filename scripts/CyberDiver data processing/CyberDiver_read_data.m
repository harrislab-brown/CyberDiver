function [config, data] = CyberDiver_read_data(file)

% read a binary file from the CyberDiver with data from an experiment
% the file starts with the configuration data for this experiment, then has
% the recorded measurements

fileID = fopen(file, 'r');

config = CyberDiver_read_config_from_fp(fileID);

addpath("versions\")
switch config.firmware_version
    case 1
        data = CyberDiver_read_data_v1(fileID);
    case 2
        data = CyberDiver_read_data_v2(fileID);
    otherwise 
        error('No data reader available for this firmware version.')
end

fclose(fileID);

end