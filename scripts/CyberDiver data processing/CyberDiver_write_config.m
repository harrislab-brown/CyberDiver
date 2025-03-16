function CyberDiver_write_config(config, file)

% converts the configuration struct to binary that the embedded system can
% understand and writes it to a file

% parameters

writer_version = config.firmware_version;  % this is compared against the firmware version

addpath("versions\")  % access the version specific readers and writers in a sub directory
switch writer_version
    case 1
        CyberDiver_write_config_v1(config, file)
    case 2
        CyberDiver_write_config_v2(config, file)
    otherwise
        error('No writer available for this firmware version.')
end

end