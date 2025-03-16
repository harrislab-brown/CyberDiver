function config = CyberDiver_read_config_from_fp(fileID)
% here fp is the pointer to an open file rather than a file name string

% parses a CyberDiver configuration file which is a binary file containing
% the contents of the configuration struct in the embedded code
% returns a struct with all of the configuration parameters

firmware_version = fread(fileID, 1, "uint32");


addpath("versions\")  % access the version specific readers and writers in a sub directory
switch firmware_version
    case 1
        config = CyberDiver_read_config_v1(fileID);
    case 2
        config = CyberDiver_read_config_v2(fileID);
    otherwise
        error('No configuration reader available for this firmware version.')
end

end