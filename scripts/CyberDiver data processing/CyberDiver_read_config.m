function config = CyberDiver_read_config(file)

fileID = fopen(file, 'r');

config = CyberDiver_read_config_from_fp(fileID);

fclose(fileID);

end




