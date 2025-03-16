function CyberDiver_tune_position_loop()

% List all available serial ports
availablePorts = serialportlist("all");

if isempty(availablePorts)
    error('No serial ports found.');
end

% Display available ports
fprintf('Available serial ports:\n');
fprintf('%-5s  %-20s\n', 'Index', 'Port');
fprintf('%s\n', repmat('-', 1, 15)); % Separator line
for i = 1:length(availablePorts)
    fprintf('%-5d  %-20s\n', i, availablePorts{i});
end

% Prompt user to select a port by index
portIndex = input('Select a serial port by entering the corresponding index: ');

% Validate the input
if portIndex < 1 || portIndex > length(availablePorts)
    error('Invalid selection.');
end

serialPort = availablePorts{portIndex};

% Create a serial object
s = serial(serialPort, 'BaudRate', 9600);

% Open the serial port
fopen(s);
pause(0.1); % Allow time for the port to open


try
    % connect to the cyberdiver
    fprintf(s, 'connect\n');
    response = fgets(s);
    if ~startsWith(response, 'ack')
        throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
    end

    fprintf(s, 'powerboard set mode idle\n');
    response = fgets(s);  % ignore the response because the power board could be in either state


    tuning = true;
    while tuning

        % get the gains
        fprintf(s, 'controller config get gains\n');
        response = fgets(s);
        if startsWith(response, 'ack')
            tokens = strsplit(strip(response));
            Kp = str2double(tokens(2));
            Ki = str2double(tokens(3));
            Kd = str2double(tokens(4));
            Kff = str2double(tokens(5));
        else
            throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
        end

        fprintf('Current gains: Kp = %f, Ki = %f, Kd = %f, Kff = %f\n', Kp, Ki, Kd, Kff);

        if input('Enter new gains? [y/n]: ', 's') == 'y'
            Kp = input('Kp: ');
            Ki = input('Ki: ');
            Kd = input('Kd: ');
            Kff = input('Kff: ');


            fprintf(s, sprintf('controller config set gains %f %f %f %f\n', Kp, Ki, Kd, Kff));
            response = fgets(s);
            if ~startsWith(response, 'ack')
                throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
            end


        else
            if input('Perform step response? [y/n]: ', 's') == 'y'

                amplitude = input('Step amplitude (mm): ');
                logging_time = input('Time to log (s): ');

                % configure the logger and controller
                fprintf(s, 'logger stream stop\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'logger set location sdram\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'logger set packetsize 1\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'logger set decimation 0\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'powerboard set mode running\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'controller resettime\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'controller set setpoint 0\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'controller set mode position_control\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'logger start\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'controller set setpoint %f\n', amplitude);
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                pause(logging_time);

                fprintf(s, 'logger stop\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'controller set mode idle\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end

                fprintf(s, 'powerboard set mode idle\n');
                response = fgets(s);
                if ~startsWith(response, 'ack')
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end


                fprintf('Waiting for data...\n')


                % retrieve the data
                fprintf(s, 'logger get available\n');
                response = fgets(s);
                if startsWith(response, 'ack')
                    tokens = strsplit(strip(response));
                    num_packets = str2double(tokens(2));
                else
                    throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                end


                time_s = [];
                current_a = [];
                force_n = [];
                position_mm = [];
                velocity_mmps = [];
                duty_cycle = [];
                accel_x_g = [];
                accel_y_g = [];
                accel_z_g = [];
                setpoint = [];
                mode = [];
                led = [];

                for j = 1:num_packets
                    fprintf(s, 'logger get packet\n');
                    response = fgets(s);

                    if startsWith(response, 'data')
                        % Split the string into tokens
                        tokens = strsplit(strip(response)); % Split by whitespace

                        % Get the number of data points
                        num_data_pts = str2double(tokens{2}); % Second token

                        % Initialize arrays for the current data packet
                        for i = 0:num_data_pts-1
                            % Extract values from the tokens
                            timestamp_token = str2double(tokens{3 + i * 12}); % Time stamp for each data point
                            current_token = str2double(tokens{4 + i * 12});
                            force_token = str2double(tokens{5 + i * 12});
                            position_token = str2double(tokens{6 + i * 12});
                            velocity_token = str2double(tokens{7 + i * 12});
                            duty_token = str2double(tokens{8 + i * 12});
                            accel_x_token = str2double(tokens{9 + i * 12});
                            accel_y_token = str2double(tokens{10 + i * 12});
                            accel_z_token = str2double(tokens{11 + i * 12});
                            setpoint_token = str2double(tokens{12 + i * 12});
                            mode_token = tokens{13 + i * 12}; % Mode (string)
                            led_token = tokens{14 + i * 12}; % LED status

                            % Store values in the corresponding arrays
                            time_s(end + 1) = timestamp_token * 0.000001;
                            current_a(end + 1) = current_token;
                            force_n(end + 1) = force_token;
                            position_mm(end + 1) = position_token;
                            velocity_mmps(end + 1) = velocity_token;
                            duty_cycle(end + 1) = duty_token;
                            accel_x_g(end + 1) = accel_x_token;
                            accel_y_g(end + 1) = accel_y_token;
                            accel_z_g(end + 1) = accel_z_token;
                            setpoint(end + 1) = setpoint_token;
                            mode{end + 1} = mode_token;
                            led{end + 1} = led_token;
                        end
                    else
                        throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                    end
                end


                fprintf('Data received\n')


                % plot the data
                figure
                set(gcf, 'color', 'w')
                subplot(2,1,1)
                hold on
                box on
                title(sprintf('Kp = %f, Ki = %f, Kd = %f, Kff = %f', Kp, Ki, Kd, Kff))
                xlabel('Time (s)')
                ylabel('Displacement (mm)')
                plot(time_s, setpoint, 'k--', 'DisplayName', 'Set point')
                plot(time_s, position_mm, 'b-', 'DisplayName', 'Measured')
                xlim tight
                legend show
                subplot(2,1,2)
                hold on
                box on
                xlabel('Time (s)')
                ylabel('Duty cycle')
                plot(time_s, duty_cycle, 'r-')
                xlim tight


            else
                if input('Save gains to SD card? [y/n]: ', 's') == 'y'
                    fprintf(s, 'config save\n');
                    response = fgets(s);
                    if ~startsWith(response, 'ack')
                        throw(MException('CyberDiver:serialError', 'Unexpected response from serial port.'));
                    end
                end

                % done with program
                tuning = false;
            end
        end
    end


    % Clean up serial
    fprintf(s, 'controller set mode idle\n');
    fprintf(s, 'disconnect\n');
    fclose(s);
    delete(s);
    clear s;


catch ME
    % Clean up
    fprintf(s, 'controller set mode idle\n');
    fprintf(s, 'disconnect\n');
    fclose(s);
    delete(s);
    clear s;
    rethrow(ME);
end

end


