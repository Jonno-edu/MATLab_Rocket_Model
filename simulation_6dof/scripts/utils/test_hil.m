% test_hil_binary.m
% Test binary communication with Pico HIL

fprintf('=== PICO BINARY COMMUNICATION TEST ===\n\n');

port_name = '/dev/tty.usbmodem21301';
fprintf('Connecting to %s...\n', port_name);

try
    % Open port - NO terminator for binary mode
    port = serialport(port_name, 115200);
    port.Timeout = 2;
    
    % Clear text data from heartbeat
    pause(0.5);
    flush(port);
    
    fprintf('✓ Connected in binary mode\n\n');
    
    % Send 10 floats (your HIL protocol expects this)
        fprintf('Sending 10 floats to Pico...\n');
    test_input = single([
        0.5,   % current_time (NEW!)
        1.0,   % qw (quaternion)
        0.0,   % qx
        0.0,   % qy
        0.0,   % qz
        0.1,   % p (body rates)
        0.2,   % q
        0.3,   % r
        0.05,  % ref_pitch
        0.0,   % ref_yaw
        0.0    % roll_cmd
    ]);
        
    write(port, test_input, "single");
    fprintf('  Sent: quat=[%.2f %.2f %.2f %.2f], rates=[%.2f %.2f %.2f], cmd=[%.2f %.2f %.2f]\n', ...
        test_input(1), test_input(2), test_input(3), test_input(4), ...
        test_input(5), test_input(6), test_input(7), ...
        test_input(8), test_input(9), test_input(10));
    
    % Wait for response (3 floats = 12 bytes expected)
    fprintf('\nWaiting for 3 floats (12 bytes) response...\n');
    
    max_wait = 2;  % 2 second timeout
    tic;
    while toc < max_wait && port.NumBytesAvailable < 12
        pause(0.01);
    end
    
    if port.NumBytesAvailable >= 12
        output = read(port, 3, "single");
        fprintf('✓ Received output: [%.4f, %.4f, %.4f]\n', output(1), output(2), output(3));
        fprintf('  Y_nozzle = %.4f rad\n', output(1));
        fprintf('  Z_nozzle = %.4f rad\n', output(2));
        fprintf('  X_roll   = %.4f rad\n', output(3));
        
        if all(output == 0)
            fprintf('\n⚠ WARNING: All outputs are zero!\n');
            fprintf('   This means either:\n');
            fprintf('   1. Controller gains are zero\n');
            fprintf('   2. Controller not initialized properly\n');
            fprintf('   3. Multi-rate timing issue\n');
        end
    else
        fprintf('⚠ Timeout! Only received %d bytes\n', port.NumBytesAvailable);
        if port.NumBytesAvailable > 0
            raw = read(port, port.NumBytesAvailable, "uint8");
            fprintf('   Raw data: ');
            fprintf('%02X ', raw);
            fprintf('\n');
        end
    end
    
    clear port;
    fprintf('\n=== TEST COMPLETE ===\n');
    
catch ME
    fprintf('\n✗ ERROR: %s\n', ME.message);
    if exist('port', 'var')
        clear port;
    end
end
