function tranzmit(angles, baudrate, port)
%   Summary of this function goes here
%   Detailed explanation goes here

micros = round(angles*(100/9)+500, 0) % Rounded to the nearest integer

% Converting to string for transmission
microsstr = string(micros)
% Implementing the custom format
microsstr = join(microsstr, '_')
microsstr = microsstr+"_e"

output = baudrate
output = port
s = serialport(port,baudrate);
writeline(s,microsstr)

end