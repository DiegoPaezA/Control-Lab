% Based on tclab - matlab library
function TC = arduino_lab1(heater)
persistent icount a

if (isempty(icount))
   try
      a = arduino;
      disp(a)
   catch
      warning('Unable to connect, user input required')
      disp('For Windows:')
      disp('  Open device manager, select "Ports (COM & LPT)"')
      disp('  Look for COM port of Arduino such as COM4')
      disp('For MacOS:')
      disp('  Open terminal and type: ls /dev/*.')
      disp('  Search for /dev/tty.usbmodem* or /dev/tty.usbserial*. The port number is *.')
      disp('For Linux')
      disp('  Open terminal and type: ls /dev/tty*')
      disp('  Search for /dev/ttyUSB* or /dev/ttyACM*. The port number is *.')
      disp('')
      com_port = input('Specify port (e.g. COM4 for Windows or /dev/ttyUSB0 for Linux): ','s');
      a = arduino(com_port,'Uno');
      disp(a)
   end
   icount = 0;
end    
    
% voltage read functions
v1 = @() readVoltage(a, 'A1');
v2 = @() readVoltage(a, 'A2');

% temperature calculations as a function of voltage for LM35
TC = @(V) (V)*100.0;          % Celsius


% temperature read functions
T1C = @() TC(v1());
T2C = @() TC(v2());

% LED function (0 <= level <= 1)
led = @(level) writePWMDutyCycle(a,'D3',max(0,min(1,level)));  % ON

% heater output (0 <= heater <= 100)
% limit to 0-1 (0-100%)
h1 = @(level) writePWMDutyCycle(a,'D6',max(0,min(100,level))*1/100);
% limit to 0-1 (0-100%)
%h2 = @(level) writePWMDutyCycle(a,'D9',max(0,min(100,level))*1/100);
    
% increment counter
icount = icount + 1;

% read temperature
TC = T1C();
% write heater
h1(heater);

% indicate high temperature with LED
if TC > 40
    led(1)
else
    led(0)    
end

end