%% AUTHOR: OLUWASEGUN SOMEFUN: <oasomefun@futa.edu.ng> (c.) 2020
% Please, Retain this Copyright Information in any Customization, Modification or
% Redistribution
%% Customize as it fits your purpose
% Live Serial DAQ Plotting script for Arduino-Matlab Interfacing
%% VERSION 1.0 -> 01.26.2020.
%
% Visualizes real-time logged data from Arduino® in MATLAB®.
%
% Collect communicated data from serial to a "streamData" object.
% Plots the contents of "streamData" in real-time, and 
% Stores "streamData" to the MATLAB workspace for later preprocessing and analysis
%% MOTIVATION:
%
% DAQ: Frustration with Available Scripts and Tools for Arduino-Matlab
% I wanted it to be Simple, and to a large extent Generalistic to the
% amount of logged Data size
% Also Readable for customization.
% I wanted to
% Input-Output Identification of the dynamics of a physical process model
% Real time logging and visualization of controller performance.
%% HELP
% 1. Make sure your Serial Output from Arduino is in a comma separated format
%    for a certain number of N data variables you want to log.
% Serial.print( (String) var_1 + "," + var_2 + "," + ... + "," + var_N);
% 2. Also Make Sure there is only one Serial command like this in the
% running *.ino or *. cpp program
%
% You might need to change the plotting configuration to change your
% preferred style
% This, although, should work well for most.
%
% Customizable Lines or Sections are started 
% with a comment in the form: % * .....
% e.g:-> % * sampling time (from running microcontroller program)
% 3. (OPTIONAL) Make sure, for safekeeping, 
% to rename the "streamData" object in the
% workspace after each run
%% House Keeping
clc;
close all;
clear *;
clear streamData; % clear stored data to workspace for future analysis
safebool = 0;
instrreset;
%
%% Initializations
% Serial
ThisPort = '/dev/tty.usbmodem14101'; % * The Port to which the Arduino or MCU Board is Connected
ThisBaudRate = 115200; % * Safe baud rate
if ~isempty(instrfind) % closes the port if it was open
    fclose(instrfind);
    delete(instrfind);
end
arduinoSerial = serial(ThisPort, 'BaudRate', ThisBaudRate, 'FlowControl', 'hardware');
%% Connect the serial port to Arduino
arduinoSerial.InputBufferSize = 512; % read only one byte every time
arduinoSerial.Terminator = 'CR/LF';
fopen(arduinoSerial);
% Timing
Tf = 50;
Ts = 0.01; % * sampling time (from running microcontroller program)
n = ((Tf - 0)/Ts) + 1;
t = 0:Ts:10;
% current sample
id = 1;
ID = zeros(n,1);
% Figure
HFig = figure;
ax1 = subplot(1,1,1);
grid (ax1, "on");
% Start timer
tic;
t(id) = 0; % toc;
% Read Data communicated from Serial
datastring = fscanf(arduinoSerial);
datastring = regexp(datastring, '\,*', 'split');
datanumeric = str2double(datastring);
[rowd, cold] = size(datanumeric);
% Data
datastring = "";
datanumeric = 0;
streamData = zeros(id,cold);
plotData = 0;
a = tic; % start timer
% Sample Evolution (Discrete Timing)
while(id < n)
    
    % Read Data communicated from Serial
    b = toc(a);    % start timer
    datastring = fscanf(arduinoSerial);
    if b > (0) % check timer
        
        ID(id) = id; % store sample count
        
        % Ensure sampling time
        if id > 1
            t(id) = t(id-1) + Ts; % toc;
            %Tdiff = t(id) - t(id-1);
            %while Tdiff < Ts
            %    t(id) = toc;
            %    Tdiff = t(id) - t(id-1);
            %end
        end
        
        % Split String Vector to access each Numeric Data Sent
        % space -> \s*
        % comma -> \,
        datastring = regexp(datastring, '\,*', 'split');
        datanumeric = str2double(datastring);
        [rowd, cold] = size(datanumeric);
        for idx = 1:cold
            streamData(id,idx) = datanumeric(idx);
        end
        
        
        % * Visualize Streamed Data up to current sample
        plotTime = t(1:id);
        plotData= streamData(1:id, :);
        
        % * Plot and Animate Data Stream : 
        % YOU SHOULD CHANGE THIS TO FIT YOUR PECUILAR DESIRE(S)
        dataColour = lines; % use matlab's color array function
        for ij= 1:cold
            line(ax1, plotTime,plotData(:,ij),'Color',dataColour(ij,:),'LineWidth',1);
            hold (ax1, "on")
        end
        hold on;
        grid on;
        drawnow limitrate; % * update screen
        a = tic; % reset timer after update
        id = id + 1;
    end
end
drawnow; % draw final frame
%% Close serial connection
if (safebool ~= 1)
    fclose(arduinoSerial);
    delete(arduinoSerial);
    clear arduinoSerial;
end