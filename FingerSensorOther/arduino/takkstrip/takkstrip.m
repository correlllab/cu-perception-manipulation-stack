%/**************************************************************************/

%@file takktile_matlab.m
%@author Yaroslav Tenzer
%@license BSD

%Matlab example to capture data from the Arduino which is connected to the TakkTile Strip sensor

%This is a library for the TakkTile Strip sensor
%----> http://www.takktile.com/product:takkstrip

%@section HISTORY

%v1.0 - First release by Yaroslav Tenzer

%@section NOTES
% inspired partially by the code of Erin, the RobotGrrl 
% from http://robotgrrl.com/blog/2010/01/15/arduino-to-matlab-read-in-sensor-data/
%/**************************************************************************/

clear all; close all;

% s = serial('COM5'); % for windows
s = serial('/dev/ttyACM2'); % for ubuntu

set(s,'Baudrate',115200);
set(s,'DataBits', 8);
set(s,'StopBits', 1);
fopen(s)
s.ReadAsyncMode = 'continuous';

% Start asynchronous reading
readasync(s);

%%

try
    
for i=1:10000
        tline1 = fscanf(s, '%s')
        if (size(tline1,1) < 2)
            continue
        end
        ss=tline1(2:end-1); % remove the outside brackets

% ------
%        a=strread(ss,'%s','delimiter', ',');
% TODO: it seems that the code below works faster than the previous line ... to check
        a={};
        n_start=1;
        n_stop=0;

        for n_stop=1:length(ss)
            if (ss(n_stop)==',')
                a=[a;ss(n_start:(n_stop-1))];
                n_start=n_stop+1;
            end
        end
        if (n_stop>n_start)
            a=[a;ss(n_start:(n_stop))];
        end
% --------
        
        tmpStr=char(a(1));
        num=str2num(tmpStr(2:end-1));

        % TODO: the plotting works, but slows down the whole system ... 
%        plot(i,num);
%        hold on;
%        drawnow;
%        pause(0.1);  

end
%% ----

catch err
%        rethrow(err);
    err
end

%% --
stopasync(s);
fclose(s)
