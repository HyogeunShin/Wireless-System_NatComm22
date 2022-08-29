%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA %

clear
clc
figure(1); cla reset

% load TXT file
fid = fopen('outfile.txt');

tic

global Xsize;
Xsize=180;% X axis range [sec]             
global Ymax;
Ymax=2;% Y axis plus peak [mV]%1
global Ymin;
Ymin=-2;% Y axis plus peak [mV]%-1
global FrameRate;
FrameRate=5;% UpdatedFrame/sec              
global NextFrameRate;
NextFrameRate=0.001;% NewPage/sec                 
global SamplingFrequency;
SamplingFrequency=8000;% Samples/sec
global Timelap;
Timelap=Xsize/2; %sec                        
global StartFrom;
StartFrom=0; %sec
global position;
position=0;

BAR_THREASHOLD=0;%off
INTER_SPIKE_PLOT=0;%off

CHANNEL=16;

Ch_No = 16; %Channel number
uint32 Start_time=0; %Time at starting point
uint32 Current_time=0; % Current time


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open File, Pre-Allocation and Initialization %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Data separation by packet size
%Packet length = 200 bytes (100 data x 2 byte resolution)
fscanfData = fscanf(fid, '%d', [1 inf]);
Ch08 = fscanfData(1,:);

%Close file
fclose(fid);

%Used for initializing first iteration of data compilation loop below
%Creates a temporary data related to a packet information
if Ch08(1)>=0
    Header1=Ch08(1);
else
    Header1=Ch08(1)+65536; 
end
if Ch08(2)>=0
    Header2=Ch08(2);
else
    Header2=Ch08(2)+65536;
end
Header1_1=(Header1-mod(Header1,256))/256;
Header1_2=mod(Header1,256);
Header2_1=(Header2-mod(Header2,256))/256;
Header2_2=mod(Header2,256);
%Summary
%Header1_1 = LSB0-3, Ch info
%Header1_2 = LSB4-11
%Header2_1 = LSB12-19
%Header2_2 = LSB20-27
Ch_No=mod(Header1_1,16)+1; %Channel information
if(CHANNEL>2)
    if(Ch_No<=CHANNEL)
        Ch_No=mod((Ch_No+CHANNEL-3),CHANNEL)+1;
    end
end
Start_time = (Header1_1-mod(Header1_1,16))/16;
Start_time = Start_time + 16*Header1_2;
Start_time = Start_time + (2^12)*Header2_1;
Start_time = Start_time + (2^20)*Header2_2;
Current_time=Start_time; %Time information
data_size = size(Ch08,2);
Temp_time=zeros(1,48/2);
Temp_data=zeros(1,48/2);
for i=0:48/2-1
    Temp_time(i+1)=i./SamplingFrequency;
    Temp_data(i+1)=Ch08(i+3).*0.195./1000;
end

hold off

% Initializes variables used for determining time
hold on;
Before_time=0;
Inter_time=zeros(1,8000);
Packets=0;
Past_spike=1;

% Pre-allocates arrays for faster processing
wf_pre = zeros((data_size/(52/2)-1), 24);

ch1_time_point = wf_pre; ch1_data_point = wf_pre;
ch2_time_point = wf_pre; ch2_data_point = wf_pre;
ch3_time_point = wf_pre; ch3_data_point = wf_pre;
ch4_time_point = wf_pre; ch4_data_point = wf_pre;
ch5_time_point = wf_pre; ch5_data_point = wf_pre;
ch6_time_point = wf_pre; ch6_data_point = wf_pre;
ch7_time_point = wf_pre; ch7_data_point = wf_pre;
ch8_time_point = wf_pre; ch8_data_point = wf_pre;
ch9_time_point = wf_pre; ch9_data_point = wf_pre;
ch10_data_point = wf_pre; ch10_time_point = wf_pre;
ch11_data_point = wf_pre; ch11_time_point = wf_pre;
ch12_data_point = wf_pre; ch12_time_point = wf_pre;
ch13_data_point = wf_pre; ch13_time_point = wf_pre;
ch14_data_point = wf_pre; ch14_time_point = wf_pre;
ch15_data_point = wf_pre; ch15_time_point = wf_pre;
ch16_data_point = wf_pre; ch16_time_point = wf_pre;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read data and compile into variables         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compiles time and voltage data into pre-allocated arrays
for i=1:(data_size/(52/2)-1)

    if ( (Ch_No == 1 ) )
        ch1_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch1_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 2 ) )
        ch2_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch2_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 3 ) )
        ch3_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch3_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 4 ) )
        ch4_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch4_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 5 ) )
        ch5_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch5_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 6 ) )
        ch6_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch6_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 7 ) )
        ch7_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch7_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 8 ) )
        ch8_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch8_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    
    if ( (Ch_No == 9 ) )
        ch9_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch9_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 10 ) )
        ch10_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch10_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 11 ) )
        ch11_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch11_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 12 ) )
        ch12_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch12_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 13 ) )
        ch13_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch13_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 14 ) )
        ch14_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch14_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 15 ) )
        ch15_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch15_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    if ( (Ch_No == 16 ) )
        ch16_time_point(i, :) = Temp_time + (Current_time-Start_time)/SamplingFrequency;
        ch16_data_point(i, :) = Ch08( 3+i*52/2 : 3+i*52/2+23 ).*0.195./1000;   
    end
    
    % Determines current channel
        Ch_No=mod(Header1_1,16)+1;
        
    if(CHANNEL>2)
        if(Ch_No<=CHANNEL)
            Ch_No=mod((Ch_No+CHANNEL-3),CHANNEL)+1;
        end
    end
    
    if Ch08(1+i*52/2)>=0
        Header1=Ch08(1+i*52/2);
    else
        Header1=Ch08(1+i*52/2)+65536;
    end
    if Ch08(2+i*52/2)>=0
        Header2=Ch08(2+i*52/2);
    else
        Header2=Ch08(2+i*52/2)+65536;
    end
    Header1_1=(Header1-mod(Header1,256))/256;
    Header1_2=mod(Header1,256);
    Header2_1=(Header2-mod(Header2,256))/256;
    Header2_2=mod(Header2,256);
    Current_time = (Header1_1-mod(Header1_1,16))/16 + 16*Header1_2 + (2^12)*Header2_1 + (2^20)*Header2_2;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Raster Plotting                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Assigns time data to a cell array for simplicity
time = {ch1_time_point, ch2_time_point, ch3_time_point, ch4_time_point ...
    ch5_time_point, ch6_time_point, ch7_time_point, ch8_time_point ...
    ch9_time_point, ch10_time_point, ch11_time_point, ch12_time_point ...
    ch13_time_point, ch14_time_point, ch15_time_point, ch16_time_point};

data = {ch1_data_point, ch2_data_point, ch3_data_point, ch4_data_point ...
    ch5_data_point, ch6_data_point, ch7_data_point, ch8_data_point ...
    ch9_data_point, ch10_data_point, ch11_data_point, ch12_data_point ...
    ch13_data_point, ch14_data_point, ch15_data_point, ch16_data_point};

save('time_points.mat', 'time');
save('data_points.mat', 'data');

xMax = max(max(time{1}));
for i = 2:length(time)
    if (max(max(time{i})) > xMax) && (max(max(time{i})) < 10^4)
        xMax = max(max(time{i}));
    end
end
xMax = xMax.*1.1;

% Plotting raster plot and determining total firing rate
hold on
netFiringRates = zeros(1, CHANNEL);
for i = 1:length(time)
    Temp_time = time{i};
    TS = Temp_time( (Temp_time(:, 1) > 0 ) );
    scatter( TS, zeros(1, length(TS))+i , '.');
    firingRate = length(TS) ./ max(TS);
    if ~isempty(firingRate)
        netFiringRates(i) = firingRate;
    end
    disp(['Ch ' num2str(i) ' total firing rate: ' num2str(firingRate)]);
end

% Graph scale - change ylim depending on time
ylim([0, CHANNEL + 1]);
%xlim([0, max(max( time ))]);
set(gca, 'YDir','reverse');

% CCCCCCCCCCCCCCCCCCCCC %

x=[];
y=[];

for i = 1:length(time)
    Temp_time = time{i};
    Temp_data = data{i};
    ind = find(Temp_time(:, 1) > 0); 
    x = zeros(1, length(ind).*24);
    y = zeros(1, length(ind).*24);
    for i2 = 1:length(ind)
        x( (24*i2 - 23):(24*i2) ) = Temp_time(ind(i2), :);
        y( (24*i2 - 23):(24*i2) ) = Temp_data(ind(i2), :); 
    end
    data2.x = x;
    data2.y = y;
    save(['ch' num2str(i) '.mat'], 'data2');
    clear data2;
end
xlim([0, xMax]);
disp('Saved time and voltage data for channels');

title('Raster Plot');

toc


%%

%-------------------------------------%
% ** CAN ONLY RUN AFTER SECTION: A ** %
%-------------------------------------%

% DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD %
% Transient and Merged (NO FOURIER)  %
% DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD %

% ** Set current folder as directory with ch1.mat-ch16.mat ** %

tic

disp(' ** Set current folder as directory with ch1.mat-ch16.mat ** ');

f2 = figure(2); cla reset;

% f4 = figure('units','normalized','outerposition',[0 0 1 1]);

% Change j for number of channels to plot - CHANNEL = 16


for j = 1
    
    load(['ch' num2str(j) '.mat']);
    
    Temp_time = data2.x;
    Temp_data = data2.y;
    
    hold on
    for i = 0:24:length(Temp_time)-24
        x = Temp_time( (i+1):(i+24) );
        y = Temp_data( (i+1):(i+24) );
        plot(x, y, 'b');
    end
    
    disp(['Channel ' num2str(j) ' plotted']);
    hold off
    
end


disp('Plotted');


f3 = figure(3); cla reset;
hold on
for i = 0:24:length(Temp_time)-24
    x = Temp_time( (i+1):(i+24) ) - Temp_time(i+1);
    y = Temp_data( (i+1):(i+24) );
    if max(max(abs(y))) < 2
        plot(x, y, 'b');
    end
end
hold off

toc


%%
% EEEEEEEEEEEEEEEEEEEEEEEEE %%%%%%%%%%%%%
% Fourier transform noise filtering %
% EEEEEEEEEEEEEEEEEEEEEEEEE %%%%%%%%%%%%%

clc

tic
figure(4); cla reset; hold on;

load('time_points.mat');
load('data_points.mat');

% j = 1:length(time); CHANNEL SELECTION
j = 1:16;
spikeInds = [];

for j = j
    
    temp_time = time{j};
    temp_data = data{j};
    
    % find index with spikes
    ind = find( temp_time(:, 1) > 0);

    for i = 1:length(ind)

        start = temp_time( ind(i), 1 );
        x = temp_time( ind(i), : );
        y = temp_data( ind(i), : );

        % For fourier transform
        fs = 8000;
        N = length(y);
        X_mags = abs(fft(y));
        bin_vals = 0:N-1;
        fax_Hz = bin_vals*fs/N;
        N_2 = ceil(N/2);

        % find index where magnitude is maximum
        fMaxInd = find(X_mags(1:N_2) == max(X_mags(1:N_2)));
        freq = fax_Hz(1:N_2);
        maxFreq = freq(fMaxInd(1));
        maxFreqSize = length(maxFreq( (maxFreq > 500) & (maxFreq < 2000) ) );

        % plot only if maximum is between 0.5 kHz and 2 kHz
        if (maxFreq > 500) && (maxFreq < 2000)
            spikeInds = [spikeInds; ind(i)];
        end

    end
    
    scatter(temp_time(spikeInds), j.*ones(1, length(spikeInds)), '.');
    
    f_x = zeros(1, length(spikeInds).*24);
    f_y = f_x;
    for i = 1:length(spikeInds)
        f_x( (24*i)-23:(24*i) ) = temp_time(spikeInds(i), :);
        f_y( (24*i)-23:(24*i) ) = temp_data(spikeInds(i), :);
    end
    
    % Saves channel time and data under variable: dataFourier %
    % Stored in file: "f_chx.mat" in current directory
    dataFourier.x = f_x;
    dataFourier.y = f_y;
    save(['f_ch' num2str(j) '.mat'], 'dataFourier');  
    
end
set(gca, 'YDir','reverse');
% Graph scale - change ylim depending on time
ylim([0, 17]);
title('Raster Plot (Fourier Transform) ');
%xlim([0, max(max( time ))]);
hold off

toc

%%

%-------------------------------------%
% ** CAN ONLY RUN AFTER SECTION: E ** %
%-------------------------------------%

%%%FFFFFFFFFFFFFFFFFFFFF %%%%%%%%%%%%%%%%%%%%%%%%
%Fourier Transient Plot & Fourier Merged Plot %
%%%%FFFFFFFFFFFFFFFFFFFFF %%%%%%%%%%%%%%%%%%%%%%%%

tic
j = 1; % CHANNEL SELECTION

figure(5); hold on;
for j = j
    
    load(['f_ch' num2str(j) '.mat']);
    x = dataFourier.x;
    y = dataFourier.y;
    
   for i = 0:24:length(x)-24      
       plot(x(i+1:i+24), y(i+1:i+24), 'r'); 
    end
    
end
hold off

figure(6); hold on
for j = j
    
    load(['f_ch' num2str(j) '.mat']);
    x = dataFourier.x;
    y = dataFourier.y;
    
    for i = 0:24:length(x)-24
       plot(x(i+1:i+24) - x(i+1), y(i+1:i+24), 'r'); 
    end
    
end
hold off

toc




