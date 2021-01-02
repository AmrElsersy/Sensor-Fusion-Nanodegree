clear all
clc;


%% Radar Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%speed of light = 3e8

range_res = 1;
max_vel = 100;
vel_res = 3;
max_range = 200;
c = 3e8;

%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
v_0 = 30; % 30 m/s
R = 100; % 100 m 

%% FMCW Waveform Generation

b_sweep = c / (2*range_res);
Tchrip = (2*max_range) / c;
Tchrip = 5.5*Tchrip; % For an FMCW radar system, the sweep time should be at least 5 to 6 
             % times the round trip time.
slope = b_sweep / Tchrip;

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq
                                                         
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each chirp
t=linspace(0,Nd*Tchrip,Nr*Nd); %total time for samples 


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t)); % trip time 


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 
disp(length(t));
for i=1:length(t)         
    
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = R + v_0 * t(i); 
    td(i) = 2*r_t(i) / c;
    
    % Receive and Transmit
    Tx(i) = cos(2*pi*(fc*t(i) + 0.5*slope*t(i)^2));
    Rx(i) = cos(2*pi*(fc*(t(i) - td(i)) + 0.5*slope*(t(i)-td(i))^2 ));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %this mix signal hold the information about both Range(comming from the freq change and Doppler comming from phase change)    
    Mix(i) = Tx(i).*Rx(i);
    
end

fprintf("Ray2");

%% RANGE MEASUREMENT
% Signal lenght
L = Tchrip *  b_sweep;

%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix,[Nr,Nd]);

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft=fft(Mix,Nr);
signal_fft = abs(signal_fft/L); % absolute

signal_fft = signal_fft(1:L/2+1);% half samples


f = b_sweep*(0:(L/2))/L;
figure('Name','FFT')
plot(f,signal_fft)

% range estimation
R = (c*Tchrip*f)/(2*b_sweep);

% Plot range
figure ('Name','Range FFT')
plot(R, signal_fft)
axis ( [0 200 0 0.5] );


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 12;
Td = 6;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 6;
Gd = 3;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 7;


% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

% Then create another vector to hold final signal after thresholding
signal_cfar = zeros(Nr/2,Nd);

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing CFAR
num_cells = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1);

for i=1:(Nr/2-(2*Gr+2*Tr))
    for j=1:(Nd-(2*Gd+2*Td))
        s1 = sum(db2pow(RDM(i:i+2*Tr+2*Gr, j:j+2*Td+2*Gd)),'all');
        s2 = sum(db2pow(RDM(i+Tr:i+Tr+2*Gr, j+Td:j+Td+2*Gd)),'all');    
        noise_level = s1 - s2;
        
%         threshold = avarage noise
        th = noise_level/num_cells;
%         * offset
        th = pow2db(th) + offset;
        th= db2pow(th);
        
        
        signal = db2pow(RDM(i+Tr+Gr, j+Td+Gd));

%         if signal > threshold take it in considration
        if (signal <= th)
            signal = 0;
        else 
            signal = 1;
        end
        
        signal_cfar(i+Tr+Gr,j+Td+Gd) = signal;        
    end
end


figure,surf(doppler_axis,range_axis,signal_cfar);
colorbar;

 
 
