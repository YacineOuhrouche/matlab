clc; clear; close all;

%% Parameters
fs = 44100;            % Sampling frequency
duration = 5;          % Signal length (seconds)

% Random input signal (simulate music/noise mix)
audio = randn(1, fs*duration);
audio = audio ./ max(abs(audio)); % normalize

%% Define 5-band frequency ranges (Hz)
bands = [60  250;    % Bass
         250  1000;  % Low-Mid
         1000 4000;  % Mid
         4000 8000;  % High-Mid
         8000 16000];% Treble

% Initial gains (0 dB = 1.0 scale)
gains = ones(1, size(bands,1));

%% Design bandpass filters (IIR Butterworth)
filters = cell(size(bands,1),1);
for k = 1:size(bands,1)
    Wn = bands(k,:)/(fs/2); % Normalize by Nyquist
    [b,a] = butter(4, Wn, 'bandpass');
    filters{k} = dfilt.df2t(b,a);
end

%% Create figure with sliders
fig = figure('Name','5-Band Random Signal Equalizer','Position',[200 200 600 400]);

sliderLabels = {'Bass (60–250 Hz)','Low-Mid (250–1k Hz)',...
                'Mid (1k–4k Hz)','High-Mid (4k–8k Hz)','Treble (8k–16k Hz)'};

sliders = gobjects(size(bands,1),1);

for k = 1:length(sliders)
    uicontrol('Style','text','Position',[50,350-60*k,120,20],...
              'String',sliderLabels{k});
    sliders(k) = uicontrol('Style','slider','Position',[200,350-60*k,300,20],...
              'Min',-12,'Max',12,'Value',0,...
              'Callback',@(src,~) updateEQ(audio,filters,sliders,fs));
end

%% First run (flat EQ)
updateEQ(audio,filters,sliders,fs);

%% ===== Local function =====
function updateEQ(audio,filters,sliders,fs)
    eqAudio = zeros(size(audio));
    for k = 1:length(filters)
        % Convert dB gain to linear
        gain = 10^(get(sliders(k),'Value')/20);

        % Filter band + apply gain
        bandSignal = filter(filters{k}, audio);
        eqAudio = eqAudio + gain*bandSignal;
    end
    
    % Normalize
    eqAudio = eqAudio ./ max(abs(eqAudio));
    
    % Plot
    subplot(2,1,1);
    pwelch(audio,[],[],[],fs);
    title('Original Random Signal Spectrum');
    
    subplot(2,1,2);
    pwelch(eqAudio,[],[],[],fs);
    title('Equalized Random Signal Spectrum');
end
