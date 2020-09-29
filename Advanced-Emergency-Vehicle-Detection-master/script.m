DATA = importdata('data.txt');
dat1 = DATA(:, 1)-2.4e4;
dat2 = DATA(:, 2)-2.4e4;

% dat1 = dat1(800:1900);
% dat2 = dat2(800:1900);

t = 1:length(dat1);

d = designfilt('lowpassiir', 'FilterOrder', 8, ...
    'PassbandFrequency', 15e3, 'PassbandRipple', 0.2,...
    'SampleRate', Fs);
% dat1 = filter(d, dat1);
% dat2 = filter(d, dat2);

clf, close all;
figure(1)
plot(t, dat1), hold on
plot(t, dat2)
title('ADC sampled signals');

figure(2)
Fs = 5.67E5;
N = length(dat1);
F0 = Fs/N;
f = [-N/2*F0:F0:(N/2-1)*F0];
plot(f, abs(fftshift(fft(dat1))/N)), hold on
plot(f, abs(fftshift(fft(dat2))/N))

scale = 1/3000;
t = 1:2*length(dat1)-1;
y = xcorr(dat1*scale, dat2*scale);
figure(3)
plot(t, y);
title('Correlation of ADC sampled signals');

[m, i] = max(y);
deltat = 1.7644e-6*(i-length(dat1))
thetaPredict = 180/pi*asin(deltat*343/0.1)

