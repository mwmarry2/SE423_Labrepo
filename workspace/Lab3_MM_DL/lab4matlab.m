t = 0:.001:1.023;
%X = sin(2*pi*60*t');
%X = sin(2*pi*60*t')+sin(2*pi*29*t')
%X = sin(2*pi*250*t')+sin(2*pi*300*t')
%X = sin(2*pi*250*t')+sin(2*pi*300*t') + 1.5;
Y = fft(X,1024);
Pyy = Y.*conj(Y)/1024;
f = 1000/1024*(0:511);
figure(1)
plot(f,Pyy(1:512))
title('Power spectral density')
xlabel('Frequency (Hz)')
figure(2)
plot(t,X)