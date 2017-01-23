ox = 0.448;
oz = -0.066;
rx = 0.029;
rz = 0.02075;
rubberLength = 0.542;
zmax = 0.3;
r = 0.0439;

% Import the data
importExpData()

% Normalize x and z directions to a world frame situated at the middle of
% the rubber when it is horizontal.
x = ox-rubberLength/2-rx-x;
z = -oz-rz+z;
psi = -psi;

% Find the approximate frequency of oscillation over the last 30 seconds
fsamp = 120;
N = length(t(t>(finalTime - 30)));
fftZ = fft(z(t>(finalTime - 30)));
dF = fsamp/N;
f = 0:dF:fsamp-dF;
fftZmag = abs(fftZ)/N;

freqLowerBound = 0.75;   %Hz
freqUpperBound = 5;   %Hz

% Just a plot of the spectral density
figure(3),clf
plot(f,fftZmag)
xlabel('$f$ [Hz]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('Magnitude of the Fourier Transform', 'Interpreter', 'LaTeX', 'FontSize', 15)
title('Frequency Content of the $z$-motion of the puck', 'Interpreter', 'LaTeX', 'FontSize', 15)

[mag,ind] = max(fftZmag( f>freqLowerBound & f < freqUpperBound ));
ind =  ind + find(f>freqLowerBound,1,'first') - 1;

signalFreq = f(ind);
signalPeriod = 1/signalFreq;


% Take the last ~M oscillations
M = 4;
takeLast = M*signalPeriod-0.05; % seconds
finalTime = t(end)-1;   % discard the last second

tTail = t(t>(finalTime-takeLast));
xTail = x(t>(finalTime-takeLast));
zTail = z(t>(finalTime-takeLast));
psiTail = psi(t>(finalTime-takeLast));

% Find the locations of the peaks
indPeaks = zeros(M+1,1);
indOld = 1;
lb = tTail(indOld)+1/2*signalPeriod;
ub = tTail(indOld)+3/2*signalPeriod;

for i = 1:M+1
    [a,b] = max(zTail(tTail > lb & tTail < ub));
    ind = find(zTail(indOld:end) == a, 1, 'first');
    indPeaks(i) = ind + indOld - 1;
    lb = tTail(indPeaks(i))+1/2*signalPeriod;
    ub = tTail(indPeaks(i))+3/2*signalPeriod;
    indOld = indPeaks(i);
end

% Find the corresponding troughs
indTroughs = zeros(M+1,1);
indOld = 1;
lb = tTail(indOld)+1/4*signalPeriod;
ub = tTail(indOld)+5/4*signalPeriod;

for i = 1:M+1
    [a,b] = min(zTail(tTail > lb & tTail < ub));
    ind = find(zTail(indOld:end) == a, 1, 'first');
    indTroughs(i) = ind + indOld - 1;
    lb = tTail(indTroughs(i))+1/4*signalPeriod;
    ub = tTail(indTroughs(i))+5/4*signalPeriod;
    indOld = indTroughs(i);
end


% Compute the distance of the puck to the paddle
paddleLine = zeros(length(tTail),2,2);
for i = 1:length(tTail)
    paddleLine(i, 1, :) = [-rubberLength/2, 0];
    paddleLine(i, 2, :) = [-rubberLength/2 + rubberLength*cos(psi(i)), rubberLength*sin(psi(i))];
end

distanceToPaddle = zeros(length(tTail),1);
for i = 1:length(tTail)
    x0 = xTail(i);
    z0 = zTail(i);
    x1 = paddleLine(i,1,1);
    z1 = paddleLine(i,1,2);
    x2 = paddleLine(i,2,1);
    z2 = paddleLine(i,2,2);
    
    distanceToPaddle(i) = abs( (z2-z1)*x0 - (x2-x1)*z0 + x2*z1 - z2*x1 ) / rubberLength;
end

% Find when the distance is minimum
indMinDistance = zeros(M+1,1);
indOld = 1;
lb = tTail(indOld)+1/4*signalPeriod;
ub = tTail(indOld)+5/4*signalPeriod;

for i = 1:M+1
    [a,b] = min(distanceToPaddle(tTail > lb & tTail < ub));
    ind = find(distanceToPaddle(indOld:end) == a, 1, 'first');
    indMinDistance(i) = ind + indOld - 1;
    lb = tTail(indMinDistance(i))+1/4*signalPeriod;
    ub = tTail(indMinDistance(i))+5/4*signalPeriod;
    indOld = indTroughs(i);
end


% Plot stuff
XData1 = [-rubberLength/2, -rubberLength/2 + rubberLength*cos(psi(indMinDistance(1)))];
ZData1 = [0, rubberLength*sin(psi(indMinDistance(1)))];
XData1wVia = [XData1(1), xTail(indTroughs(1)), XData1(2)];
ZData1wVia = [ZData1(1), zTail(indTroughs(1)), ZData1(2)];

XData2 = [-rubberLength/2, -rubberLength/2 + rubberLength*cos(psi(indMinDistance(2)))];
ZData2 = [0, rubberLength*sin(psi(indMinDistance(2)))];
XData2wVia = [XData2(1), xTail(indTroughs(2)), XData2(2)];
ZData2wVia = [ZData2(1), zTail(indTroughs(2)), ZData2(2)];

XData3 = [-rubberLength/2, -rubberLength/2 + rubberLength*cos(psi(indMinDistance(3)))];
ZData3 = [0, rubberLength*sin(psi(indMinDistance(3)))];
XData3wVia = [XData3(1), xTail(indTroughs(3)), XData3(2)];
ZData3wVia = [ZData3(1), zTail(indTroughs(3)), ZData3(2)];

XData4 = [-rubberLength/2, -rubberLength/2 + rubberLength*cos(psi(indMinDistance(4)))];
ZData4 = [0, rubberLength*sin(psi(indMinDistance(4)))];
XData4wVia = [XData4(1), xTail(indTroughs(5)), XData4(2)];
ZData4wVia = [ZData4(1), zTail(indTroughs(5)), ZData4(2)];

XData5 = [-rubberLength/2, -rubberLength/2 + rubberLength*cos(psi(indMinDistance(5)))];
ZData5 = [0, rubberLength*sin(psi(indMinDistance(5)))];
XData5wVia = [XData5(1), xTail(indTroughs(4)), XData5(2)];
ZData5wVia = [ZData5(1), zTail(indTroughs(4)), ZData5(2)];


% Circle
theta = linspace(-pi,pi,1001);
shaftX = zeros(length(theta),5);
shaftZ = zeros(length(theta),5);
rMotor = 0.02;
for i = 1:length(theta)
    for j = 1:5
        shaftX(i,j) = -rubberLength/2 + (rMotor-0.005*(j-1))*cos(theta(i));
        shaftZ(i,j) = (rMotor-0.005*(j-1))*sin(theta(i));
    end
end


figure(1), clf
subplot(2,2,1)
plot(tTail, xTail)
axis('tight')
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(2,2,2)
plot(tTail, zTail)
axis('tight')
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(2,2,3)
plot(tTail, psiTail)
axis('tight')
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$\psi$ [rad]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(2,2,4)
plot(tTail, distanceToPaddle)
% plot(xTail, zTail)
% axis([-rubberLength/2, rubberLength/2, min(z), zmax+0.05])
% xlabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
% ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('d(Puck, Paddle)', 'Interpreter', 'LaTeX', 'FontSize', 15)

figure(20), clf
plot(xTail,zTail, 'LineWidth', 2)
axis([-rubberLength/2-0.4*rMotor, rubberLength/2, min(z), zmax+0.05])

line(xTail(1:indTroughs(1)), zTail(1:indTroughs(1)), 'Color', 'r', 'LineWidth', 1.5)
line(xTail(indPeaks(1)), zTail(indPeaks(1)), 'Marker', 'x', 'MarkerSize', 10, 'Color', 'r', 'LineWidth', 1.5)
line(xTail(indTroughs(1):indPeaks(1)), zTail(indTroughs(1):indPeaks(1)), 'Color', 'r', 'LineWidth', 1.5)
line(xTail(indPeaks(2)), zTail(indPeaks(2)), 'Marker', 'x', 'MarkerSize', 10, 'Color', 'm', 'LineWidth', 1.5)
line(xTail(indPeaks(1):indPeaks(2)), zTail(indPeaks(1):indPeaks(2)), 'Color', 'm', 'LineWidth', 1.5)
line(xTail(indPeaks(3)), zTail(indPeaks(3)), 'Marker', 'x', 'MarkerSize', 10, 'Color', 'k', 'LineWidth', 1.5)
line(xTail(indPeaks(2):indPeaks(3)), zTail(indPeaks(2):indPeaks(3)), 'Color', 'k', 'LineWidth', 1.5)
line(xTail(indPeaks(4)), zTail(indPeaks(4)), 'Marker', 'x', 'MarkerSize', 10, 'Color', [0.2, 0.6, 0.5], 'LineWidth', 1.5)
line(xTail(indPeaks(3):indPeaks(4)), zTail(indPeaks(3):indPeaks(4)), 'Color', [0.2, 0.6, 0.5], 'LineWidth', 1.5)
line(xTail(indPeaks(5)), zTail(indPeaks(5)), 'Marker', 'x', 'MarkerSize', 10, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5)


line(XData1,ZData1, 'Color', 'r', 'LineStyle', '-.')
line(XData1wVia,ZData1wVia, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1.5)
line(XData2,ZData2, 'Color', 'm', 'LineStyle', '-.')
line(XData2wVia,ZData2wVia, 'Color', 'm', 'LineStyle', '-', 'LineWidth', 1.5)
line(XData3,ZData3, 'Color', 'k', 'LineStyle', '-.')
line(XData3wVia,ZData3wVia, 'Color', 'k', 'LineStyle', '-', 'LineWidth', 1.5)
line(XData4,ZData4, 'Color', [0, 0.4470, 0.7410], 'LineStyle', '-.')
line(XData4wVia,ZData4wVia, 'Color', [0, 0.4470, 0.7410], 'LineStyle', '-', 'LineWidth', 1.5)
line(XData5,ZData5, 'Color', [0.2, 0.6, 0.5], 'LineStyle', '-.')
line(XData5wVia,ZData5wVia, 'Color', [0.2, 0.6, 0.5], 'LineStyle', '-', 'LineWidth', 1.5)

% for j = 1:5
%     line(shaftX(:,j), shaftZ(:,j))
% end
line(-rubberLength/2, 0, 'Marker', '.', 'MarkerSize', 50)


xlabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)