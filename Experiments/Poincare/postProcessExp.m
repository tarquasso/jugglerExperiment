importExpData()

figure(2)
plot(t,z)

takeLast = 5; % seconds
finalTime = t(end);

ox = 0.448;
oz = -0.066;
rx = 0.029;
rz = 0.02075;
rubberLength = 0.542;
zmax = 0.3;
% r = 0.0439;

% Normalize x and z directions to a world frame situated at the middle of
% the rubber when it is horizontal.
x = ox-rubberLength/2-rx-x;
z = -oz-rz+z;

tLast = t(t>(finalTime-takeLast));
xLast = x(t>(finalTime-takeLast));
zLast = z(t>(finalTime-takeLast));
psiLast = psi(t>(finalTime-takeLast));

figure(1), clf
subplot(2,2,1)
plot(tLast, xLast)
axis('tight')
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(2,2,2)
plot(tLast, zLast)
axis('tight')
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(2,2,3)
plot(tLast, psiLast)
axis('tight')
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$\psi$ [rad]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(2,2,4)
plot(xLast, zLast)
axis([-rubberLength/2, rubberLength/2, min(z), zmax+0.05])
xlabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)

% figure(2), clf
% plot(xLast,zLast)
% axis([-rubberLength/2, rubberLength/2, 0, zmax])
% xlabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
% ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)