[fNumber, t, x, z, xd, zd, psi] = importfile('CClient-output.pts');

fig = figure(1); clf
subplot(3,2,1)
plot(t,x, 'LineWidth', 2)
axis tight
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(3,2,2)
plot(t,xd, 'LineWidth', 2)
axis tight
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(3,2,3)
plot(t,z, 'LineWidth', 2)
axis tight
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(3,2,4)
plot(t,zd, 'LineWidth', 2)
axis tight
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$\dot{z}$ [m/s]', 'Interpreter', 'LaTeX', 'FontSize', 15)
subplot(3,2,5:6)
plot(t,psi, 'LineWidth', 2)
axis tight
xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
ylabel('$\psi$ [rad]', 'Interpreter', 'LaTeX', 'FontSize', 15)

fig.Name = 'Collected Data';
fig.NumberTitle = 'off';
fig.OuterPosition = [680 528 660 600];