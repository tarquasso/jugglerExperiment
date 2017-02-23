figure(1), clf
subplot(2,1,1)
plot(t,z)
subplot(2,1,2)
plot(z)

epsilon = 0.0005;
eps2 = 1e-4;

indStart = find(z > 0.5, 1, 'last');
% indStart = find(z < 0.5, 1, 'first');
indStart = find( (z > max(z(indStart-100:indStart)) - epsilon), 1, 'last' );
zStart = mean(z(indStart-240:indStart));
indStart = find(abs(z-zStart)<eps2,1,'last');

indFinish = indStart + find(z(indStart:indStart+100) < 0,1,'first') - 1;

deltat = t(indFinish) - t(indStart);
deltaz = z(indFinish) - z(indStart);

g = 9.81;

sbeta = (-2*deltaz/g)/deltat/deltat;
cbeta = sqrt(1 - sbeta^2);
betaEstimate = atan(sbeta/cbeta);

fprintf(['betaEstimate = ', num2str(betaEstimate*180/pi), '\n'])