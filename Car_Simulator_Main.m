clear all; clc; close all;

figure(1)
axis equal
numCars = 1;

testCar = Car;

% for i = 1:numCars
%     highway.introduce();
% end
dt = .1;
velArr = testCar.vel(2);
accArr = testCar.acc(2);
tArr = 0;
desiredSpeed = testCar.getDesiredSpeed;
desiredSpeed = desiredSpeed(2);

while 1
%     figure(1)
    testCar.update(dt);
    drawnow
    
    figure(2)
    clf
    velArr = [velArr, testCar.vel(2)];
    accArr = [accArr, testCar.acc(2)];
    tArr = [tArr, tArr(end) + dt];
    plot(tArr, velArr); 
    hold on
    plot(tArr, desiredSpeed * ones(1, length(tArr)));
    plot(tArr, accArr);
%     legend({'Y-Speed', 'desired Speed', 'Y-Accel'});
%     xlabel('time (s)')
%     ylabel('m/s (v) m/s^2 (a)')
    
end