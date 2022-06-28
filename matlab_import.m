close all
clear 
clc

folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

vispa = importrobot('VISPA_modifiedDH.urdf');
vispa.DataFormat = 'row';
vispa.Gravity = [0,0,-9.81];

nJoints = 6;
posStart = [0,0,pi/2,0,pi/2,pi/2];
posEnd = [pi/4, -pi/2, 0, pi, 0, 0];

timeStart = 0;
timeEnd = 3;
numberPoints = 100;

time = linspace(timeStart, timeEnd, numberPoints);

jointPositions = zeros(nJoints, length(time));
jointVelocities = zeros(nJoints, length(time));
jointAccelerations = zeros(nJoints, length(time));

for nJoint = 1:6
    
    polyCoefficientsPos = ...
        get5thPolynomialTrajectoryCoefficients(timeStart, timeEnd,...
                                               posStart(nJoint), ...
                                               posEnd(nJoint));
                                           
    polyCoefficientsVel = polyder(polyCoefficientsPos);
    polyCoefficientsAcc = polyder(polyCoefficientsVel);
                                           
    for timeIndex = 1:length(time)
        sampleTime = time(timeIndex);
        jointPositions(nJoint, timeIndex) = polyval(polyCoefficientsPos, ...
                                                    sampleTime);
        jointVelocities(nJoint, timeIndex) = polyval(polyCoefficientsVel, ...
                                                    sampleTime);
        jointAccelerations(nJoint, timeIndex) = polyval(polyCoefficientsAcc, ...
                                                    sampleTime);
    end
end

figure;
axis equal

jointTorques = zeros(nJoints, length(time));
for timeIndex = 1:length(time)
    torques = inverseDynamics(vispa, ...
                              jointPositions(:, timeIndex)',...
                              jointVelocities(:, timeIndex)',...
                              jointAccelerations(:, timeIndex)');
    jointTorques(:, timeIndex) = torques';
    
    show(vispa, jointPositions(:,timeIndex)','Visuals', 'off')                     
    endEffectorPose = getTransform(vispa, jointPositions(:,timeIndex)',...
                                   'Link_6');
                               
    endEffectorPos = endEffectorPose(1:3,4)';
    hold on;
    plot3(endEffectorPos(1),endEffectorPos(2),endEffectorPos(3),'k*')
   
end

show(vispa, posStart, 'Visuals', 'on');
show(vispa, posEnd, 'Visuals', 'on');

figure;
hold on;
tags = strings(1,nJoints);
for nJoint = 1:6
    p(nJoint) = plot(time,jointPositions(nJoint,:));
    tags(nJoint) = sprintf('Joint%u', nJoint);
end
legend(p(1:nJoint), tags)
title('Example Trajectory: Joint Positions')
xlabel('Joint Position [rad]')
ylabel('Time [s]')

figure;
hold on;
tags = strings(1,nJoints);
for nJoint = 1:6
    p(nJoint) = plot(time,jointVelocities(nJoint,:));
    tags(nJoint) = sprintf('Joint%u', nJoint);
end
legend(p(1:nJoint), tags)
title('Example Trajectory: Joint Velocities')
xlabel('Joint Velocities [rad/s]')
ylabel('Time [s]')

figure;
hold on;
tags = strings(1,nJoints);
for nJoint = 1:6
    p(nJoint) = plot(time,jointAccelerations(nJoint,:));
    tags(nJoint) = sprintf('Joint%u', nJoint);
end
legend(p(1:nJoint), tags)
title('Example Trajectory: Joint Accelerations')
xlabel('Joint Accelerations [rad/s2]')
ylabel('Time [s]')

figure;
hold on;
tags = strings(1,nJoints);
for nJoint = 1:6
    p(nJoint) = plot(time,jointTorques(nJoint,:));
    tags(nJoint) = sprintf('Joint%u', nJoint);
end
legend(p(1:nJoint), tags)
title('Example Trajectory: Joint Torques')
xlabel('Joint Torques [Nm]')
ylabel('Time [s]')


function polyCoefficients = get5thPolynomialTrajectoryCoefficients(timeStart, timeEnd, posStart, posEnd)

A = [...
    1, timeStart, timeStart^2, timeStart^3, timeStart^4, timeStart^5;...
    1, timeEnd, timeEnd^2, timeEnd^3, timeEnd^4, timeEnd^5;...
    0, 1, 2*timeStart, 3*timeStart^2, 4*timeStart^3, 5*timeStart^4;...
    0, 1, 2*timeEnd, 3*timeEnd^2, 4*timeEnd^3, 5*timeEnd^4;
    0, 0, 1, 6*timeStart, 12*timeStart^2, 20*timeStart^3;
    0, 0, 1, 6*timeEnd, 12*timeEnd^2, 20*timeEnd^3];

b = [posStart, posEnd, 0, 0, 0, 0]';

polyCoefficients = flipud(A\b);

end
