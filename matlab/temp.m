%% 

for k = 1:10
    size = roads{k}.boxSize;
    low = roads{k}.position - size / 2;
    rectangle('Position', [low(1), low(2), size(1), size(2)]);
    axis([-30 30 -30 30]); grid on; 
end

%% Trajectory planning

%Waypoints
x0 = [-5.15, 9.15, -90]; 
x1 = [-5.15, 5.2, -90];
x2 = [-5.15, 5.2, -90];
x3 = [-4.75, 4.5, -45]; 
x4 = [-3.5, 3.7, 0];
x5 = [-2.5, 3.7, 0];
x5 = [4.5, 3.7, 0];

%Parameters
refPoses= [x0; x1; x2; x3; x4; x5];
refDirections = [1 1 1 1 1 1  ]';
numSmoothPoses = 50;

%SmoothPathSpline
[refPoses, directions, cumLength] = smoothPathSpline(refPoses,refDirections,numSmoothPoses);

% Plot the smoothed path
hold on
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off

%Speed settings
maxSpeed   = 5; % in meters/second
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second
%Speed profile
speedProfileGenerator = HelperSpeedProfileGenerator(startSpeed, endSpeed, maxSpeed);
refSpeeds = speedProfileGenerator(cumLength);
plotSpeedProfile(cumLength, refSpeeds, maxSpeed)
