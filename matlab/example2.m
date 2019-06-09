clear all;
clc;
%Run a simulation with a predefined path for the car

%%  Init simulation

if ~exist('vrepSim', 'var')
	vrepSim = VREPSimulation();
	vrepSim.startCommunication();
end

if ~vrepSim.initialised && vrepSim.vrepComm.clientID > -1
	tic
	vrepSim.init();
	toc
end

%% Draw roads
load('roads.mat');
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
transitionPoses = [x0; x1; x2; x3; x4; x5];
directions = [1 1 1 1 1 1  ]';
numSmoothPoses = 100;

%SmoothPathSpline
[refPoses, directions, cumLength] = smoothPathSpline(transitionPoses,directions,numSmoothPoses);
goalPose = refPoses(:,end);
% Plot the smoothed path
hold on
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off

%Vehicle settings
vehicleDims = vehicleDimensions('Wheelbase', 0.755);

%Speed settings
maxSpeed   = 20; % in meters/second
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second
MaxAcceleration = 20;
MaxDeceleration = 20;
%Speed profile
speedProfileGenerator = HelperSpeedProfileGenerator(startSpeed, endSpeed, maxSpeed,...
    MaxDeceleration, MaxDeceleration);
refSpeeds = speedProfileGenerator(cumLength);
%Patha Analyzer
pathAnalyzer = HelperPathAnalyzer(refPoses, refSpeeds, directions, ...
    'Wheelbase', vehicleDims.Wheelbase);
%Longitudal controller
sampleTime = 0.05;
%Timing
controlRate = HelperFixedRate(1/sampleTime); % in Hertz

%% Simulation
reachGoal = false;

vrepSim.updateCars();
iter = 0;
while ~reachGoal
    iter = iter+1;
    tic;
    vrepSim.updateCars();
    %Get Pos and Velocity
    currentPose(1:2) = vrepSim.cars{2}.position;
    currentPose(3) = vrepSim.cars{2}.orientation - 90; %przeliczenie
    tempVel = vrepSim.cars{2}.velocity;
    currentVel = double(sqrt(tempVel * tempVel')) / 0.07857;
    vlll(iter) = currentVel;
    % Find the reference pose on the path and the corresponding velocity
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);

    % Compute steering command
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction);
   
    %Send commands
    vrepSim.cars{2}.setSpeed(refVel);
    vrepSim.cars{2}.setAngle(steeringAngle);
    
    % Wait for fixed-rate execution
    waitfor(controlRate);
    
    % Check if the vehicle reaches the goal
    reachGoal = helperGoalChecker(goalPose, currentPose, currentVel,...
        endSpeed, direction);
    %toc;
end


