clc; clear; close all;
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

%% Settings

%Vehicle settings
vehicleDims = vehicleDimensions('Wheelbase', 0.755);
%Speed settings
maxSpeed   = 10; % in meters/second
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second
MaxAcceleration = 20;
MaxDeceleration = 20;
%Longitudal controller
sampleTime = 0.05;
%Timing
controlRate = HelperFixedRate(1/sampleTime); % in Hertz

%% Initial trajectory
load('roads.mat');
%Parameters
init_pos = [-5.15, 9.15, -90];

destination = 16;
%Myclasses
myMap = Map(roads);
intersectionList = myMap.findShortestPath2(init_pos, destination);
myPath = Path(intersectionList, init_pos);
goalPose = myPath.refPoses(end,:);
%Speed profile
speedProfileGenerator = HelperSpeedProfileGenerator(startSpeed, endSpeed, maxSpeed,...
    MaxDeceleration, MaxDeceleration);
refSpeeds = speedProfileGenerator(myPath.cumLength);
%Patha Analyzer
pathAnalyzer = HelperPathAnalyzer(myPath.refPoses, refSpeeds, myPath.directions, ...
    'Wheelbase', vehicleDims.Wheelbase);


%% Simulation
reachGoal = false;

vrepSim.updateCars();
while ~reachGoal
    tic;
    vrepSim.updateCars();
    %Get Pos and Velocity
    currentPose(1:2) = vrepSim.cars{2}.position;
    currentPose(3) = vrepSim.cars{2}.orientation - 90; %przeliczenie
    tempVel = vrepSim.cars{2}.velocity;
    currentVel = double(sqrt(tempVel * tempVel')) / 0.07857;
    % Find the reference pose on the path and the corresponding velocity
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);

    % Compute steering command
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'MaxSteeringAngle',45);
   
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


