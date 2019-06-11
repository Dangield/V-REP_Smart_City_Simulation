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
sampleTime = 0.1;
%Timing
controlRate = HelperFixedRate(1/sampleTime); % in Hertz

%% Initial trajectory
load('roads.mat');
vrepSim.updateCars();
numCars = length(vrepSim.cars);
%Destination
destination = [14, 14, 9, 23];

%Every car has a map and it's own trajectory generator
tempMap = Map(roads);
worldMap = tempMap;
%Init arrays
carMap = {};
carPath = {};
speedProfileGenerato = {};


%Initialize maps, trajectories
for k = 1:numCars
    init_pos(k,:) = vrepSim.cars{k}.getPosition;
    carMap{k} = tempMap;
    intersectionList = carMap{k}.findShortestPath2(init_pos(k,:), destination(k));
    carPath{k} = Path(intersectionList, init_pos(k,:));
    goalPose(k,:) = carPath{k}.refPoses(end,:);
    
    %Speed profile
    speedProfileGenerator = HelperSpeedProfileGenerator(startSpeed, endSpeed, maxSpeed,...
        MaxDeceleration, MaxDeceleration);
    refSpeeds(k,:) = speedProfileGenerator(carPath{k}.cumLength);
    
    %Patha Analyzer
    pathAnalyzer{k} = HelperPathAnalyzer(carPath{k}.refPoses, refSpeeds(k,:)', carPath{k}.directions, ...
    'Wheelbase', vehicleDims.Wheelbase);
    reachGoal(k) = false;
    
    %Current pose
    currentPose(k,:) = vrepSim.cars{k}.getPosition;
end

%% Simulation
vrepSim.updateCars();
reachAllGoal = false;
while ~reachAllGoal
    vrepSim.updateCars();
    worldMap.buildAgents(currentPose); 
    for k = 1:numCars        
        %Get Pos and Velocity and Permission        
        currentPose(k,:) = vrepSim.cars{k}.getPosition;
        tempVel = vrepSim.cars{k}.velocity;
        currentVel = double(sqrt(tempVel * tempVel')) / 0.07857;
        currPermission = worldMap.getPermission(k, currentPose(k,:));
        currentLaser = vrepSim.cars{k}.laserDistances(1:2); 
        
        % Find the reference pose on the path and the corresponding velocity
        [refPose, refVel, direction] = pathAnalyzer{k}(currentPose(k,:), currentVel);
        
        % Compute steering command
        steeringAngle = lateralControllerStanley(refPose, currentPose(k,:), currentVel, ...
            'Direction', direction, 'MaxSteeringAngle',45);

        %Send commands
        if currPermission            
                          
        else
            refVel = 0;
        end
        %Collision detection
        if currentLaser < [0.5 0.5]
            refVel = 0;
        end
        vrepSim.cars{k}.setSpeed(refVel); 

        vrepSim.cars{k}.setAngle(steeringAngle);
        
        

        %Check if the vehicle reaches the goal
        reachGoal(k) = helperGoalChecker(goalPose, currentPose, currentVel,...
            endSpeed, direction);
    end
    reachAllGoal = true;
    for k = 1:numCars
        reachAllGoal = reachAllGoal & reachGoal(k);
    end
    % Wait for fixed-rate execution
    waitfor(controlRate);
end


