classdef VREPSimulation < handle
	properties
		vrepComm;
		cars = {};
		roads = {}
		roadSigns = {};
		trafficLights = {};
		carIterator = 0;
		CAR_NAME = "smartCar#";
		LANE_NAME = "Lane";
		SIGN_NAME = "RoadSigns";
		TRAFFIC_LIGHT_NAME = "S#";
		initialised = false;
		trafficLightTimer;
	end
	
	methods
		function obj = VREPSimulation()
			a = timerfindall; delete(a);
			obj.vrepComm = VREPCommunication();
			obj.trafficLightTimer = timer;
			obj.trafficLightTimer.Period = 30;
			obj.trafficLightTimer.ExecutionMode = 'fixedRate';
			obj.trafficLightTimer.TimerFcn = @(~,~)obj.trafficLightTimerFunction;
		end
		
		function startCommunication(obj)
			obj.vrepComm.start();
		end
		
		function stopCommunication(obj)
			obj.vrepComm.stop();
		end
		
		function addCar(obj, name, handle)
			obj.carIterator = obj.carIterator + 1;
			obj.cars{obj.carIterator} = Car(obj.vrepComm, name, handle);
		end
		
		function updateCars(obj)
			for i = 1:size(obj.cars, 2)
				car = obj.cars{i};
				car.update();
			end
		end
		
		function updateLights(obj)
			for i = 1:size(obj.trafficLights, 2)
				trafficLight = obj.trafficLights{i};
				trafficLight.update();
			end
		end
		
		function trafficLightTimerFunction(obj)
			obj.updateLights();
			t = timer;
			t.StartDelay = 2;
			t.TimerFcn = @(~,~)obj.updateLights;
			start(t)
		end
		
		function getRoads(obj)
			[laneHandles, laneNames] = obj.vrepComm.getHandles(obj.LANE_NAME);
			
			for i = 1:size(laneHandles, 2)
				position = obj.vrepComm.getPosition(laneHandles(i), obj.vrepComm.vrep.simx_opmode_blocking);
				boxSize     = obj.vrepComm.getSize(laneHandles(i), obj.vrepComm.vrep.simx_opmode_blocking);
				obj.roads{i} = Road(position, boxSize);
			end
		end
		
		function getRoadSigns(obj)
			[signHandle, ~] = obj.vrepComm.getHandles(obj.SIGN_NAME);
			[signHandles, signNames] = obj.vrepComm.getChildHandles(signHandle);
			
			for i = 1:size(signHandles, 2)
				position    = obj.vrepComm.getPosition(signHandles(i), obj.vrepComm.vrep.simx_opmode_blocking);
				orientation = obj.vrepComm.getOrientation(signHandles(i), obj.vrepComm.vrep.simx_opmode_blocking);
				if contains(signNames{i}, 'Pierw', 'IgnoreCase', true)
					type = 0;
				elseif contains(signNames{i}, 'UstP', 'IgnoreCase', true)
					type = 1;
				else
					type = 2;
				end
				obj.roadSigns{i} = RoadSign(position, type, round(orientation));
			end
		end
		
		function getTrafficLights(obj)
			[lightHandles, lightNames] = obj.vrepComm.getHandles(obj.TRAFFIC_LIGHT_NAME);
			
			for i = 1:size(lightHandles, 2)
				position    = obj.vrepComm.getPosition(lightHandles(i), obj.vrepComm.vrep.simx_opmode_blocking);
				orientation = obj.vrepComm.getOrientation(lightHandles(i), obj.vrepComm.vrep.simx_opmode_blocking);
				obj.trafficLights{i} = TrafficLight(obj.vrepComm, lightHandles(i), lightNames(i), position, orientation);
			end
		end
		
		function init(obj)
			[carHandles, carNames] = obj.vrepComm.getHandles(obj.CAR_NAME);
			for i = 1:size(carHandles, 2)
				obj.addCar(carNames(i), carHandles(i))
			end
			pause(1);
			obj.getRoads();
			obj.getRoadSigns();
			obj.getTrafficLights();
			obj.initialised = true;
			start(obj.trafficLightTimer)
		end
		
		function removeCar(obj, index)
			car = obj.cars{index};
			for laserHandle = car.laserHandles
				obj.vrepComm.removeObject(laserHandle);
			end
			obj.vrepComm.removeModel(car.handle);
		end
		
		function delete(obj)
			obj.vrepComm.delete();
			stop(obj.trafficLightTimer);
			delete(obj.trafficLightTimer);
		end
	end
end

