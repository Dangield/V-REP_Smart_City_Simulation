classdef VREPSimulation < handle
	properties
		vrepComm;
		cars = {};
		roads = {}
		roadSigns = {};
		carIterator = 0;
		CAR_NAME = "smartCar#";
		LANE_NAME = "Lane";
		SIGN_NAME = "RoadSigns";
		initialised = false;
	end
	
	methods
		function obj = VREPSimulation()
			obj.vrepComm = VREPCommunication();
		end
		
		function startCommunication(obj)
			obj.vrepComm.start();
		end
		
		function stopCommunication(obj)
			obj.vrepComm.stop();
		end
		
		function delete(obj)
            obj.vrepComm.delete();
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
		
		function init(obj)
			[carHandles, carNames] = obj.vrepComm.getHandles(obj.CAR_NAME);
			for i = 1:size(carHandles, 2)
				obj.addCar(carNames(i), carHandles(i))
			end
			pause(1);
			obj.getRoads();
			obj.getRoadSigns();
			obj.initialised = true;
		end
	end
end

