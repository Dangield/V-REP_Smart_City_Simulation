classdef VREPSimulation < handle
	properties
		vrepComm;
		cars = {};
		roads = {}
		carIterator = 0;
		CAR_NAME = "smartCar#";
		LANE_NAME = "Lane";
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
		
		function init(obj)
			[carHandles, carNames] = obj.vrepComm.getHandles(obj.CAR_NAME);
			for i = 1:size(carHandles, 2)
				obj.addCar(carNames(i), carHandles(i))
			end
			pause(1);
			obj.getRoads();
			obj.initialised = true;
		end
	end
end

