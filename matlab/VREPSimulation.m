classdef VREPSimulation < handle
	properties
		vrepComm;
		cars = {};
		carIterator = 0;
		CAR_NAME = "smartCar#";
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
		
		function init(obj)
			[carHandles, carNames] = obj.vrepComm.getHandles(obj.CAR_NAME);
			for i = 1:size(carHandles, 2)
				obj.addCar(carNames(i), carHandles(i))
			end
			pause(1);
			obj.initialised = true;
		end
	end
end

