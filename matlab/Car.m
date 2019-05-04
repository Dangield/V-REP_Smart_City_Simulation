classdef Car < handle
	properties
		vrepComm;
		handle   = -1;
		laserHandles = zeros(1, 14);
		laserDistances = zeros(1, 14);
		laserNames = zeros(1, 14);
		position = zeros(1, 2);
		name;
		updateMode;
	end
	
	methods
		function obj = Car(vrepComm, name, handle)
			obj.vrepComm   = vrepComm;
			obj.name       = name;
			obj.handle     = handle;
			obj.updateMode = vrepComm.vrep.simx_opmode_streaming;
			
			[childHandles, childNames] = vrepComm.getChildHandles(handle);
			proximitySensorHandle = childHandles(contains(childNames, "proximitySensors", 'IgnoreCase', true));
			[laserPointerHandles, obj.laserNames] = vrepComm.getChildHandles(proximitySensorHandle);

			i = 1;
			for laserPointerHandle = laserPointerHandles
				[childHandles, childNames] = vrepComm.getChildHandles(laserPointerHandle);
				obj.laserHandles(i) = childHandles(contains(childNames, "sensor", 'IgnoreCase', true));
				i = i+1;
			end
			obj.updateLasers();
			obj.updatePosition();
			obj.updateMode = vrepComm.vrep.simx_opmode_buffer;
		end
		
		function updatePosition(obj)
			obj.position = obj.vrepComm.getPosition(obj.handle, obj.updateMode);
		end
		
		function updateLasers(obj)
			for i  = 1 : size(obj.laserHandles, 2)
				[rtn, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector] = obj.vrepComm.vrep.simxReadProximitySensor(obj.vrepComm.clientID, obj.laserHandles(i), obj.updateMode);
				if rtn == 0
					obj.laserDistances(i) = detectedPoint(3);
				end
			end
		end
		
		function update(obj)
			obj.updateLasers();
			obj.updatePosition();
		end
	end
end

