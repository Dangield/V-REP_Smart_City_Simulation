classdef Car < handle
	properties
		vrepComm;
		handle   = -1;
		laserHandles = zeros(1, 14);
		laserDistances = zeros(1, 14);
		laserNames = zeros(1, 14);
		position = zeros(1, 2);
		orientation = 0;
		wheelSpeed = 0;
		velocity = zeros(1, 2);
		name;
		updateMode;
		
		steeringLeftHandle  = -1;
		steeringRightHandle = -1;
		
		motorLeftHandle  = -1;
		motorRightHandle = -1;
		
		d = 0.755;  % 2*d=distance between left and right wheels
		l = 2.5772; % l=distance between front and read wheels
	end
	
	methods
		function obj = Car(vrepComm, name, handle)
			obj.vrepComm   = vrepComm;
			obj.name       = name;
			obj.handle     = handle;
			obj.updateMode = vrepComm.vrep.simx_opmode_streaming;
			
			[carChildHandles, childNames] = vrepComm.getChildHandles(handle);
			proximitySensorHandle = carChildHandles(contains(childNames, "proximitySensors", 'IgnoreCase', true));
			[laserPointerHandles, obj.laserNames] = vrepComm.getChildHandles(proximitySensorHandle);
			
			leftDamperHandle = carChildHandles(contains(childNames, "frontLeftSpring", 'IgnoreCase', true));
			[leftCylinder, ~] = vrepComm.getChildHandles(leftDamperHandle);
			[obj.steeringLeftHandle, ~] = vrepComm.getChildHandles(leftCylinder);
			[leftCylinder, ~] = vrepComm.getChildHandles(obj.steeringLeftHandle);
			[obj.motorLeftHandle, ~] = vrepComm.getChildHandles(leftCylinder);
			
			rightDamperHandle = carChildHandles(contains(childNames, "frontRightSpring", 'IgnoreCase', true));
			[rightCylinder, ~] = vrepComm.getChildHandles(rightDamperHandle);
			[obj.steeringRightHandle, ~] = vrepComm.getChildHandles(rightCylinder);
			[rightCylinder, ~] = vrepComm.getChildHandles(obj.steeringRightHandle);
			[obj.motorRightHandle, ~] = vrepComm.getChildHandles(rightCylinder);
			
			i = 1;
			for laserPointerHandle = laserPointerHandles
				[childHandles, childNames] = vrepComm.getChildHandles(laserPointerHandle);
				obj.laserHandles(i) = childHandles(contains(childNames, "sensor", 'IgnoreCase', true));
				i = i+1;
			end
			
			
			
			obj.updateLasers();
			obj.updatePosition();
			obj.updateOrientation();
			obj.updateVelocity();
			obj.updateMode = vrepComm.vrep.simx_opmode_buffer;
		end
		
		function updatePosition(obj)
			obj.position = obj.vrepComm.getPosition(obj.handle, obj.updateMode);
		end
		
		function updateVelocity(obj)
			obj.velocity = obj.vrepComm.getVelocity(obj.handle, obj.updateMode);
        end
        
        function updateOrientation(obj)
			obj.orientation = obj.vrepComm.getOrientation(obj.handle, obj.updateMode);
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
            obj.updateOrientation();
		end
		
		function setSpeed(obj, desiredSpeed)
			obj.vrepComm.vrep.simxSetJointTargetVelocity(obj.vrepComm.clientID, obj.motorLeftHandle,  desiredSpeed, obj.vrepComm.vrep.simx_opmode_streaming);
			obj.vrepComm.vrep.simxSetJointTargetVelocity(obj.vrepComm.clientID, obj.motorRightHandle, desiredSpeed, obj.vrepComm.vrep.simx_opmode_streaming);
			obj.wheelSpeed = desiredSpeed;
		end
		
		function setAngle(obj, desiredAngle)
			if (desiredAngle > 45)
				desiredAngle = 45;
			end
			if (desiredAngle < -45)
				desiredAngle = -45;
			end
			
			desiredAngle = desiredAngle*pi/180;
			
			steeringAngleRight  = atan(obj.l/(-obj.d+obj.l/tan(desiredAngle)));
			steeringAngleLeft = atan(obj.l/(obj.d+obj.l/tan(desiredAngle)));
		
			obj.vrepComm.vrep.simxSetJointTargetPosition(obj.vrepComm.clientID, obj.steeringLeftHandle,  steeringAngleLeft,  obj.vrepComm.vrep.simx_opmode_streaming);
			obj.vrepComm.vrep.simxSetJointTargetPosition(obj.vrepComm.clientID, obj.steeringRightHandle, steeringAngleRight, obj.vrepComm.vrep.simx_opmode_streaming);
		end
	end
end

