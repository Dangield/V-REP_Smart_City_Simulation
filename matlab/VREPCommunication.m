classdef VREPCommunication < handle
	properties
		clientID;
		vrep;
	end
	
	methods
		function obj = VREPCommunication()
			addpath('C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab\');
			obj.vrep = remApi('remoteApi');
			obj.vrep.simxFinish(-1);
			obj.clientID = -1;
		end
		
		function delete(obj)
            obj.vrep.delete();
        end
		
		function start(obj)
			obj.clientID = obj.vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
			if (obj.clientID <= -1)
				disp('Failed connecting to remote API server');
			else
				disp('Program started');
			end
		end
		
		function stop(obj)
			obj.vrep.delete();
			disp('Program ended');
		end
		
		function [retHandles, retNames] = getHandles(obj, name)
			if (obj.clientID > -1)
				[returnCode, handles, intData, floatData, stringData] = obj.vrep.simxGetObjectGroupData(obj.clientID, obj.vrep.sim_appobj_object_type, 0, obj.vrep.simx_opmode_blocking);
				if (returnCode == 0)
					retHandles = [];
					retNames   = [];
					handlesNames = stringData(:, 1);
					for i = 1:size(handlesNames, 1)
						if contains(handlesNames{i}, name, 'IgnoreCase', true)
							retHandles = [retHandles, handles(i)];
							retNames   = [retNames, string(handlesNames{i})];
						end
					end
				end
			end
		end
		
		function [childHandles, childNames] = getChildHandles(obj, handle)
			if (obj.clientID>-1)
				[returnCode, handles, intData, floatData, stringData] = obj.vrep.simxGetObjectGroupData(obj.clientID, obj.vrep.sim_appobj_object_type, 0, obj.vrep.simx_opmode_blocking);
				childHandles = [];
				childNames   = [];
				childIndex   = 0;
				childHandle  = 0;
				while true
					[returnCode, childHandle] = obj.vrep.simxGetObjectChild(obj.clientID, handle, childIndex, obj.vrep.simx_opmode_blocking);
					if (childHandle == -1)
						break
					end
					childIndex = childIndex + 1;
					childHandles = [childHandles, childHandle];
					childNames   = [childNames, string(stringData{childHandle+1})];
				end
			end
		end
		
		function position = getPosition(obj, handle, mode)
			[returnCode, position] = obj.vrep.simxGetObjectPosition(obj.clientID, handle, -1, mode);
			if (returnCode == 0)
				position = position(1:2);
			end
		end
		
		function size = getSize(obj, handle, mode)
			[~, minx] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, handle, 21, mode);
			[~, maxx] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, handle, 24, mode);
			[~, miny] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, handle, 22, mode);
			[~, maxy] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, handle, 25, mode);
			
			size = [maxx - minx, maxy - miny];
		end
		
		function orientation = getOrientation(obj, handle, mode)
			[returnCode, orientation] = obj.vrep.simxGetObjectOrientation(obj.clientID, handle, -1, mode);
			orientation = orientation(3)*180/pi;
		end
		
	end
end

