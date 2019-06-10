classdef TrafficLight < handle
	properties
		vrepComm;
		handle      = -1;
		position    = zeros(1, 2);
		orientation = 0;
		name;
		updateMode;
		
		bulbHandles = zeros(1, 3); %r y g
		bulbNames   = cell(2, 3);
		bulbColors  = cell(2, 3);
		
		RED    = [1 0 0];
		YELLOW = [1 1 0];
		GREEN  = [0 1 0];
		
		state = 0;
	end
	
	methods
		function obj = TrafficLight(vrepComm, handle, name, position, orientation)
			obj.vrepComm    = vrepComm;
			obj.name        = name;
			obj.handle      = handle;
			obj.position    = position;
			obj.orientation = orientation;
			obj.updateMode  = vrepComm.vrep.simx_opmode_streaming;
			nameChar = char(name);
			number = str2num(nameChar(3:end));
			while number - 4 > 1
				number = number - 4;
			end
			if (mod(number, 2) && mod(number, 3) && mod(number, 4)) ...
					|| (~mod(number, 2) && mod(number, 3) && mod(number, 4))
				obj.state = 0;
			else
				obj.state = 2;
			end
			
			[childHandles, bulbNames] = vrepComm.getChildHandles(handle);
			obj.bulbHandles = flipud(childHandles');
			obj.bulbNames = flipud(bulbNames');
			
			multiplier = 4;
			
			obj.bulbColors = {obj.RED, obj.RED/multiplier; obj.YELLOW, obj.YELLOW/multiplier; obj.GREEN, obj.GREEN/multiplier};
			
		end
		
		function changeColor(obj, bulbIndex, bulbState, mode)
			obj.vrepComm.vrep.simxCallScriptFunction(obj.vrepComm.clientID, 'remoteApiCommandServer', obj.vrepComm.vrep.sim_scripttype_childscript, 'changeColor_function', obj.bulbHandles(bulbIndex), obj.bulbColors{bulbIndex, bulbState}, '', [], mode);
		end
		
		function update(obj)
			obj.state = mod(obj.state + 1, 4);
			mode = obj.vrepComm.vrep.simx_opmode_streaming;
			switch obj.state
				case 0
					obj.changeColor(1, 1, mode);
					obj.changeColor(2, 2, mode);
				case 1
					obj.changeColor(2, 1, mode);
				case 2
					obj.changeColor(1, 2, mode);
					obj.changeColor(2, 2, mode);
					obj.changeColor(3, 1, mode);
				case 3
					obj.changeColor(2, 1, mode);
					obj.changeColor(3, 2, mode);
			end
		end
		
		
	end
end

