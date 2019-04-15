function Sensors()
	addpath('C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab\')
	
    disp('Program started');
	
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getSensorData_function',[],[],'Hello world!',[],vrep.simx_opmode_blocking);
		if (res==vrep.simx_return_ok)
            fprintf('Everything OK!\n');
			for i=1:length(retFloats)
				fprintf('Distance read: %.4f\n', retFloats(i))
			end
        else
            fprintf('Remote function call failed\n');
        end
    else
        disp('Failed connecting to remote API server');
	end
	
    vrep.delete(); % call the destructor!
    disp('Program ended');
end