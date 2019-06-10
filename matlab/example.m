clc
clear all
close all

if ~exist('vrepSim', 'var')
	vrepSim = VREPSimulation();
	vrepSim.startCommunication();
end

if ~vrepSim.initialised && vrepSim.vrepComm.clientID > -1
	tic
	vrepSim.init();
	toc
end

times = [];
for i = 1:60
	tic
	vrepSim.updateCars();
	times(i) = toc;
	vrepSim.cars{1}.position
	vrepSim.cars{1}.laserDistances
end
avgTime = mean(times);
