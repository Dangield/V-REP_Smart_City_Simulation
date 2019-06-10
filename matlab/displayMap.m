clc; clear; close all;
load('roads.mat');

%Set destination
destination = 16;

myMap = Map(roads);
figure(1);
myMap.displayIntersectionsPathGraph();
init_pos = [-5.15, 9.15, -90];
[intersectionNum, otherIntersectionNum] = myMap.findIntersection(init_pos);
figure(2);
myMap.displayRoads;

intersectionList = myMap.findShortestPath(intersectionNum, 1destination);

myPath = Path(intersectionList, init_pos);
myPath.displayTrajectory();