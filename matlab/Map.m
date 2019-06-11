classdef Map < handle
    properties
        roadsList = {};
        roadIntersectionsMat = {}
        intersectionsList = [];
        intersectionsPathMat = [];
        intersectionsPathMatIndex = []
        intersectionsPathGraph = [];
        pathList = [];
        numRoads = 0;
        numPaths = 0;
        numIntersections = 0;
        
        %Car-Path-Intersection list
        carPathIntersectionList = [];
    end
    
    methods
        %Constructor
        function obj = Map(roads, handle)
            obj.roadsList = roads;            
            obj.numRoads = length(roads);
            
            %Build roads intersection matrix and list
            obj.roadIntersectionsMat = cell(10,10);
            iter = 1;
            for k = 1:obj.numRoads   
                for j = k+1:obj.numRoads
                    [flag, pos] = obj.roadIntersect(k, j);
                    if flag
                        obj.intersectionsList(iter, :) = [pos, iter];
                        obj.roadIntersectionsMat{k,j} = [pos, iter];
                        obj.roadIntersectionsMat{j,k} = [pos, iter];
                        iter = iter + 1;
                    end
                end
            end
            obj.numIntersections = length(obj.intersectionsList);
            
            %Build road matrix and list
            iter = 1;
            for k = 1:obj.numRoads
                %Given row
                tempM = obj.roadIntersectionsMat(k,:);
                %Remove empty cells
                tempM = tempM(~cellfun('isempty', tempM))';    
                tempM = cell2mat(tempM);
                %Check if road is horizontal or vertical then sort
                if tempM(1,1) ~= tempM(2,1)
                    tempM = sortrows(tempM, 1);   
                else
                    tempM = sortrows(tempM, 2);   
                end    
                %Build road matrix
                [rows, cols] = size(tempM);
                for l = 2:rows
                    %Calculate distance
                    Road_len = pdist([tempM(l-1,1:2); tempM(l, 1:2)],'euclidean');
                    %Build graph
                    index1 = tempM(l-1,3);
                    index2 = tempM(l, 3);        
                    obj.intersectionsPathMat(index1, index2) = Road_len;
                    obj.intersectionsPathMat(index2, index1) = Road_len;
                    obj.intersectionsPathMatIndex(index1, index2) = iter;
                    obj.intersectionsPathMatIndex(index2, index1) = iter;
                    %Build list
                    obj.pathList(iter,:) = [index1, index2, iter]; iter = iter + 1;
                    
                end
            end            
            obj.numPaths = length(obj.pathList);
            
            %Build graph
            obj.buildIntersectionsPathGraph();
        end
        
        %Plot roads
        function displayRoads(obj)
            for k = 1:obj.numRoads
                high = obj.roadsList{k}.boxSize;
                low = obj.roadsList{k}.position - high / 2;
                rectangle('Position', [low(1), low(2), high(1), high(2)]);
                axis([-30 30 -30 30]); grid on; 
            end
        end 
        
        %Check if two roads intersect
        function [out, pos] = roadIntersect(obj, roadIndex1, roadIndex2)
            %Temporary variables
            road1 = obj.roadsList{roadIndex1};
            road2 = obj.roadsList{roadIndex2};           
            
            %Load road coordinates
            size1 = road1.boxSize;
            pos1 = road1.position;
            left1 = pos1(1) - size1(1) / 2;
            bottom1 = pos1(2) - size1(2) / 2;
            right1 = pos1(1) + size1(1) / 2;
            top1 = pos1(2) + size1(2) / 2;

            size2 = road2.boxSize;
            pos2 = road2.position;
            left2 = pos2(1) - size2(1) / 2;
            bottom2 = pos2(2) - size2(2) / 2;
            right2 = pos2(1) + size2(1) / 2;
            top2 = pos2(2) + size2(2) / 2;

            %Crossing flag and pos
            out = 1;

            %Calculate intersection position
            if (top1 - bottom1 == 2)
                posy = (top1 + bottom1) / 2;
            else
                posy = (top2 + bottom2) / 2;
            end
            if (right1 - left1 == 2)
                posx = (left1 + right1) / 2;
            else
                posx = (left2 + right2) / 2;
            end

            %Check overlapping
            if (top1 < bottom2 || top2 < bottom1)
                out = 0;
                posx = 0;
                posy = 0;
            else
                if (right1 < left2 || right2 < left1)
                    out = 0;
                    posx = 0;
                    posy = 0;
                end
            end     

            pos = [posx, posy];    
        end
        
        %Find path index for given position
        function [firstPath, allPath] = findPath(obj, pos)
            %Remove angle
            pos = pos(1:2);            
            %Default return value
            allPath = [-1];
            %Iterate over all paths
            iter = 1;
            for k = 1:obj.numPaths
                %Get road's crosses
                intersection1 = obj.pathList(k,1);
                intersection2 = obj.pathList(k,2);
                %Get intersection's positions
                pos1 = obj.intersectionsList(intersection1, 1:2);
                pos2 = obj.intersectionsList(intersection2, 1:2);
                %Check if car's position on roads
                if obj.onRoad(pos, pos1, pos2)
                    allPath(iter) = obj.pathList(k,3); iter = iter + 1;
                end
            end
            firstPath = allPath(1);
        end
        
        %Find closest intersection index for given position
        function [intersectionNum, otherIntersectionNum] = findIntersection(obj, pos)
            %ExtractAngle
            posA = pos(3);
            pos = pos(1:2);
            %Get index of path
            [pathNum, allPath] = obj.findPath(pos);
            %Get coord
            path = obj.pathList(pathNum,:);
            %Temporary intersection pair
            tempInterectionPair = obj.intersectionsList([path(1), path(2)],:);
            
            if sind(posA) < -sqrt(2) / 2
                [M,Index] = min(tempInterectionPair(:,2));                
            elseif sind(posA) > sqrt(2) / 2
                [M,Index] = max(tempInterectionPair(:,2));
            elseif cosd(posA) > sqrt(2) / 2
                [M,Index] = max(tempInterectionPair(:,1));
            elseif cosd(posA) < sqrt(2) / 2
                [M,Index] = min(tempInterectionPair(:,1));
            end
            %Select other node
            intersectionNum = tempInterectionPair(Index, 3);
            otherIntersectionNum = tempInterectionPair(mod(Index,2)+1, 3);
        end
       
        %Point in Rectangle
        function bool = onRoad(obj, pos, pos1, pos2)
            %Default value
            bool = 0;
            x1 = pos1(1); y1 = pos1(2);
            x2 = pos2(1); y2 = pos2(2);          
            
            %Pos on road condition
            if abs(x1-x2) < 0.1 %road vertical
                if y1 > y2
                    xv = [x1-1 x1+1 x2+1 x2-1];
                    yv = [y1 y1 y2 y2];
                else
                    xv = [x2-1 x2+1 x1+1 x1-1];
                    yv = [y2 y2 y1 y1];
                end
            else
                if x1 > x2
                    xv = [x1 x1 x2 x2];
                    yv = [y1-1 y1+1 y2+1 y2-1];
                else
                    xv = [x2 x2 x1 x1];
                    yv = [y2-1 y2+1 y1+1 y1-1];                    
                end                
            end
            [in, on] = inpolygon(pos(1),pos(2),xv,yv);            
            bool = in | on;
        end
        
        %Calculate distane between point and line
        function d = point_to_line(obj, pt, v1, v2)
              a = v1 - v2;
              b = pt - v2;
              d = norm(cross(a,b)) / norm(a);
        end
        
        %Build intersectionsPathGraph
        function buildIntersectionsPathGraph(obj)
            obj.intersectionsPathGraph = graph(obj.intersectionsPathMat);
        end
        
        %Plot intersectionsPathGraph
        function displayIntersectionsPathGraph(obj)
            plot(obj.intersectionsPathGraph);
        end

        %Find shortest path
        function intersectionList = findShortestPath(obj, intersection1, intersection2)
            intersectionIndex = shortestpath(obj.intersectionsPathGraph,intersection1, intersection2);
            intersectionList = obj.intersectionsList(intersectionIndex,:);            
        end       
        
        %Find shortest path
        function intersectionList = findShortestPath2(obj, pos, intersection2)
            [intersection1, other] = obj.findIntersection(pos);
            intersectionIndex = shortestpath(obj.intersectionsPathGraph,intersection1, intersection2);
            intersectionList = obj.intersectionsList(intersectionIndex,:);            
        end       
        
        %Build car-path-intersection object
        function buildAgents(obj, pos)
            [row, col] = size(pos);
            for k = 1:row
                currPos = pos(k,:);
                [currPath, allPath] = obj.findPath(currPos);
                currIntersection = obj.findIntersection(currPos);
                %Don't update if car on 2 paths
                if length(allPath) == 1
                    obj.carPathIntersectionList(k,:) = [k, currPath, currIntersection];
                end
            end
        end    
        
        %Send stop or start command to car
        function perm = getPermission(obj, carNum)
            currIntersection = obj.carPathIntersectionList(carNum,3);
            currPriority = obj.carPathIntersectionList(carNum,2);
            tempCarPathIntersectionList = obj.carPathIntersectionList(obj.carPathIntersectionList(:,3) == currIntersection,:);
            if max(tempCarPathIntersectionList(:,2)) == currPriority            
                perm = 1;
            else
                perm = 0;
            end
        end
    end
end