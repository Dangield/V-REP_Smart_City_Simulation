classdef Path < handle
    properties
        intersectionList = [];
        intersectionNum = 0;
        intersectionDirList = [];
        initialPos = zeros(1,3);
        transitionPoses = [];
        refPoses = [];
        directions = [];
        cumLength = [];
        goalPose = [];
    end
    
    methods
        %Constructor
        function obj = Path(intersectionList, init_pos, handle)
            obj.intersectionList = intersectionList(:,1:2);    
            obj.intersectionNum = length(obj.intersectionList);
            
            obj.setInitialPos(init_pos);
            obj.orientation();
            obj.buildPath();
            obj.buildTrajectory();
        end
        
        %Create intersection list with directions
        function orientation(obj)
            for k = 1 : obj.intersectionNum - 1
               if obj.intersectionList(k+1, 1) - obj.intersectionList(k, 1) > 0
                   angle = 0;
               elseif obj.intersectionList(k+1, 1) - obj.intersectionList(k, 1) < 0
                   angle = 180;
               elseif obj.intersectionList(k+1, 2) - obj.intersectionList(k, 2) > 0
                   angle = 90;    
               elseif obj.intersectionList(k+1, 2) - obj.intersectionList(k, 2) < 0
                   angle = -90;     
               end
               obj.intersectionDirList(k,:) = [obj.intersectionList(k,:), angle];
            end
           obj.intersectionDirList(obj.intersectionNum,:) = [obj.intersectionList(obj.intersectionNum,:),angle];
        end
        
        %Set vehicle initial position
        function setInitialPos(obj, initialPos)
            obj.initialPos = initialPos;
        end
        
        %Build path
        function buildPath(obj)        
            iter = 1;
            newPath = [];
            ROAD_WIDTH = 0.5;
            tempPath = [obj.initialPos; obj.intersectionDirList];
            numOfNodes = obj.intersectionNum + 1;
            %Add points to smooth the path
            for k = 2:numOfNodes
                %Extract path components
                prevX = tempPath(k-1,1); prevY = tempPath(k-1,2); prevA = tempPath(k-1,3);
                currX = tempPath(k,1); currY = tempPath(k,2); currA = tempPath(k,3);
                if k ~= numOfNodes
                    nextX = tempPath(k+1,1); nextY = tempPath(k+1,2); nextA = tempPath(k+1,3);
                end
                %Add points before crossroad
                if 1 < currX- prevX
                    newPath(iter,:) = [currX - 1.1, currY - ROAD_WIDTH, prevA]; iter = iter + 1;
                    newPath(iter,:) = [currX - 1, currY - ROAD_WIDTH, prevA]; iter = iter + 1;
                elseif prevX - currX > 1
                    newPath(iter,:) = [currX + 1.1, currY + ROAD_WIDTH, prevA]; iter = iter + 1;
                    newPath(iter,:) = [currX + 1, currY + ROAD_WIDTH, prevA]; iter = iter + 1;
                elseif 1 < currY - prevY
                    newPath(iter,:) = [currX + ROAD_WIDTH, currY - 1.1, prevA]; iter = iter + 1;
                    newPath(iter,:) = [currX + ROAD_WIDTH, currY - 1, prevA]; iter = iter + 1;
                elseif prevY - currY > 1
                    newPath(iter,:) = [currX - ROAD_WIDTH, currY + 1.1, prevA]; iter = iter + 1;
                    newPath(iter,:) = [currX - ROAD_WIDTH, currY + 1, prevA]; iter = iter + 1;
                end
                %For the last node - only points beofre are added
                if k == numOfNodes;
                    break
                end        

                %Add points after crossroad
                if nextX < currX
                    newPath(iter,:) = [currX - 1, currY + ROAD_WIDTH, currA]; iter = iter + 1;
                    newPath(iter,:) = [currX - 1.1, currY + ROAD_WIDTH, currA]; iter = iter + 1;
                elseif nextX > currX
                    newPath(iter,:) = [currX + 1, currY - ROAD_WIDTH, currA]; iter = iter + 1;
                    newPath(iter,:) = [currX + 1.1, currY - ROAD_WIDTH, currA]; iter = iter + 1;
                elseif nextY > currY
                    newPath(iter,:) = [currX + ROAD_WIDTH, currY + 1, currA]; iter = iter + 1;
                    newPath(iter,:) = [currX + ROAD_WIDTH, currY + 1.1, currA]; iter = iter + 1;
                elseif nextY < currY
                    newPath(iter,:) = [currX - ROAD_WIDTH, currY - 1, currA]; iter = iter + 1;
                    newPath(iter,:) = [currX - ROAD_WIDTH, currY - 1.1, currA]; iter = iter + 1;
                end
            end
            obj.transitionPoses = [obj.initialPos; newPath];            
        end      
               
        %Build trajectory
        function buildTrajectory(obj)
            %Parameters
            tempDirections = ones(length(obj.transitionPoses),1);
            numSmoothPoses = 200;
            
            %SmoothPathSpline
            [obj.refPoses, obj.directions, obj.cumLength] = smoothPathSpline(obj.transitionPoses,tempDirections,numSmoothPoses);
            obj.goalPose = obj.refPoses(:,end);
        end
        
        %Display trajectory
        function displayTrajectory(obj)
            hold on
            plot(obj.refPoses(:, 1), obj.refPoses(:, 2), 'r', 'LineWidth', 2, ...
                'DisplayName', 'Smoothed Path');
            hold off
        end  
    end        
end