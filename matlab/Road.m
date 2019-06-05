classdef Road
	properties
		position = zeros(1, 2);
		boxSize  = zeros(1, 2);
	end
	
	methods
		function obj = Road(position, boxSize)
			obj.position = position;
			obj.boxSize  = boxSize;
		end
	end
end

