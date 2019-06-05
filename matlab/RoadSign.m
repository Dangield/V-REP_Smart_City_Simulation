classdef RoadSign
	properties
		position = zeros(1, 2);
		type = -1;
		orientation = 0;
	end
	
	methods
		function obj = RoadSign(position, type, orientation)
			obj.position = position;
			obj.type = type;
			obj.orientation = orientation;
		end
	end
end

