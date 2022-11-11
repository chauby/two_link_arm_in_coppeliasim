%% Rodrigues: function description
function [outputs] = Rodrigues(a, q)
	outputs = eye(3) + matHat(a)*sin(q) + (1.0-cos(q))*matHat(a)*matHat(a);
end
