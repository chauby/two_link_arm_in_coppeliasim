function [mat] = matHat(vector)
	wx = vector(1);
	wy = vector(2);
	wz = vector(3);

	mat = zeros(3);
	mat(1,1) = 0.0;
	mat(1,2) = -wz;
	mat(1,3) = wy;

	mat(2,1) = wz;
	mat(2,2) = 0.0;
	mat(2,3) = -wx;

	mat(3,1) = -wy;
	mat(3,2) = wx;
	mat(3,3) = 0.0;
end