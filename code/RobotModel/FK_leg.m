function FK_leg(j)
global uLINK
if j ==0
	return;
end
if j ~= 1
	i = uLINK(j).mother;
	uLINK(j).p = uLINK(i).R * uLINK(j).b + uLINK(i).p;
	uLINK(j).R = uLINK(i).R * Rodrigues(uLINK(j).a, uLINK(j).q);
end

FK_leg(uLINK(j).sister);
FK_leg(uLINK(j).child);

end