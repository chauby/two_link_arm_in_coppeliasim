function PrintLinkName(j)
global uLINK
if j~=0
	fprintf('j=%d: %s\n',j,uLINK(j).name);
	PrintLinkName(uLINK(j).sister);
	PrintLinkName(uLINK(j).child);
end
end