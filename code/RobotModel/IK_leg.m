function q=IK_leg(Body, D, A, B, Foot)
r=Foot.R'*(Body.p+Body.R*[0 D 0]' - Foot.p); % hip pose in foot coordination
C=norm(r);
c5=(C^2-A^2-B^2)/(2.0*A*B);

if c5 >= 1
    q5=0.0;
elseif c5 <= -1
    q5=pi;
else
    q5=acos(c5); % pitch angle for knee joint
end

q6a=asin((A/C)*sin(pi-q5));
q7=atan2(r(2), r(3));
if q7>pi/2
    q7=q7-pi;
elseif q7< -pi/2
    q7=q7+pi;
end

q6 = -atan2(r(1), sign(r(3))*sqrt(r(2)^2+r(3)^2)) - q6a; % pitch angle for ankle joint
R = Body.R'*Foot.R*rotX(-q7)*rotY(-q6 - q5); % hipZ*hipX*hipY

q2=atan2(-R(1,2), R(2,2)); % yaw angle for hip joint
cz=cos(q2);
sz=sin(q2);
q3=atan2(R(3,2), -R(1,2)*sz + R(2,2)*cz); % roll angle for hip joint
q4=atan2(-R(3,1), R(3,3)); % pitch angle for hip joint

% % Joint limitation for hip joint
% if q4 < -pi
%     q4 = -pi;
% elseif q4 > pi
%     q4 = pi;
% end

% % joint limitation for knee joint
% if q5 < 0
%     q5 = 0;
% elseif q5 > pi - 0.01
%     q5 = pi - 0.01;
% end

q=[q2 q3 q4 q5 q6 q7];

end
