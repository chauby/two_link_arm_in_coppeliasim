function [PVT_param] = PVTSpline2Points(PVT0, PVT1)
    p0 = PVT0(1);
    v0 = PVT0(2);
    t0 = PVT0(3);

    p1 = PVT1(1);
    v1 = PVT1(2);
    t1 = PVT1(3);

    T = t1-t0;
    d = p0;
    c = v0;
    b = (3*p1 - v1*T - 2*v0*T - 3*p0)/(T^2);
    a = (-2*p1 + v1*T + v0*T + 2*p0)/(T^3);
    PVT_param = [a, b, c, d];
end

