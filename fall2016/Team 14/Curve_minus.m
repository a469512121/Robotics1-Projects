function [Pcurve] = Curve_minus(Pinit,Pfinal,speed)
D = Pfinal-Pinit;
Pcurve = zeros(3,speed);
s = speed - 1;
%display(D)

%curve second
if norm([D(1),D(2)]) <= norm(D(3))
    r = [0 0 D(3)*(norm([D(1),D(2)])/norm(D(3)))];
    Pc = Pinit + r;
    ti = int16(s-s*(norm(r)*pi/2)/((norm(r)*pi/2)+norm(Pc-[D(1),D(2),0]-Pinit))+1);
    T = 0;
    %display('2nd')
%curve first
else
    r = [[D(1) D(2)]*norm(D(3))/norm([D(1) D(2)],D(3)),0];
    Pc = Pinit + r;
    ti = int16(s*(norm(r)*pi/2)/((norm(r)*pi/2)+norm(Pc-[D(1),D(2),0]-Pinit))+1);
    T = 1;
    %display('1st')
end
%display(r)
%display(Pc)
%display(ti)

for i=1:1:ti
    if T == 0
        d_theta = pi*double(i)/(double(ti)*2.0);
        rv = [sin(d_theta)*norm(r)*([D(1),D(2)]/norm([D(1),D(2)])),cos(d_theta)*norm(r)];
        P = Pc + rv;
        Pcurve(1:3,i) = P;
        %display(d_theta)
    else
        P = Pinit + (((Pc-[norm(r)*([D(1),D(2)]/norm([D(1),D(2)])),0])-Pinit)*double(i)/double(ti));
        Pcurve(1:3,i) = P;
    end
end
for i=ti:1:speed
    if T == 0
        P = Pcurve(1:3,ti)' + ((Pfinal-Pcurve(1:3,ti)')*double(i-ti)/double(speed-ti));
        Pcurve(1:3,i) = P;
    else
        d_theta = pi*double(i-ti)/(double(speed-ti)*2.0);
        rv = [cos(d_theta)*norm(r)*(-[D(1),D(2)]/norm([D(1),D(2)])),sin(d_theta)*norm(r)];
        P = Pc + rv;
        Pcurve(1:3,i) = P;
    end
end