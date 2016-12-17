function [Pcorner] = corner(Pinit,Pfinal,time)
x_travel = Pfinal(1) - Pinit(1);
y_travel = Pfinal(2) - Pinit(2);
z_travel = Pfinal(3) - Pinit(3);
Pcorner = zeros(3,time);

for i = 1:1:time/2
    if z_travel > 0
        Pmove = Pinit + [0 0 z_travel*double(i)/double(time/2)];
        Pcorner(1:3,i) = Pmove;
    else
        Pmove = Pinit + [x_travel*double(i)/double(time/2) y_travel*double(i)/double(time/2) 0];
        Pcorner(1:3,i) = Pmove;
    end
end
Pinit = Pmove;
for i = time/2+1:1:time
    if z_travel > 0
        Pmove = Pinit + [x_travel*(double(i)-double(time/2))/(double(time)-double(time/2)) y_travel*(double(i)-double(time/2))/(double(time)-double(time/2)) 0];
        Pcorner(1:3,i) = Pmove;
    else
        Pmove = Pinit + [0 0 z_travel*(double(i)-double(time/2))/(double(time)-double(time/2))];
        Pcorner(1:3,i) = Pmove;
    end
end



