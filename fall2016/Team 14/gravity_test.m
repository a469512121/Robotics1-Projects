%gravity test
l0=4.5; 
l1=5.75; 
l1_1=1.5;
l2=5.75;
l3=4.5;
ex=[1;0;0];
ey=[0;1;0];
ez=[0;0;1];
zv=[0;0;0];
H = [-ez ey -ey];
P = [l0*ez zv l1*ez+l1_1*ex l2*ex];
type = [0 0 0];
q2 = -(500-700)/300*pi/2;
q3 = (500-700)/300*pi/2;
q4 = 0;

n = 3;
P03 = l0*ez + roty(q2)*(l1*ez+l1_1*ex);
P04 = P03 + roty(q2)*roty(q3)*(l2*ex);
[gain2,gain3] = gravitational_offset(P03,P04);
display(gain2);
display(gain3);