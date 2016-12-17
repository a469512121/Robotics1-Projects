%Connects to the robot
clear;
robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');
cam = webcam(1);
%Set initial Conditions
time = 100;
init_pos = robot.getJointPositions();
final_pos = int16( [500;300;300;500;450] ); %Target servo positions change to the final location.
new_pos = int16( [500;500;500;500;500] );
%Set robot dimensions and coordinates
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
n = 3;


%This loop is used to make the robot move to a desired point
while 1
    %{
    %Set robot to default
    for i=1:1:50
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/50;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    %Camera Calibration Initial Settings
    Servo_Pos = [500,500,500,500;500,262,321,500;...
        500,410,408,500;500,819,776,500;...
        500,500,500,500];
    time_c = [50,50,50,50,50];
    %Camera Calibration Code
    for j=1:1:4;
        final_pos = Servo_Pos(1:5,j);
        % used to find P0T and R0T of camera calibration spots
        init_pos = new_pos;
        q1 = pi/2*(Servo_Pos(1,j)-500)/300;
        q2 = pi/2*(Servo_Pos(2,j)-500)/300;
        q3 = pi/2*(Servo_Pos(3,j)-500)/300;
        q = [q1 q2 q3];
        [P0T1,R0T1]=fwdkinrecursion(1,eye(3),q,type,H,P,n);
        PTC = [4.5-.75;-.5;1.75];
        q4 = pi/2*(Servo_Pos(4,j)-500)/300;
        P0C = P0T1 + roty(q4)*PTC;
        R0C = R0T1*roty(q4);
        
        for i=1:1:time_c(j)
            for k=1:1:5
                new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time_c(j);
            end
            new_pos;
            robot.setJointPositions(new_pos);
            pause(.1);
        end
        init_pos = new_pos;
        if(j == 2)
            for i=1:1:10
            robot.setJointPositions(init_pos);
            pause(.1);
            end
            Ia = snapshot(cam);
            for i=1:1:10
            robot.setJointPositions(init_pos);
            pause(.1);
            end
        end
        if (j==3)
            for i=1:1:10
            robot.setJointPositions(init_pos);
            pause(.1);
            end
            Ib = snapshot(cam);
            for i=1:1:10
            robot.setJointPositions(init_pos);
            pause(.1);
            end
        end
        
    end
    %}
    %8.7896   -0.8488    1.7387
    P0T = [8.7896-cos(1/3*pi)*4.5;0;1.7387+sin(1/3*pi)*4.5];
    %Pc = [0.425;0.18;0.429];
    P0Tf = P0T% + Pc;
    q = [0 0 0];
    [P0T1,R0T1]=fwdkinrecursion(1,eye(3),q,type,H,P,n);
    P0Ti = P0T1;
    [q1,q2,q3] = inverseK(P0Tf,P,H);
    %Travelling with the corner Function
    Pp = corner(P0Ti',P0Tf',50)
    [m,n] = size(Pp);
    q1o=new_pos(1);
    q2o=new_pos(2);
    q3o=new_pos(3);
    for j=1:1:n
        [q1,q2,q3] = inverseK(Pp(1:3,j),P,H);
        [os1,os2,os3,os4] = offset(q2,q3);
        q1(isnan(q1)) = q1o;
        q2(isnan(q2)) = q2o;
        q3(isnan(q3)) = q3o;
        q1 = q1 + os1;
        q2 = q2 + os2;
        q3 = q3 + os3;
        q4 = q3-q2+700 + os4;
        new_pos = int16([q1;q2;q3;q4;500]);
        robot.setJointPositions(new_pos);
        pause(.1);
        q1o=q1;
        q2o=q2;
        q3o=q3;
    end
    pause(1);
    q=robot.getJointPositions();
    q1 = (double(q(1))-500)/300*pi/2;
    q2 = (double(q(2))-500)/300*pi/2;
    q3 = (double(q(3))-500)/300*pi/2;
    q = [q1,q2,q3];
    [P0T1,R0T1]=fwdkinrecursion(1,eye(3),q,type,H,P,3);
    pause(10);
    
    %{
    %Travelling with the Curve Function
    Pp = Curve_minus(P0Ti',P0Tf',50)
    [m,n] = size(Pp);
    q1o=new_pos(1);
    q2o=new_pos(2);
    q3o=new_pos(3);
    for j=1:1:n
        [q1,q2,q3] = inverseK(Pp(1:3,j),P,H);
        q1(isnan(q1)) = q1o;
        q2(isnan(q2)) = q2o;
        q3(isnan(q3)) = q3o;
        q4 = q3-q2+700;
        new_pos = int16([q1;q2;q3;q4;500]);
        robot.setJointPositions(new_pos);
        pause(.1);
        q1o=q1;
        q2o=q2;
        q3o=q3;
    end
    %}
    
    %{
    init_pos = new_pos;
    %Set for linear travel
    P0Tpush = P0Tf - [2*sin(1/6*pi);0;2*cos(1/6*pi)];
    %P0Ti = [5;0;5.5];
    [q1,q2,q3] = inverseK((P0Tpush),P,H);
    q4 = q3-q2+700;
    final_pos = int16([q1,q2,q3,q4,500]);
    for i=1:1:time
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    init_pos = new_pos;
    %Set for linear travel
    P0Tf;
    %P0Ti = [5;0;5.5];
    [q1,q2,q3] = inverseK((P0Tf),P,H);
    q4 = q3-q2+700;
    final_pos = int16([q1,q2,q3,q4,500]);
    for i=1:1:time
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    %}
    init_pos = new_pos;
    final_pos = int16( [500;500;500;500;500] );
    for i=1:1:50
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/50;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    %{
    %Set robot to default
    for i=1:1:time
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time;
        end
        new_pos
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    init_pos = new_pos;
    %}
    %{
    %Semi Circle Parameters
    P0Tf = [11;0;5.5];
    P0Ti = [5;0;5.5];
    [q1,q2,q3] = inverseK((P0Ti),P,H);
    q4 = q3-q2+800;
    %sets to initial position
    final_pos = int16([q1,q2,q3,q4,500]);
    for i=1:1:time
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    %Travelling with the Semi_Circle Function
    Pp = semi_circle(P0Ti',P0Tf',150);
    [m,n] = size(Pp);
    q1o=new_pos(1);
    q2o=new_pos(2);
    q3o=new_pos(3);
    for j=1:1:n
        [q1,q2,q3] = inverseK(Pp(1:3,j),P,H);
        q1(isnan(q1)) = q1o;
        q2(isnan(q2)) = q2o;
        q3(isnan(q3)) = q3o;
        q4 = q3-q2+800;
        new_pos = int16([q1;q2;q3;q4;500]);
        robot.setJointPositions(new_pos);
        pause(.1);
        q1o=q1;
        q2o=q2;
        q3o=q3;
    end
    
    P0Tf = [5;0;5.5];
    P0Ti = [11;0;5.5];
    %Travelling with the Semi_Circle Function
    Pp = semi_circle(P0Ti',P0Tf',150);
    [m,n] = size(Pp);
    q1o=new_pos(1);
    q2o=new_pos(2);
    q3o=new_pos(3);
    for j=1:1:n
        [q1,q2,q3] = inverseK(Pp(1:3,j),P,H);
        q1(isnan(q1)) = q1o;
        q2(isnan(q2)) = q2o;
        q3(isnan(q3)) = q3o;
        q4 = q3-q2+800;
        new_pos = int16([q1;q2;q3;q4;500]);
        robot.setJointPositions(new_pos);
        pause(.1);
        q1o=q1;
        q2o=q2;
        q3o=q3;
    end
    %}
    %{
    %Set robot to default
    for i=1:1:time
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    
    init_pos = new_pos;
    pause(.5);
    %Robot Arm Calibration Initial Settings
    armcal = 'y';
    x_offset = 0.5;
    y_offset = 0;
    z_offset = 0.5;
    %Robot Arm Calibration Code
    Parmcal = double([10+x_offset;0+y_offset;4.5+z_offset]);
    [q1,q2,q3] = inverseK(Parmcal,P,H);
    q4 = q3-q2+800;
    final_pos = int16( [q1;q2;q3;q4;500] )
    for i=1:1:time
        for k=1:1:5
            new_pos(k) = init_pos(k) + (final_pos(k)-init_pos(k))*i/time;
        end
        new_pos;
        robot.setJointPositions(new_pos);
        pause(.1);
    end
    %}
    %{
    %Travelling with the Curve Function
    Pp = Curve(P0Ti',P0Tf',50);
    [m,n] = size(Pp);
    q1o=new_pos(1);
    q2o=new_pos(2);
    q3o=new_pos(3);
    for j=1:1:n
        [q1,q2,q3] = inverseK(Pp(1:3,j),P,H);
        q1(isnan(q1)) = q1o;
        q2(isnan(q2)) = q2o;
        q3(isnan(q3)) = q3o;
        q4 = q3-q2+500;
        new_pos = int16([q1;q2;q3;q4;500]);
        %robot.setJointPositions(new_pos);
        pause(.1);
        q1o=q1;
        q2o=q2;
        q3o=q3;
    end
    %}
    
    break
    init_pos = new_pos;
end
