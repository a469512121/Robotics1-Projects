% This file requires functions include:
    % triangulation_test,m 
    % regionMatrix.m
    % eightpoint.m
    % point_searching.m
%% read image and assign parameters    
clear;
%Connects to the robot
clear;
robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');
cam = webcam(1);
cam.Contrast = 27;
cam.resolution = '1280x960'
preview(cam);
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


    
    
    % This file requires functions include:
    % triangulation_test,m 
    % regionMatrix.m
    % eightpoint.m
    % point_searching.m
%% read image and assign parameters    
clear;
Ia=imread('1.JPG');
Ib=imread('2.JPG');
Ia=rgb2gray(Ia);
Ib=rgb2gray(Ib);

intrinsic = [1440.4 0 639.5 0;0 1438.02 359.5 0;0 0 1 0 ];  
POC_1 = [0.5059; -0.5000; 7.8735];
R_1 =  [0.6252         0    0.7804;
         0    1.0000         0;
   -0.7804         0    0.6252];


POC_2 =[3.6238; -0.5000; 8.1415];
R_2 = [0.5490         0    0.8358;
         0    1.0000         0;
   -0.8358         0    0.5490];

camera_1 = intrinsic*[R_1 POC_1; [0 0 0 1]];
camera_2 = intrinsic*[R_2 POC_2; [0 0 0 1]];

%% select points in image 1 (selected_pt)

imtool(Ia);
x_coord = input('Input x-axis coordinate:');
y_coord = input('Input y-axis coordinate:');
imtool close all;
N = 4; % number of sample points in each axis
selected_pt = [x_coord y_coord]; %  a point in image1 with pixel coordinate and creates a cross around the point
for i = -N*4:4:N*4
    temp = [selected_pt(1,1)+i selected_pt(1,2)];
    selected_pt = [ selected_pt; temp];
end
for i = -N*4:4:N*4
    temp = [selected_pt(1,1) selected_pt(1,2)+i];
    selected_pt = [ selected_pt; temp];
end 
selected_pt = [selected_pt;[480 180];[496 483];[493 403]]; % inlude a reference point
%% select corresponding point in image 2 

% collect Interest Points from Each Image
blobs1 = detectSURFFeatures(Ia);
blobs2 = detectSURFFeatures(Ib);
[features1, validBlobs1] = extractFeatures(Ia, blobs1);
[features2, validBlobs2] = extractFeatures(Ib, blobs2);
indexPairs = matchFeatures(features1, features2);
matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);
for i=1:matchedPoints1.Count
    xa(i,:)=matchedPoints1.Location(i);
    ya(i,:)=matchedPoints1.Location(i,2);
    xb(i,:)=matchedPoints2.Location(i);
    yb(i,:)=matchedPoints2.Location(i,2);
end
pts1=[xa,ya];
pts2=[xb,yb];
width=size(Ia,2);
height=size(Ib,1);
[fLMedS,inliers] = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'NumTrials',4000);
pts1 = pts1(inliers,:);
pts2 = pts2(inliers,:);
inter = zeros(size(pts1,1),1);
for i=1:size(pts1,1)
    d(i,1) = norm(pts1(i,:) - pts2(i,:));
    if (d(i,1)<50)
        inter(i,1) = 1;
    end
end
inter = logical(inter);
pts1 = pts1(inter,:);
pts2 = pts2(inter,:);
F=eightpoint(pts1,pts2,width,height);
points_2 = zeros(size(selected_pt,1),2);
for i=1:size(selected_pt,1)
    points_2(i,:) = point_searching(Ia,Ib,F,selected_pt(i,1),selected_pt(i,2));
end


%% triangluation 
pts1 = selected_pt;
pts2 = points_2;
XP=[];
for i = 1:size(pts1,1)
    
    %XP(i,:) = roty(-65/180*pi)*triangulate_test(camera_1,t,camera_2,pts2(i,:))';
    XP(i,:) = triangulate_wen(pts1(i,:),pts2(i,:),intrinsic(:,1:3),R_1',R_2',POC_1,POC_2);
end 

x =[1*XP(2,1);1*XP(2,2); 1*XP(2,3)] - [1*XP(10,1);1*XP(10,2); 1*XP(10,3)];
y = [1*XP(19,1);1*XP(19,2); 1*XP(19,3)] - [1*XP(11,1);1*XP(11,2); 1*XP(11,3)];
cross_out = 3*cross(x,y);
x_axis = -0.5*(XP(size(XP,1),:)-XP(size(XP,1)-1,:));
CosTheta = dot(cross_out, x_axis)/(norm(cross_out)*norm(x_axis));
theta = acos(CosTheta)*180/pi
%% plot

%legend('points in 3D','normal vector','X axis');
%% kinematic
P0T = [point_3d(1)-cos(theta/180*pi)*4.5;0;point_3d(3)+sin(theta/180*pi)*4.5];
%Pc = [0.425;0.18;0.429];322
P0Tf = P0T;% + Pc;
q = [0 0 0];
[P0T1,R0T1]=fwdkinrecursion(1,eye(3),q,type,H,P,3);
P0Ti = P0T1;
%Travelling with the corner Function
Pp = corner(P0Ti',P0Tf',50);
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
    %q1 = q1 + os1;
    %q2 = q2 + os2;
    %q3 = q3 + os3;
    q4 = q3-q2+(theta/90*300)+500 + os4;
    new_pos = int16([q1;q2;q3;q4;500]);
    robot.setJointPositions(new_pos);
    pause(.1);
    q1o=q1;
    q2o=q2;
    q3o=q3;
    end
end
