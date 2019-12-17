%as of 12/12/2019 this program only achieves a desired x, y, z of our end
%effector when fully extended (gripper is closed). It does not take into 
%account roll, pitch and yaw as of yet.


Inverse_Kinematics;
arm.plot(qz) %plot arm in resting postion with all motor angles set to 0
coord = input('Enter desired x y z coordinates as [x y z]: ');
disp('The motor angles are: ');

%translation matrix for end effector
T = transl(coord(1), coord(2), coord(3)) * rpy2tr(0, 0, 0, 'deg');

%q is a vector of motor angles corresponding to, first angle is for the 
%first motor, second angle is for the second motor etc. make note that
%there are 7 angles listed when there are only 6 motors on our arm, this is
%because to get the the inverse kinematic equation working witht he tool 
%box we are using, the wrist must be sperical which means having 3 axis of 
%rotation at the end effector. We also will ignore the last three angles
%as this program does not account for roll, pitch and yaw yet. 
q = arm.ikine(T);  
disp(q);
arm.plot(q);