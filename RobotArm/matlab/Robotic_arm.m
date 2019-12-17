
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


i1=q(1,1);
i2=-q(1,2);
i3=-q(1,3);
i4=-q(1,4);


vrep=remApi('remoteApi');
vrep.simxFinish(-1); 

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('connected');
    [returnCode,bottom_joint]=vrep.simxGetObjectHandle(clientID,'bottom_joint',vrep.simx_opmode_oneshot_wait);
    [returnCode,joint1]=vrep.simxGetObjectHandle (clientID,'joint1',vrep.simx_opmode_oneshot_wait);
    [returnCode,joint2]=vrep.simxGetObjectHandle (clientID,'joint2',vrep.simx_opmode_oneshot_wait);
    [returnCode,joint3]=vrep.simxGetObjectHandle (clientID,'joint3',vrep.simx_opmode_oneshot_wait);
    
   
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,bottom_joint,i1,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition (clientID,joint1,i2,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition (clientID,joint2,i3,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition (clientID,joint3,i4,vrep.simx_opmode_blocking);
  
     vrep.simxFinish(-1);
     
end
