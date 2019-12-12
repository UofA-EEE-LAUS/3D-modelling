vrep=remApi('remoteApi');
vrep.simxFinish(-1); 

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('connected');
    [returnCode,bottom_joint]=vrep.simxGetObjectHandle(clientID,'bottom_joint',vrep.simx_opmode_oneshot_wait);
    [returnCode,joint1]=vrep.simxGetObjectHandle (clientID,'joint1',vrep.simx_opmode_oneshot_wait);
    [returnCode,joint2]=vrep.simxGetObjectHandle (clientID,'joint2',vrep.simx_opmode_oneshot_wait);
    [returnCode,joint3]=vrep.simxGetObjectHandle (clientID,'joint3',vrep.simx_opmode_oneshot_wait);
    
    for i = 0:100
    pause(0.5);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,bottom_joint,i,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition (clientID,joint1,i,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition (clientID,joint2,i,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition (clientID,joint3,i,vrep.simx_opmode_blocking);
    end 
     vrep.simxFinish(-1);
     
end
