vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('connected');
    [returnCode,crazyflie2_propeller_respondable1]=vrep.simxGetObjectHandle(clientID,'crazyflie2_propeller_respondable1',vrep.simx_opmode_oneshot_wait);
    [returnCode,crazyflie2_propeller_respondable2]=vrep.simxGetObjectHandle(clientID,'crazyflie2_propeller_respondable2',vrep.simx_opmode_oneshot_wait);
    [returnCode,crazyflie2_propeller_respondable3]=vrep.simxGetObjectHandle(clientID,'crazyflie2_propeller_respondable3',vrep.simx_opmode_oneshot_wait);
    [returnCode,crazyflie2_propeller_respondable4]=vrep.simxGetObjectHandle(clientID,'crazyflie2_propeller_respondable4',vrep.simx_opmode_oneshot_wait);
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,crazyflie2_propeller_respondable1,100,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,crazyflie2_propeller_respondable2,100,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,crazyflie2_propeller_respondable3,100,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,crazyflie2_propeller_respondable4,100,vrep.simx_opmode_oneshot_wait);
    
      vrep.simxFinish(-1);
     
end