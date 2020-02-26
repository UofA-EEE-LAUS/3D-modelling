clear all;
coordinate_input=input('Enter Quadricopter final coordinates x y z as [x y z]: ');

    CIX=coordinate_input(:,1);
    CIY=coordinate_input(:,2);
    CIZ=coordinate_input(:,3);

    pParam=2;
    iParam=0;
    dParam=0;
    vParam=-2;

    cumul=0;
    lastE=0;
    pAlphaE=0;
    pBetaE=0;
    psp2=0;
    psp1=0;

vrep=remApi('remoteApi');
vrep.simxFinish(-1); 

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('connected');
    %read the coordinate of Quadricopter base
    
    [returnCode,Quadricopter_base]=vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_oneshot_wait);
   % [returnCode,ResizableFloor_5_25]=vrep.simxGetObjectHandle(clientID,'ResizableFloor_5_25',vrep.simx_opmode_oneshot_wait);
    
   
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Quadricopter_base,-1,vrep.simx_opmode_blocking);
    [returnCode,Matrix]=vrep.simxGetObjectMatrix(clientID,Quadricopter_base,-1,vrep.simx_opmode_blocking);
    
    
   posx=position(:,1);
   posy=position(:,2);
   posz=position(:,3);
   % Vertical control:
     while abs(CIX-posx)>=0.01 || abs(CIY-posy)>=0.01 || abs(CIZ-posz)>=0.01;
    
      vrep.simxFinish(-1);
     
end
