clear all;

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

 [returnCode,Quadricopter_target]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_oneshot_wait);
 [returnCode,ResizableFloor_5_25]=vrep.simxGetObjectHandle(clientID,'ResizableFloor_5_25',vrep.simx_opmode_oneshot_wait);
 [returnCode,Quadricopter_target0]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target#0',vrep.simx_opmode_oneshot_wait);


if (clientID>-1)
    disp('connected');
    
    switch_move=input('Whether to perform displacement simlation input 1 or 0');
    
    if switch_move==1
    
    target_position=input('Enter Quadricopter final coordinates x y z as [x y z]: ');
    x=target_position(1,1);
    y=target_position(1,2);
    z=target_position(1,3);


  
   % [returnCode]=vrep.simxSetObjectPosition(clientID,Quadricopter_target,-1,[1,1,1],vrep.simx_opmode_blocking);
    %[returnCode]=vrep.simxSetObjectOrientation(clientID,Quadricopter_target,Quadricopter,[0,0,0.5],vrep.simx_opmode_blocking)
    %vrep.simxAddDrawingObject_points(number size, array color, array coords, string topic)
    
     [returnCode,position]=vrep.simxGetObjectPosition(clientID,Quadricopter_target,-1,vrep.simx_opmode_blocking);
     Posx=position(1,1);
     Posy=position(1,2);
     Posz=position(1,3);
    pause(2);
    for(i=1:36)
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,Quadricopter_target,-1,vrep.simx_opmode_blocking);
       
        Posx0=position0(1,1);
        Posy0=position0(1,2);
        Posz0=position0(1,3);
        
       [returnCode]=vrep.simxSetObjectPosition(clientID,Quadricopter_target,ResizableFloor_5_25,[Posx+(x-Posx)/36*i,Posy+(y-Posy)/36*i,Posz+(z-Posz)/36*i],vrep.simx_opmode_blocking);
       
       [returnCode,position1]=vrep.simxGetObjectPosition(clientID,Quadricopter_target0,-1,vrep.simx_opmode_blocking);
        Posx1=position1(1,1);
        Posy1=position1(1,2);
        Posz1=position1(1,3);
       
        r=sqrt((Posx0-Posx1)^2+(Posy0-Posy1)^2);
        
        [returnCode]=vrep.simxSetObjectPosition(clientID,Quadricopter_target0,ResizableFloor_5_25,[Posx0-r*cos(i*pi/18),Posy0-r*sin(i*pi/18),Posz0],vrep.simx_opmode_blocking);
        
        pause(0.4);
    end
    


    end
    
    
    vrep.simxFinish(-1);
     
end