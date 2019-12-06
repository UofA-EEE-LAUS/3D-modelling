vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('connected to v-rep');
    
    %code here
    
    vrep.simxFinish(-1);
    
end

vrep.delete()