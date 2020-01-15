%MATLAB script for controlling the 3 omni-wheel rover platform

%------------------------------INSTRUCTIONS-------------------------------%
%{
1.  Ensure the files inside of
    C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab
    C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\lib\lib\Windows\64Bit
    are copied into the current MATLAB workspace

2.  In v-rep, ensure that the line:
    simRemoteApi.start(19999)
    is inside a non-threaded child script, under sysCall_init()
    (this runs once and starts the internal server for MATLAB to connect to)

3.  For more information on API functions, please see
    http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
%}

%---------------------------------SCRIPT----------------------------------%
clear all
close all
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    
    %------------------------------CODE HERE------------------------------%
    
    %defining motor handles
    %return code functions as a debug tool/error message
    [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,'motor_0',vrep.simx_opmode_oneshot_wait);
    [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,'motor_1',vrep.simx_opmode_oneshot_wait);
    [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,'motor_2',vrep.simx_opmode_oneshot_wait);
    
    %defining sensor handle
    [returnCode,laser_sensor]=vrep.simxGetObjectHandle(clientID,'laser_sensor',vrep.simx_opmode_oneshot_wait);
    
    %syntax
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,MOTOR_X,TARGET_V,vrep.simx_opmode_blocking);
    
    %setting motor speeds for fixed rotation (units in rad/s)
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,-1,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,-1,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,-1,vrep.simx_opmode_oneshot_wait);    
    
    %read laser sensor
    [returnCode]=vrep.simxReadProximitySensor(clientID,laser_sensor,vrep.simx_opmode_oneshot_wait);
    [returnCode,detectionState, detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,laser_sensor,vrep.simx_opmode_streaming); %start measuring
    
    %read orientation
    [returnCode] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_streaming);
    [returnCode, quaternions] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);


    
%User Defined Properties 
plotTitle = 'Laser Sensor';  % plot title
delay = .01;                     % make sure sample faster than resolution 
%Define Function Variables
count = 0;
%Set up Plot

theta = 0;
dist = 0;

figure
plotGraph = polarscatter(theta,dist)  % every AnalogRead needs to be on its own Plotgraph

thetalim([-180,180])

hold on                            %hold on makes sure all of the channels are plotted
tic
while ishandle(plotGraph) %Loop when Plot is Active will run until plot is closed
         [returnCode, detectionState, detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID, laser_sensor, vrep.simx_opmode_buffer);% measurement refresh
         [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
         dist_sample = norm(detectedPoint); %distance data from laser sensor
         
         %if dist_sample<0.001
         %   dist_sample = 4;
         %end
         
         if (orientations(1) >= 0)
            theta_sample = orientations(2);
         else
             if(orientations(1) < 0)
                 theta_sample = (pi/2-orientations(2))+pi/2;
             elseif(orientations(2) < 0)
                 theta_sample = (pi/2-orientations(2))-pi/2;
             end
         end
         
         count = count + 1;  
         
         theta(count) = theta_sample;    
         dist(count) = dist_sample;
         %This is the magic code 
         %Using plot will slow down the sampling time.. At times to over 20
         %seconds per sample!
         set(plotGraph,'XData',theta,'YData',dist);
         %Update the graph
         refreshdata;
         pause(0.01);
  end
    
    %destroy connection to v-rep simulation
    vrep.simxFinish(-1);
else
    disp('Failed connecting to remote API server');
end

vrep.delete()