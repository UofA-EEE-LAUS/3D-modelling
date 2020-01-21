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
    
    %defining sensor handles
    [returnCode,laser_sensor]=vrep.simxGetObjectHandle(clientID,'laser_sensor',vrep.simx_opmode_oneshot_wait);
    [returnCode,gyro]=vrep.simxGetObjectHandle(clientID,'GyroSensor',vrep.simx_opmode_oneshot_wait);
    [returnCode,accelerometer]=vrep.simxGetObjectHandle(clientID,'Accelerometer',vrep.simx_opmode_oneshot_wait);
    
    %syntax
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,MOTOR_X,TARGET_V,vrep.simx_opmode_blocking);
    
    %setting motor speeds for straight line (units in rad/s)
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,1,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,1,vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,1,vrep.simx_opmode_oneshot_wait);    
    
    %read laser sensor
    [returnCode]=vrep.simxReadProximitySensor(clientID,laser_sensor,vrep.simx_opmode_oneshot_wait);
    [returnCode,detectionState, detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,laser_sensor,vrep.simx_opmode_streaming); %start measuring
    %destroy connection to v-rep simulation
    
    %User Defined Properties 
plotTitle = 'Laser Sensor';  % plot title
xLabel = 'Elapsed Time (s)';     % x-axis label
yLabel = 'Distance (m)';      % y-axis label
legend1 = 'Temperature Sensor 1'
%legend2 = 'Temperature Sensor 2'
%legend3 = 'Temperature Sensor 3'
yMax  = 5;                           %y Maximum Value
yMin  = 0;                       %y minimum Value
plotGrid = 'on';                 % 'off' to turn off grid
min = 0;                         % set y-min
max = 5;                        % set y-max
delay = .01;                     % make sure sample faster than resolution 
%Define Function Variables
time = 0;
data = 0;
%data1 = 0;
%data2 = 0;
count = 0;
%Set up Plot
plotGraph = plot(time,data,'-r' )  % every AnalogRead needs to be on its own Plotgraph
hold on                            %hold on makes sure all of the channels are plotted
%plotGraph1 = plot(time,data1,'-b')
%plotGraph2 = plot(time, data2,'-g' )
title(plotTitle,'FontSize',15);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
%legend(legend1,legend2,legend3)
axis([yMin yMax min max]);
grid(plotGrid);
tic
while ishandle(plotGraph) %Loop when Plot is Active will run until plot is closed
         [returnCode, detectionState, detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID, laser_sensor, vrep.simx_opmode_buffer);% measurement refresh
         dat = norm(detectedPoint); %Data from the arduino
         %dat1 = a.analogRead(2)* 0.48875855327; 
         %dat2 = a.analogRead(4)* 0.48875855327;       
         count = count + 1;    
         time(count) = toc;    
         data(count) = dat(1);         
         %data1(count) = dat1(1)
         %data2(count) = dat2(1)
         %This is the magic code 
         %Using plot will slow down the sampling time.. At times to over 20
         %seconds per sample!
         set(plotGraph,'XData',time,'YData',data);
         %set(plotGraph1,'XData',time,'YData',data1);
         %set(plotGraph2,'XData',time,'YData',data2);
          axis([0 time(count) min max]);
          %Update the graph
          pause(delay);
end
    
    
    vrep.simxFinish(-1);
else
    disp('Failed connecting to remote API server');
end

vrep.delete()