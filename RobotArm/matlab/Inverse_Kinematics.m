function r = arm()
    
    deg = pi/180;
    
    % robot length values (metres)
    a1 = 0;
    a2 = 0.08;
    a3 = 0.081;
    a4 = 0.172;
    a5 = 0;
    a6 = 0;
    d1 = 0;
    d2 = 0;
    d3 = 0;
    d4 = 0;
    d5 = 0;
    d6 = 0;
    
    % DH parameter table
    %     theta d a alpha
    dh = [0 d1 a1  -pi/2
          0 d2  a2  0
          0 d3  a3  0
          0 d4 a4   0
          0 d5  a5   -pi/2
          0 d6  a6   pi
          0  0   0   0]; %add 7th joint to make wrist spherical so inverse kinematics works
    tool = transl(0, 0, 0);
    
    
    % and build a serial link manipulator
    
%     robot = SerialLink(dh, 'name', 'Arm', ...
%         'manufacturer', 'ABB', 'ikine', 'nooffset'); 
robot = SerialLink(dh, 'name', 'Arm', 'manufacturer', 'ABB')
    robot.tool = tool;
    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'arm', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0 0]); % zero angles
        assignin('caller', 'qd', [0 -90 180 0 0 -90 0]*deg); % data sheet pose, horizontal
        assignin('caller', 'qr', [0 -90 90 0 90 -90 0]*deg); % ready pose, arm up
    end
end