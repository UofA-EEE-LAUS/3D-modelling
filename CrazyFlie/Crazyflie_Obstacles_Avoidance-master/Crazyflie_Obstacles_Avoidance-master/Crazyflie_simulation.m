%% Init
clear all;
close all;
clc;

model_init;
%% New simulation?
simulation = input("Run new simulation? [1/0] \n")

if simulation == 1
    %% New trajectory?
    generate = input("Generate new trajectory? [1/0] \n")

    if generate == 1
        map_dimension = [2000 2000];

        obstacles = [[500,50,200,200];
                     [300,500,400,600];
                     [1800,1500,100,150];
                     [100,1000,150,200];
                     [1200,400,200,1000];
                     [600,1500,600,200];
                     [1450,800,350,200]];
        %eps = 15;
        eps = 40;
        %r = 60;
        r = 30;

        numNodes = 2000;
        
        
        % Draw map
        figure(1)
        axis([0 map_dimension(1) 0 map_dimension(2)])

        %% Draw obstacles
        obstacle_size = size(obstacles);
        for i=1:obstacle_size(1)
            rectangle('Position',obstacles(i,:),'FaceColor',[.5 .5 .5])
            hold on
        end
        
        % get start and stop points
        x_start = input('x starting coordinate [0 2000]: ');
        y_start = input('y starting coordinate [0 2000]: ');
        x_stop = input('x arriving coordinate [0 2000]: ');
        y_stop = input('y arriving coordinate [0 2000]: ');
        start_point = [x_start y_start];
        stop_point = [x_stop y_stop];
        %numNodes = input('Number of nodes: ');
        %start_point = [0 0];
        %stop_point = [0 1800];

        rrt_trajectory = rrt(map_dimension,obstacles,eps,r,numNodes,start_point,stop_point);
    else 
        % Load an example trajectory
        example_trajectory;
    end

    %% Model reference
    model = 'crazyflie';

    %% Intial conditions
    % Position
    x_ic = 0;
    y_ic = 0;
    z_ic = 0;
    % Angles
    phi_ic = 0;
    theta_ic = 0;
    psi_ic = 0;
    % Rotors
    sigma1_ic = 0;
    sigma2_ic = 0;
    sigma3_ic = 0;
    sigma4_ic = 0;

    %% Hovering
    z_r = 1;
    x_r = 0;
    y_r = 0;
    psi_r = 0;

    % Start simulation
    sim(model);

    %% Integrator refresh
    % Position
    x_ic = x(end);
    y_ic = y(end);
    z_ic = z(end);
    % Angles
    phi_ic = phi(end);
    theta_ic = theta(end);
    psi_ic = psi(end);
    % Rotor
    sigma1_ic = sigma_1(end);
    sigma2_ic = sigma_2(end);
    sigma3_ic = sigma_3(end);
    sigma4_ic = sigma_4(end);

    %% Trajectory tracking
    l_tr = length(rrt_trajectory(:,1));
    l_x = length(x);
    l_y = length(y);
    x_f = [];
    y_f = [];
    x_m = [];
    y_m = [];
    
    x_tot = zeros(l_x,l_tr);
    y_tot = zeros(l_y,l_tr);
    
    for i=1:l_tr

        % Get new target point (cm)
        x_r = 10*rrt_trajectory(i,1)/map_dimension(1);
        y_r = 10*rrt_trajectory(i,2)/map_dimension(2);

        % Starting simulation
        sim(model);

        % Integrator refresh
        % Position
        x_ic = x(end);
        y_ic = y(end);
        z_ic = z(end);
        % Angles
        phi_ic = phi(end);
        theta_ic = theta(end);
        psi_ic = psi(end);
        % Rotor
        sigma1_ic = sigma_1(end);
        sigma2_ic = sigma_2(end);
        sigma3_ic = sigma_3(end);
        sigma4_ic = sigma_4(end);

        % Saving the position reached x_f(i) = x(end)*map_dimension(1)/10;
        x_f(i) = x(end)*map_dimension(1)/10;
        y_f(i) = y(end)*map_dimension(2)/10;
        sprintf('Simulation n: %d of %d',i,l_tr)
        % Complete evolution plot
        figure(1);
        plot(x*map_dimension(1)/10,y*map_dimension(2)/10,'Color',[1 0 0],'LineWidth',2);hold on;
        axis([0 map_dimension(1) 0 map_dimension(2)])
   
    end
    % Save new worksapce
    save('work.mat');
else 
    % load old workspace
    load('work.mat');
end

%% Draw results
obstacles_size = size(obstacles);
figure(2);
hold on;
plot(rrt_trajectory(:,1),rrt_trajectory(:,2),'-','Color',[0 0 1],'LineWidth',1);hold on;
grid on;
%legend({'Desired trajectory','Drone trajectory (evolution)'},'FontSize',14);
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
title('Trajectory tracking with obstacle avoidance (hovering 1 meter)','FontSize',20);


figure(3),
for i=1:obstacles_size(1)
    rectangle('Position',obstacles(i,:),'FaceColor',[.5 .5 .5])
    hold on
end
plot(rrt_trajectory(:,1),rrt_trajectory(:,2),'-','Color',[0 0 1],'LineWidth',1);hold on
plot(x_f,y_f,'.','Color',[1 0 0],'LineWidth',2,'MarkerSize',10);hold on;grid on;
legend({'Desired trajectory','Drone trajectory end point'},'FontSize',14);
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
title('Trajectory tracking with obstacle avoidance (hovering 1 meter)','FontSize',20);
