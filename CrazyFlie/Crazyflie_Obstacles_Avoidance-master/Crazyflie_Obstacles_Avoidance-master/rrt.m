% Description of algorithm: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.
% 7. Add q_new to node list.
% 8. Continue until maximum number of nodes is reached or goal is hit.

% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.


function path = rrt(map,obs,eps,radius,nNodes,start,stop)
    x_max = map(1);
    y_max = map(2);
    obstacles = obs;
    EPS = eps;
    r = radius;
    numNodes = nNodes;

    %% Starting point
    q_start.coord = start;
    q_start.cost = 0;
    q_start.parent = 0;
    %% Arriving point
    q_goal.coord = stop;
    q_goal.cost = 0;

    %% Draw map
    nodes(1) = q_start;
    figure
    title('RRT MAP','FontSize',20);
    axis([0 x_max 0 y_max])

    %% Draw obstacles
    obstacle_size = size(obstacles);
    for i=1:obstacle_size(1)
        rectangle('Position',obstacles(i,:),'FaceColor',[.5 .5 .5])
        hold on
    end

    for i = 1:1:numNodes
        % Print number of nodes
        sprintf("Number of nodes: %d",length(nodes))
        % Generate randomly the new point
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        plot(q_rand(1), q_rand(2), '.', 'Color',  [0 0.4470 0.7410])

        % Break if goal node is already reached
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_goal.coord
                break
            end
        end

        % Pick the closest node from existing list to branch out from
        ndist = [];
        for j = 1:1:length(nodes)
            n = nodes(j);
            tmp = dist(n.coord, q_rand);
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_near = nodes(idx);

        % Steer towards q_near
        q_new.coord = steer(q_rand, q_near.coord, val, EPS);

        if (multiple_collision(q_rand, q_near.coord, obstacles))

            line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
            q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;

            % Within a radius of r, find all existing nodes
            q_nearest = [];
            neighbor_count = 1;
            for j = 1:1:length(nodes)
                if (dist(nodes(j).coord, q_new.coord) <= r ...
                    && multiple_collision(nodes(j).coord, q_new.coord, obstacles))

                    q_nearest(neighbor_count).coord = nodes(j).coord;
                    q_nearest(neighbor_count).cost = nodes(j).cost;
                    neighbor_count = neighbor_count+1;
                end
            end

            % Initialize cost to currently known value 
            q_min = q_near;
            C_min = q_new.cost;

            % Iterate through all nearest neighbors to find alternate lower
            % cost paths

            for k = 1:1:length(q_nearest)
                if (q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min...
                    && multiple_collision(q_nearest(k).coord, q_new.coord, obstacles))    

                    q_min = q_nearest(k);
                    C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                    line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                    hold on
                end
            end

            % Update parent to least cost-from node
            for j = 1:1:length(nodes)
                if nodes(j).coord == q_min.coord
                    q_new.parent = j;
                end
            end

            % Append to nodes
            nodes = [nodes q_new];
        end
    end

    D = [];
    for j = 1:1:length(nodes)
        tmpdist = dist(nodes(j).coord, q_goal.coord);
        D = [D tmpdist];
    end

    % Search backwards from goal to start to find the optimal least cost path
    [val, idx] = min(D);
    q_final = nodes(idx);
    q_goal.parent = idx;
    q_end = q_goal;
    nodes = [nodes q_goal];
    i = 1;
    while q_end.parent ~= 0
        start = q_end.parent;
        pos2(i,:) = [q_end.coord(1) q_end.coord(2)];
        i = i + 1;
        line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
        hold on
        q_end = nodes(start);
    end
    pos2(i,:) = q_start.coord;
    pos2 = flip(pos2);
    path = pos2;
end