% important values
% Tp,i denotes number of slots for packet transmission
% We have constant bit rate traffic which means that packets arrive from
% each connection in fixed duration. Suppose for a voice call every 20ms
% each connection is sending a packet to the wireline network. 

% TSI is the time of one scheduling interval where all the scheduling
% decisions are taken. We assume that one packet is arriving in each 
% scheduling time TSI fromevery connection. 
% So suppose we have connections that are present at the start of
% scheduling interval, then a number of slots are present for each
% scheduling interval and using these slots we must transmit the packet to
% the Root AP. 
% Am,t = 0 represent that the AP does not transmit a packet in slot m. 

function path = shortest_path(adj_matrix, src, dst)
    graph = digraph(adj_matrix);
    path = shortestpath(graph, src, dst);
end

function available_time = find_next_available_slot(AP_slots, AP, t_si, total_slots, is_bottleneck, start_time, direction)
    % Find all occupied slots for transmitting AP
    scheduled_slots = find(AP_slots(AP, :) == 0); 
    if isempty(scheduled_slots)
        if is_bottleneck
            T_min = t_si;
            T_max = total_slots - t_si;
        else
            T_min = 1;
            T_max = total_slots;
        end
    else
        t_m_max = max(scheduled_slots);
        t_m_min = min(scheduled_slots);
        T_min = max(1, t_m_max - t_si);
        T_max = min(total_slots, t_m_min + t_si);
    end

    if nargin < 6
        start_time = 1; % Default start time
    end
    if nargin < 7
        direction = 'forward'; % Default direction
    end

    if is_bottleneck == true
        available_time = -1;
        for t = int32((T_min + T_max) / 2):min(total_slots, T_max)
            if AP_slots(AP, t) == 1
                available_time = t;
                break
            end
        end
        for t = int32((T_min + T_max) / 2):-1:max(1, T_min)
            if AP_slots(AP, t) == 1 & available_time == -1
                available_time = t;
                break;
            end
        end
        if AP_slots(AP, int32(T_max + T_min) / 2) == 1
          available_time = int32(T_max + T_min) / 2;
        end
        return
    end
                
    % Next available slot
    if strcmp(direction, 'forward')
        for t = max(start_time, T_min):min(total_slots, T_max)
            if AP_slots(AP, t) == 1
                available_time = t;
                return;
            end
        end
    elseif strcmp(direction, 'backward')
        for t = min(start_time, T_max):-1:max(1, T_min)
            if AP_slots(AP, t) == 1
                available_time = t;
                return;
            end
        end
    end

    available_time = -1; % connection is blocked if can't be scheduled
end


function [success,delay] = try_schedule_connections(connections, num_APs, adj_matrix)
    t_si = 15;
    total_slots = 15;
    AP_slots = ones(num_APs, total_slots);
    success = true;
    delay = 0;

    % Calculate shortest paths for all the connnections
    root_AP = 1;
    paths = cell(length(connections), 1);
    for i = 1:length(connections)
        paths{i} = shortest_path(adj_matrix, connections(i), root_AP);
    end

    % we calculate load on each for claculating bottleneck
    AP_load = zeros(1, num_APs);
    for i = 1:length(paths)
        for j = 1:length(paths{i})
            AP_load(paths{i}(j)) = AP_load(paths{i}(j)) + 1;
        end
    end

    % sort all path by lengh
    [~, sorted_indices] = sort(cellfun(@length, paths), 'descend');
    connections = connections(sorted_indices);
    paths = paths(sorted_indices);

    for i = 1:length(paths)
        path = paths{i};
        % Index of bottleneck in the path
        slot_min = 100;
        slot_max = 0;

        [~, bottleneck_idx] = max(AP_load(path)); 
        bottleneck_AP = path(bottleneck_idx);

        % bottleneck hop case 2 pseudocode 2. 
        bottleneck_time = find_next_available_slot(AP_slots, bottleneck_AP, t_si, total_slots, true);
        if bottleneck_time == -1
            success = false;
            return;
        end
        AP_slots(bottleneck_AP, bottleneck_time) = 0;
        slot_min = min(slot_min, bottleneck_time);
        slot_max = max(slot_max, bottleneck_time);

        % we schedule all downstream
        current_time = bottleneck_time;
        for j = bottleneck_idx+1:length(path)-1
            tx_AP = path(j);
            rx_AP = path(j + 1);
            available_time = find_next_available_slot(AP_slots, tx_AP, t_si, total_slots, false, current_time+1);
            if available_time == -1
                success = false;
                return;
            end
            AP_slots(rx_AP, available_time) = 0;
            slot_min = min(slot_min, available_time);
            slot_max = max(slot_max, available_time);
            current_time = available_time;
        end
            
        current_time = bottleneck_time;
        % now schedule all upstream
        for j = bottleneck_idx-1:-1:1
            tx_AP = path(j);
            rx_AP = path(j + 1);
            available_time = find_next_available_slot(AP_slots, tx_AP, t_si, total_slots, false, current_time-1, 'backward');
            if available_time == -1
                success = false;
                return;
            end
            AP_slots(tx_AP, available_time) = 0;
            slot_min = min(slot_min, available_time);
            slot_max = max(slot_max, available_time);
            current_time = available_time; 
        end
        delay = delay + slot_max - slot_min;
    
    end
end


%% Simulation Part where we simulate traffic loads on the connections
function result = do_simulation(adj_matrix, traffic_loads, num_simulations, num_APs)
    blocking_rates = zeros(size(traffic_loads)); % To store blokc count
    average_delay = zeros(size(traffic_loads));
    for idx = 1:length(traffic_loads)
        traffic_load = traffic_loads(idx);
        rejected_connections = 0;
        total_connections = 0;   
        call_duration = 60;
        total_delay = 0;
        
        for sim = 1:num_simulations
            num_requests = ceil(traffic_load * call_duration);
            requested_connections = randi([2, num_APs], num_requests, 1);
            current_connections = [];
            for con = 1:num_requests
                total_connections = total_connections + 1;
                [success, delay] = try_schedule_connections([current_connections; requested_connections(con)], num_APs, adj_matrix);
                if success
                    current_connections = [current_connections; requested_connections(con)];
                    total_delay = total_delay + delay;
                else 
                    rejected_connections = rejected_connections + 1;
                end 
            end 
        end
        average_delay(idx) = total_delay / (total_connections - rejected_connections);
        blocking_rates(idx) = rejected_connections / total_connections;
    end
    
    % Plot results
    figure;
    plot(traffic_loads, blocking_rates, '-o', 'LineWidth', 1);
    xlabel('Traffic Load');
    ylabel('Connection Blocking Rate');
    title('Blocking Rate vs. Traffic Load');
    grid on;

    figure;
    plot(traffic_loads, average_delay, '-o', 'LineWidth', 2);
    xlabel('Traffic Load');
    ylabel('Average Packet Transmission Delay');
    title('Average Transmission Delay');
    grid on;
end

%% Running Simulation For different topologies
traffic_loads = 0.2:0.1:1.4; 
packet_arrival_time = 20;       
t_si = 20;
total_slots = 40;

%% 3 x 3 Grid Topology
num_APs = 9;  
adj_matrix = [0 1 0 1 0 0 0 0 0;
              1 0 1 0 1 0 0 0 0;
              0 1 0 0 0 1 0 0 0;
              1 0 0 0 1 0 1 0 0;
              0 1 0 1 0 1 0 1 0;
              0 0 1 0 1 0 0 0 1;
              0 0 0 1 0 0 0 1 0;
              0 0 0 0 1 0 1 0 1;
              0 0 0 0 0 1 0 1 0];
 num_simulations = 1;
do_simulation(adj_matrix, traffic_loads, num_simulations, num_APs);

%% 1 + 2 * 4 Topology
num_APs = 9;  % Adjust the number of APs for this topology
adj_matrix = [0 1 1 0 0 0 0 0 0;
              1 0 0 1 1 0 0 0 0;
              1 0 0 0 0 1 1 0 0;
              0 1 0 0 0 0 0 1 0;
              0 1 0 0 0 0 0 0 1;
              0 0 1 0 0 0 0 1 0;
              0 0 1 0 0 0 0 0 1;
              0 0 0 1 0 1 0 0 0;
              0 0 0 0 1 0 1 0 0];
% num_simulations = 100;
% do_simulation(adj_matrix, traffic_loads, num_simulations, num_APs);

%% 1 + 4 * 2 Topology
num_APs = 9;  % Adjust the number of APs for this topology
adj_matrix = [0 1 1 0 0 0 0 0 0;
              1 0 0 1 1 0 0 0 0;
              1 0 0 0 0 1 1 0 0;
              0 1 0 0 1 0 0 1 0;
              0 1 0 1 0 1 0 0 1;
              0 0 1 0 1 0 1 0 0;
              0 0 1 0 0 1 0 0 0;
              0 0 0 1 0 0 0 0 0;
              0 0 0 0 1 0 0 0 0];
num_simulations = 1000;
% do_simulation(adj_matrix, traffic_loads, num_simulations, num_APs);

%% Random 20 x 20 Topology
num_APs = 20;  % Number of APs for this topology
adj_matrix = zeros(num_APs);  % Initialize the adjacency matrix

% Create random connections with a density factor
density = 0.2;  % Adjust density for sparsity
for i = 1:num_APs
    for j = i+1:num_APs
        if rand < density
            adj_matrix(i, j) = 1;
            adj_matrix(j, i) = 1;
        end
    end
end
num_simulations = 100;
% do_simulation(adj_matrix, traffic_loads, num_simulations, num_APs);






