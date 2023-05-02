% Load the bandwidth data from the CSV file
bandwidth = readmatrix('Bandwidth.csv', 'NumHeaderLines', 1);
nNodes = size(bandwidth, 1); % Number of nodes
nChannels = size(bandwidth, 2); % Number of channels

% Define the objective function to optimize the bandwidth allocation
objectiveFunction = @(x) sum((x.*bandwidth').^2);

% Set the algorithm parameters
nWhales = 10; % Number of whales
nIterations = 50; % Number of iterations

% Initialize the population of whales
population = rand(nWhales, nNodes); % Randomly assign bandwidth for nNodes nodes
best_whale = population(1,:); % Initialize the best whale found so far
best_fitness = objectiveFunction(best_whale); % Initialize the fitness of the best whale found so far

% Apply the WOA for nIterations iterations
for iter = 1:nIterations
    a = 2 - iter * (2 / nIterations); % Update parameter a
    % Update the position of each whale
    for i = 1:nWhales
        r1 = rand(); % Random number between 0 and 1
        r2 = rand(); % Random number between 0 and 1
        A = 2 * a * r1 - a; % Update parameter A
        C = 2 * r2; % Update parameter C
        p = rand(); % Random number between 0 and 1
        
        % Update the position using the search equation
        if p < 0.5
            % Use the search equation with exploitation phase
            D = abs(C * best_whale - population(i,:));
            new_position = best_whale - A * D;
        else
            % Use the search equation with exploration phase
            rand_whale_idx = randi(nWhales);
            rand_whale = population(rand_whale_idx,:);
            D = abs(C * rand_whale - population(i,:));
            new_position = rand_whale - A * D;
        end
        
        % Check if the new position is within the feasible range
        if min(new_position) >= 0 && max(new_position) <= 1
            population(i,:) = new_position;
        end
        
        % Update the best whale found so far
        fitness = objectiveFunction(population(i,:));
        if fitness < best_fitness
            best_whale = population(i,:);
            best_fitness = fitness;
        end
    end
end

% Get the best solution found
allocated_bandwidth = best_whale .* bandwidth';

% Format and display the allocated bandwidth
output_string = sprintf('Allocated bandwidth: %s\n', mat2str(allocated_bandwidth));
disp(output_string);

bar(allocated_bandwidth)
xlabel('Node')
ylabel('Allocated Bandwidth')
title('Allocated Bandwidth per Node and Channel')
