% Load the bandwidth data from the CSV file
bandwidth = readmatrix('Bandwidth.csv', 'NumHeaderLines', 1);
nNodes = size(bandwidth, 2); % Number of nodes
nChannels = size(bandwidth, 1); % Number of channels
bandwidth = bandwidth'; % Reshape to 10 by 25 matrix

% Set the seed value of the random number generator
rng(1);

% Define the objective function to optimize the bandwidth allocation
objectiveFunction = @(x) sum((x.*bandwidth').^2);

% Set the algorithm parameters
nParticles = 10; % Number of particles
nIterations = 50; % Number of iterations
c1 = 2; % Coefficient of the cognitive component
c2 = 2; % Coefficient of the social component
w = 0.7; % Inertia weight

% Initialize the population of particles
population = rand(nParticles, nNodes); % Randomly assign bandwidth for nNodes nodes
best_particle = population(1,:); % Initialize the best particle found so far
best_fitness = objectiveFunction(best_particle); % Initialize the fitness of the best particle found so far

% Initialize the velocity of each particle
velocity = zeros(nParticles, nNodes);

% Apply the PSO for nIterations iterations
for iter = 1:nIterations
    % Update the position and velocity of each particle
    for i = 1:nParticles
        % Update the velocity using the cognitive and social components
        r1 = rand(); % Random number between 0 and 1
        r2 = rand(); % Random number between 0 and 1
        cognitive_component = c1*r1*(best_particle - population(i,:));
        social_component = c2*r2*(best_particle - population(i,:));
        velocity(i,:) = w*velocity(i,:) + cognitive_component + social_component;
        
        % Update the position
        new_position = population(i,:) + velocity(i,:);
        
        % Check if the new position is within the feasible range
        if min(new_position) >= 0 && max(new_position) <= 1
            population(i,:) = new_position;
        end
        
        % Update the best particle found so far
        fitness = objectiveFunction(population(i,:));
        if fitness < best_fitness
            best_particle = population(i,:);
            best_fitness = fitness;
        end
    end
    
    % Update the inertia weight
    w = w - (0.7 - 0.4)/nIterations;
end

% Get the best solution found
allocated_bandwidth = best_particle .* bandwidth';

% Format and display the allocated bandwidth
output_string = sprintf('Allocated bandwidth: %s\n', mat2str(allocated_bandwidth));
disp(output_string);

bar(allocated_bandwidth')
xlabel('Channel')
ylabel('Allocated Bandwidth')
title('Allocated Bandwidth per Channel and Node')
