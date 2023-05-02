% Load the bandwidth data from the CSV file
bandwidth = readmatrix('Bandwidth.csv', 'NumHeaderLines', 1);
nNodes = size(bandwidth, 1); % Number of nodes
nChannels = size(bandwidth, 2); % Number of channels

rng(123);

%% Whale Optimization Algorithm (WOA)

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

% Get the best solution found by WOA
allocated_bandwidth_woa = best_whale .* bandwidth';

%% Particle Swarm Optimization (PSO)

% Define the objective function to optimize the bandwidth allocation
objectiveFunction = @(x) sum((x.*bandwidth').^2);

% Set the algorithm parameters
nParticles = 10; % Number of particles
nIterations = 50; % Number of iterations
c1 = 2; % Coefficient of the cognitive component
c2 = 2; % Coefficient of the social component
w = 0.7; % Inertia weight

% Initialize the population of particles 
particles = rand(nParticles, nNodes); % Randomly assign bandwidth for nNodes nodes
velocities = zeros(nParticles, nNodes); % Initialize the velocities of the particles
best_particle = particles(1,:); % Initialize the best particle found so far
best_particle_fitness = objectiveFunction(best_particle); % Initialize the fitness of the best particle found so far

% Initialize the particle fitness
particle_fitness = zeros(nParticles, 1);

% Apply the PSO for nIterations iterations
for iter = 1:nIterations
    % Update the velocity and position of each particle
    for i = 1:nParticles
        % Update the velocity using the velocity update equation
        velocities(i,:) = w * velocities(i,:) ...
            + c1 * rand() * (best_particle - particles(i,:)) ...
            + c2 * rand() * (global_best_particle - particles(i,:));
        
        % Update the position using the position update equation
        particles(i,:) = particles(i,:) + velocities(i,:);
        
        % Check if the new position is within the feasible range
        if min(particles(i,:)) >= 0 && max(particles(i,:)) <= 1
            % Evaluate the fitness of the new position
            fitness = objectiveFunction(particles(i,:));
            
            % Update the personal best of the particle
            if fitness < particle_fitness(i)
                personal_best_particle = particles(i,:);
                particle_fitness(i) = fitness;
                
                % Update the global best particle
                if fitness < best_particle_fitness
                    best_particle = particles(i,:);
                    best_particle_fitness = fitness;
                end
            end
        end
    end
end

% Get the best solution found by PSO
allocated_bandwidth_pso = best_particle .* bandwidth';

%fprintf('Optimized objective function value: %.2f\n', best_fitness);

% Plot the results of the WOA and PSO algorithms
figure
plot(allocated_bandwidth_woa, '-o', 'LineWidth', 2)
hold on
plot(allocated_bandwidth_pso, '-*', 'LineWidth', 2)
hold off
legend('WOA', 'PSO')
xlabel('Node')
ylabel('Bandwidth')
title('Bandwidth allocation results')