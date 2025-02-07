function [pop, loss] = ga_optimize(optim_func, first_indiv, hyperparams)
% Genetic algorithm to optimize parameters regarding an optimization
% function.
% Arguments :
%   - optim_func : the function to minimize. The function should take as
%   input individuals and return a score.
%   - first_indiv : first parameters of the population
%   - hyperparams cell containing the following hyperparameters : 
%       - population_size
%       - nb_epochs
%       - nb_selection
%       - params_min
%       - params_max
%       - mutation_param_prob : The probabiliy of that a parameter is
%       changed when an individual is mutated
%       - mutation_stds : std of the parameter modification during mutation for each dimension.
%       The mutation is gaussian
import controlOptim.mutate
import controlOptim.select
import controlOptim.rudder_control_loss

population_size = hyperparams{1};
nb_epochs = hyperparams{2};
nb_selection = hyperparams{3};
params_min = hyperparams{4};
params_max = hyperparams{5};
param_mutation_prob = hyperparams{6};
mutation_stds = hyperparams{7};
nb_params = length(first_indiv);

% parallel execution
num_workers = 7;
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', num_workers);
end

% Population initialization
pop = zeros(population_size, nb_params);
pop(1, 1:nb_params) = first_indiv;
for i=2:population_size
    pop(i, 1:nb_params) = mutate(first_indiv, params_min, params_max, ...
        param_mutation_prob, mutation_stds);
end

disp('WARNING ! Control the line 47 of ga_optimize.m to make sure the right function is optimized')
optim_func_handle = @(x) rudder_control_loss(x); % f**k matlab, optim_func here does not work in parfor

loss = zeros(1,population_size);
for i=1:nb_epochs
    disp(['epoch : ' num2str(i)])
    % evaluate
    parfor j=1:population_size
        warning('off')
        indiv = pop(j,:);
        loss(j) = optim_func_handle(indiv);
    end
    disp(['best loss : ' num2str(min(loss))])
    
    % Selection
    pop = select(pop, loss, nb_selection);
    
    % Mutation
    mutated = zeros(population_size-nb_selection, nb_params);
    for j=1:population_size-nb_selection
        mutated(j, 1:nb_params) = mutate(...
            pop(mod(j,nb_selection)+1, 1:nb_params),...
            params_min, params_max, param_mutation_prob, mutation_stds);
    end
    pop = [pop;mutated];
end

% Computing the loss for the last population
disp('final loss evaluation')
parfor j=1:population_size
    loss(j) = optim_func_handle(pop(j, :));
end

% Sorting the parameters by loss
[loss, idx] = sort(loss);
pop = pop(idx, 1:nb_params);

delete(gcp('nocreate'))
end

