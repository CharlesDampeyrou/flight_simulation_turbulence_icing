function indiv = mutate(original_indiv, params_min, params_max, ...
        param_mutation_prob, mutation_stds)
% Function to do a mutation of individuals for a genetic algorithm.
% Arguments :
%       - params_min
%       - params_max
%       - mutation_param_prob : The probabiliy of that a parameter is
%       changed when an individual is mutated
%       - mutation_stds : std of each parameter modification during mutation.
%       The mutation is gaussian
indiv = original_indiv;
for i=1:length(indiv)
    if rand()<param_mutation_prob
        indiv(i) = indiv(i) + mutation_stds(i)*randn();
        if indiv(i)<params_min
            indiv(i) = params_min;
        elseif indiv(i)>params_max
            indiv(i) = params_max;
        end
    end
end

