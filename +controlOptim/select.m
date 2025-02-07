function selected_pop = select(pop, loss, nb_selection)
% Function to make a selection from the population in a genetic algorithm.
% Arguments :
%       - population
%       - loss : the loss score of all the individuals
%       - nb_selection : number of individuals to select
% Returns :
%       - selected_pop : the selected individuals
selected_pop = zeros(nb_selection, size(pop, 2));
[~, idx] = sort(loss);
selected_pop(1, 1:end) = pop(idx(1), 1:end); % The best individual is always kept
disp(['Best individual : ' num2str(selected_pop(1,:))])
proba = 1./ (loss(2:end)+0.01);
proba = proba ./ sum(proba);
remaining_idx = 1:size(pop,1);
remaining_idx = remaining_idx(remaining_idx~=idx(1));
for i=2:nb_selection
    proba = proba / sum(proba);
    selected_idx = randsrc(1,1,[remaining_idx; proba]);
    selected_pop(i,1:end) = pop(selected_idx, 1:end);
    proba = proba(remaining_idx~=selected_idx);
    remaining_idx = remaining_idx(remaining_idx~=selected_idx);
end

