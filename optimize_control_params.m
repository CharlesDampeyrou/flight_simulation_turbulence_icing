import controlOptim.rudder_control_loss
import controlOptim.ga_optimize

%optim_func = @(x) (x(1)^2+x(2)-11)^2+(x(1)+x(2)^2-7)^2;
%initial_indiv = [0,0];

optim_func = @(x) rudder_control_loss(x);
initial_indiv=[288055, 138267, 9628];
std = [20000, 10000, 1000];

hyperparams = {
    14 % pop_size
    25 % epochs
    3 % nb_selection
    0 % param_min
    1000000 % param max
    0.8 % mutation param proba
    std % mutation std
};

initial_loss = optim_func(initial_indiv);
disp(['Loss of the initial parameters : ' num2str(initial_loss)]);
[pop, loss] = ga_optimize(optim_func, initial_indiv, hyperparams);

disp([pop, loss'])