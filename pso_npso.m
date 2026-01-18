function [gbest, gbest_val, convergence] = pso_npso(fobj, n_vars, lb, ub, n_pop, max_iter)
    w = 0.729; c1 = 1.49; c2 = 1.49;
    
    pos = zeros(n_pop, n_vars);
    for i=1:n_pop, pos(i,:) = lb + (ub-lb).*rand(1,n_vars); end
    vel = zeros(n_pop, n_vars);
    pbest = pos;
    pbest_val = inf(n_pop,1);
    
    for i=1:n_pop
        pbest_val(i) = fobj(pos(i,:));
    end
    [gbest_val, idx] = min(pbest_val);
    gbest = pbest(idx,:);
    convergence = zeros(1, max_iter);
    
    for t = 1:max_iter
        for i = 1:n_pop
            % --- TÌM lBest (LOCAL BEST) TRONG LÂN CẬN ---
            % Lân cận Ring: Trái (i-1), Giữa (i), Phải (i+1)
            idx_left = i - 1; if idx_left < 1, idx_left = n_pop; end
            idx_right = i + 1; if idx_right > n_pop, idx_right = 1; end
            
            neighbors = [idx_left, i, idx_right];
            [~, local_best_idx] = min(pbest_val(neighbors));
            lbest = pbest(neighbors(local_best_idx), :);
            % --------------------------------------------
            
            r1 = rand(1,n_vars); r2 = rand(1,n_vars);
            
            % Dùng lbest thay cho gbest
            vel(i,:) = w*vel(i,:) + c1*r1.*(pbest(i,:) - pos(i,:)) + c2*r2.*(lbest - pos(i,:));
            
            pos(i,:) = pos(i,:) + vel(i,:);
            pos(i,:) = max(pos(i,:), lb);
            pos(i,:) = min(pos(i,:), ub);
            
            fit = fobj(pos(i,:));
            if fit < pbest_val(i)
                pbest_val(i) = fit;
                pbest(i,:) = pos(i,:);
            end
            
            % Vẫn cập nhật gBest để theo dõi hội tụ toàn cục
            if pbest_val(i) < gbest_val
                gbest_val = pbest_val(i);
                gbest = pbest(i,:);
            end
        end
        convergence(t) = gbest_val;
    end
end