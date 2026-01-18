function [gbest, gbest_val, convergence] = pso_xpso(fobj, n_vars, lb, ub, n_pop, max_iter)
    % THIẾT LẬP HỆ SỐ NÉN CHI
    phi1 = 2.05;
    phi2 = 2.05;
    phi = phi1 + phi2; % = 4.1
    
    % Tính Chi
    chi = 2 / abs(2 - phi - sqrt(phi^2 - 4*phi)); % ~ 0.729
    
    % KHỞI TẠO (Tương tự)
    pos = zeros(n_pop, n_vars);
    for i=1:n_pop, pos(i,:) = lb + (ub-lb).*rand(1,n_vars); end
    vel = zeros(n_pop, n_vars);
    pbest = pos;
    pbest_val = inf(n_pop,1);
    
    % Đánh giá đầu tiên
    for i=1:n_pop
        pbest_val(i) = fobj(pos(i,:));
    end
    [gbest_val, idx] = min(pbest_val);
    gbest = pbest(idx,:);
    convergence = zeros(1, max_iter);
    
    for t = 1:max_iter
        for i = 1:n_pop
            r1 = rand(1,n_vars); r2 = rand(1,n_vars);
            
            % Cập nhật vận tốc dùng Chi (Bỏ w)
            vel(i,:) = chi * (vel(i,:) + phi1*r1.*(pbest(i,:) - pos(i,:)) + phi2*r2.*(gbest - pos(i,:)));
            
            pos(i,:) = pos(i,:) + vel(i,:);
            pos(i,:) = max(pos(i,:), lb);
            pos(i,:) = min(pos(i,:), ub);
            
            fit = fobj(pos(i,:));
            if fit < pbest_val(i)
                pbest_val(i) = fit;
                pbest(i,:) = pos(i,:);
            end
            if pbest_val(i) < gbest_val
                gbest_val = pbest_val(i);
                gbest = pbest(i,:);
            end
        end
        convergence(t) = gbest_val;
    end
end