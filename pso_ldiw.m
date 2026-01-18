function [gbest, gbest_val, convergence] = pso_ldiw(fobj, n_vars, lb, ub, n_pop, max_iter)
    % THAM SỐ KHÁC BIỆT: W GIẢM DẦN
    w_max = 0.9;
    w_min = 0.4;
    c1 = 2.0; c2 = 2.0;
    
    % KHỞI TẠO (Giống Classic)
    pos = zeros(n_pop, n_vars);
    for i=1:n_pop, pos(i,:) = lb + (ub-lb).*rand(1,n_vars); end
    vel = zeros(n_pop, n_vars);
    pbest = pos;
    pbest_val = inf(n_pop,1);
    for i=1:n_pop
        fit = fobj(pos(i,:));
        pbest_val(i) = fit;
    end
    [gbest_val, idx] = min(pbest_val);
    gbest = pbest(idx,:);
    convergence = zeros(1, max_iter);
    
    % VÒNG LẶP
    for t = 1:max_iter
        % Cập nhật trọng số w tuyến tính
        w = w_max - ((w_max - w_min)/max_iter) * t;
        
        for i = 1:n_pop
            r1 = rand(1,n_vars); r2 = rand(1,n_vars);
            
            % Công thức vận tốc dùng w động
            vel(i,:) = w*vel(i,:) + c1*r1.*(pbest(i,:) - pos(i,:)) + c2*r2.*(gbest - pos(i,:));
            
            pos(i,:) = pos(i,:) + vel(i,:);
            pos(i,:) = max(pos(i,:), lb); % Chặn dưới
            pos(i,:) = min(pos(i,:), ub); % Chặn trên
            
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