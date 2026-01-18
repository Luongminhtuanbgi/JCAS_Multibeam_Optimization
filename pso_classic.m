function [gbest, gbest_val, convergence] = pso_classic(fobj, n_vars, lb, ub, n_pop, max_iter)
    % THAM SỐ PSO CƠ BẢN
    w = 0.7;            % Hệ số quán tính cố định
    c1 = 2.0;           % Hệ số học cá nhân
    c2 = 2.0;           % Hệ số học xã hội
    
    % KHỞI TẠO
    pos = zeros(n_pop, n_vars);
    vel = zeros(n_pop, n_vars);
    
    % Sinh ngẫu nhiên vị trí ban đầu
    for i = 1:n_pop
        pos(i,:) = lb + (ub - lb) .* rand(1, n_vars);
    end
    
    fitness = zeros(n_pop, 1);
    pbest = pos;            % Khởi tạo pBest bằng vị trí đầu
    pbest_val = inf(n_pop, 1);
    gbest = zeros(1, n_vars);
    gbest_val = inf;
    
    convergence = zeros(1, max_iter); % Lưu lịch sử hội tụ
    
    % Đánh giá ban đầu
    for i = 1:n_pop
        fitness(i) = fobj(pos(i,:));
        pbest_val(i) = fitness(i);
        if pbest_val(i) < gbest_val
            gbest_val = pbest_val(i);
            gbest = pbest(i,:);
        end
    end
    
    % VÒNG LẶP CHÍNH
    for t = 1:max_iter
        for i = 1:n_pop
            % Cập nhật vận tốc
            r1 = rand(1, n_vars);
            r2 = rand(1, n_vars);
            vel(i,:) = w*vel(i,:) + c1*r1.*(pbest(i,:) - pos(i,:)) + c2*r2.*(gbest - pos(i,:));
            
            % Cập nhật vị trí
            pos(i,:) = pos(i,:) + vel(i,:);
            
            % Xử lý biên (Clamping)
            pos(i,:) = max(pos(i,:), lb);
            pos(i,:) = min(pos(i,:), ub);
            
            % Đánh giá Fitness
            fitness(i) = fobj(pos(i,:));
            
            % Cập nhật pBest
            if fitness(i) < pbest_val(i)
                pbest_val(i) = fitness(i);
                pbest(i,:) = pos(i,:);
            end
            
            % Cập nhật gBest
            if pbest_val(i) < gbest_val
                gbest_val = pbest_val(i);
                gbest = pbest(i,:);
            end
        end
        convergence(t) = gbest_val;
    end
end