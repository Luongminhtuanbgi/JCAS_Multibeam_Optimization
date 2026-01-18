function cost = fitness_function(x, a_comm, a_scan, M, d_lambda, theta_range, target_pattern)
    % Hàm tính giá trị thích nghi (Cost Function)
    % Input:
    %   x: Vector vị trí của hạt [rho, phi]
    %   Các tham số khác: Cấu hình hệ thống truyền từ Main
    
    rho = x(1);
    phi = x(2);
    
    % 1. Tái tạo vector trọng số w từ rho và phi (Công thức ghép búp tối ưu)
    % w = sqrt(rho)*w_c + sqrt(1-rho)*exp(j*phi)*w_s
    w_t = sqrt(rho)*a_comm + sqrt(1-rho)*exp(1j*phi)*a_scan;
    
    % 2. Chuẩn hóa công suất
    w_t = w_t / norm(w_t);
    
    % 3. Tính đáp ứng mảng thực tế trên toàn dải góc theta_range
    % Array Factor = |w^H * a(theta)|
    % Để tính nhanh cho nhiều góc, ta gọi steering_vector cho cả dải
    A_matrix = steering_vector(M, theta_range, d_lambda); 
    
    % Tính biên độ (Magnitude)
    response_actual = abs(w_t' * A_matrix);
    
    % 4. Tính sai số khớp mẫu (MSE - Mean Squared Error)
    % J = mean( (Actual - Target)^2 )
    diff = response_actual - target_pattern;
    cost = mean(diff.^2);
    