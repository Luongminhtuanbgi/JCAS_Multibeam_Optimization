% Mô tả: Chương trình chính tối ưu hóa JCAS Multibeam
% So sánh: Bài toán gốc  vs PSO và các biến thể
% =========================================================================

clc; clear; close all;

%% 1. THIẾT LẬP HỆ THỐNG VÀ MỤC TIÊU
M = 16;               
d_lambda = 0.5;         
theta = -90:0.5:90;    

theta_comm = 0;      
theta_scan = -40;       

% Tạo vector lái
a_comm = steering_vector(M, theta_comm, d_lambda);
a_scan = steering_vector(M, theta_scan, d_lambda);

% Tạo MẪU BÚP SÓNG (Target Pattern) để so sánh
target_pattern = zeros(size(theta));
width = 5; 
for i = 1:length(theta)
    if abs(theta(i) - theta_comm) <= width || abs(theta(i) - theta_scan) <= width
        target_pattern(i) = 1; % Gain cao (0dB)
    else
        target_pattern(i) = 0.1; % Sidelobe thấp (-20dB)
    end
end

%% 2. TÍNH TOÁN "BÀI TOÁN GỐC" (BASELINE - KHÔNG TỐI ƯU)
% Giả sử chia đều công suất (rho=0.5) và không chỉnh pha (phi=0)
rho_fix = 0.5;
phi_fix = 0; 
x_fix = [rho_fix, phi_fix];

% Tính hàm thích nghi (Cost) cho trường hợp này
cost_func = @(x) fitness_function(x, a_comm, a_scan, M, d_lambda, theta, target_pattern);
cost_fix = cost_func(x_fix);

% Tạo búp sóng Fixed để vẽ
w_fix = sqrt(rho_fix)*a_comm + sqrt(1-rho_fix)*exp(1j*phi_fix)*a_scan;
w_fix = w_fix / norm(w_fix);
AF_fix = zeros(size(theta));
for i = 1:length(theta)
    AF_fix(i) = abs(w_fix' * steering_vector(M, theta(i), d_lambda));
end
AF_fix_dB = 20*log10(AF_fix + 1e-9);

%% 3. CHẠY CÁC THUẬT TOÁN PSO (TỐI ƯU)
n_pop = 40; max_iter = 100; n_var = 2;
lb = [0, -pi]; ub = [1,  pi];

fprintf('1. Chạy Classic PSO...\n');
[gbest_classic, fit_classic, curve_classic] = pso_classic(cost_func, n_var, lb, ub, n_pop, max_iter);

fprintf('2. Chạy LIDW-PSO...\n');
[gbest_lidw, fit_lidw, curve_lidw] = pso_ldiw(cost_func, n_var, lb, ub, n_pop, max_iter);

fprintf('3. Chạy X-PSO...\n');
[gbest_xpso, fit_xpso, curve_xpso] = pso_xpso(cost_func, n_var, lb, ub, n_pop, max_iter);

fprintf('4. Chạy N-PSO...\n');
[gbest_npso, fit_npso, curve_npso] = pso_npso(cost_func, n_var, lb, ub, n_pop, max_iter);
%% ========================================================================
%% 4. XỬ LÝ SỐ LIỆU ĐỂ VẼ (CHUẨN HOÁ & TÍNH TOÁN)
% =========================================================================

% --- A. Tái tạo trọng số ---
% 1. Baseline (Fixed)
w_fix = sqrt(rho_fix)*a_comm + sqrt(1-rho_fix)*exp(1j*phi_fix)*a_scan;
w_fix = w_fix / norm(w_fix); % Chuẩn hoá công suất vector

% 2. Tối ưu (Lấy từ X-PSO hoặc thuật toán tốt nhất)
best_sol = gbest_xpso; 
rho_opt = best_sol(1); 
phi_opt = best_sol(2);
w_opt = sqrt(rho_opt)*a_comm + sqrt(1-rho_opt)*exp(1j*phi_opt)*a_scan;
w_opt = w_opt / norm(w_opt); % Chuẩn hoá công suất vector

% --- B. Tính đáp ứng mảng (Array Factor) ---
AF_fix = zeros(size(theta));
AF_opt = zeros(size(theta));
for i = 1:length(theta)
    at = steering_vector(M, theta(i), d_lambda);
    AF_fix(i) = abs(w_fix' * at);
    AF_opt(i) = abs(w_opt' * at);
end

% --- C. CHUẨN HOÁ ĐỈNH VỀ 1 (0 dB) ---
AF_fix_norm = AF_fix / max(AF_fix);
AF_opt_norm = AF_opt / max(AF_opt);
target_norm = target_pattern / max(target_pattern);

% Chuyển sang dB (thêm 1e-6 để tránh log(0))
AF_fix_dB = 20*log10(AF_fix_norm + 1e-6);
AF_opt_dB = 20*log10(AF_opt_norm + 1e-6);
target_dB = 20*log10(target_norm + 1e-6);

min_dB = -60;
AF_fix_dB(AF_fix_dB < min_dB) = min_dB;
AF_opt_dB(AF_opt_dB < min_dB) = min_dB;
target_dB(target_dB < min_dB) = min_dB;

%% ========================================================================
%% 5. VẼ ĐỒ THỊ
% =========================================================================

%% --- HÌNH 1: ĐỒ THỊ HỘI TỤ 
figure(1);
semilogy(curve_classic, 'k-', 'LineWidth', 1.2); hold on;
semilogy(curve_lidw, 'b--', 'LineWidth', 1.2);
semilogy(curve_xpso, 'r-.', 'LineWidth', 1.2);
semilogy(curve_npso, 'g:', 'LineWidth', 2);

% Vẽ đường Baseline (Mức tham chiếu)
baseline_line = cost_fix * ones(1, max_iter);
semilogy(baseline_line, 'm-', 'LineWidth', 2);

grid on;
xlabel('Số vòng lặp (Iterations)'); 
ylabel('Giá trị Cost (Thang Log)');
title('Hình 1: Tốc độ hội tụ (So sánh với Baseline)');
legend('Classic PSO', 'LIDW-PSO', 'X-PSO', 'N-PSO', 'Baseline (Fixed)', 'Location', 'northeast');


% ylim([0.2705 0.275]); 

%% --- HÌNH 2: GIẢN ĐỒ BỨC XẠ  ---
figure(2);
plot(theta, target_dB, 'k--', 'LineWidth', 1.0); hold on;
plot(theta, AF_fix_dB, 'r:', 'LineWidth', 1.5);
plot(theta, AF_opt_dB, 'b-', 'LineWidth', 2);

grid on;
xline(theta_comm, 'k-', 'Alpha', 0.2);
xline(theta_scan, 'k-', 'Alpha', 0.2);

xlabel('Góc (Độ)'); 
ylabel('Biên độ chuẩn hóa (dB)');
title('Hình 2: Giản đồ bức xạ (Normalized Pattern)');

legend('Mẫu (Target)', ...
       'Baseline (Fixed)', ...
       sprintf('Tối ưu (PSO: \\rho=%.2f, \\phi=%.1f^o)', rho_opt, rad2deg(phi_opt)), ...
       'Location', 'southwest');

xlim([-90 90]);
ylim([-50 5]);

%% --- HÌNH 3: BIỂU ĐỒ CỘT SO SÁNH  ---
figure(3);
final_costs = [cost_fix, fit_classic, fit_lidw, fit_xpso, fit_npso];
algo_names = {'Baseline', 'Classic', 'LIDW', 'X-PSO', 'N-PSO'};

b = bar(final_costs);
b.FaceColor = 'flat';
b.CData(1,:) = [0.8 0.2 0.2]; 
b.CData(2:5,:) = repmat([0.2 0.4 0.8], 4, 1);

grid on;
ylabel('Cost (MSE - Càng nhỏ càng tốt)');
title('Hình 3: So sánh hiệu quả tối ưu hóa cuối cùng');

% Cài đặt nhãn trục X 
set(gca, 'XTick', 1:length(algo_names), 'XTickLabel', algo_names);

for i = 1:length(final_costs)
    text(i, final_costs(i), num2str(final_costs(i), '%.4f'), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 10, 'FontWeight', 'bold');
end

ylim([min(final_costs)*0.9, max(final_costs)*1.1]);