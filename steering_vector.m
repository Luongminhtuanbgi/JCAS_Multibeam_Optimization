function a = steering_vector(M, theta, d_lambda)
    % Hàm tạo vector lái cho mảng ULA
    % Input:
    %   M: Số phần tử ăng-ten
    %   theta: Góc (độ) - có thể là 1 số hoặc 1 mảng
    %   d_lambda: Khoảng cách phần tử (chuẩn hóa theo bước sóng, thường là 0.5)
    
    % Chuyển vector phần tử về dạng cột [0; 1; ...; M-1]
    n = (0:M-1)';
    
 