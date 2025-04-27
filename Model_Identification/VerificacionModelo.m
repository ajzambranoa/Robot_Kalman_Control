clear; % clc;

%% Cargar parámetros optimizados (ya esto se hizo, y se obtuvieron
% los parametros Km, Ke, R, c_alpha e I_3) 
run('config_parametros.m');

%% Tiempo de simulación
T_total = 2000;           
T_s = 0.005;           % tiempo de muestreo (5 ms)
N = T_total / T_s;    
t = (0:N) * T_s;

% Estados iniciales [x; dx; theta; dtheta; phi; dphi]
x = zeros(6, N+1);
x(3,1) = deg2rad(45);
% Parámetros del robot
g = 9.81; d = 0.164; l = 0.116; r = 0.033;
m_B = 0.384; m_W = 0.2199;
J = 0.00012; K = 0.00006;
I_1 = 0.00516; I_2 = 0.00516;
% Parámetros obtenidos
Km      = 0.188;       
Ke      = 0.05;       
R       = 0.234;        
c_alpha = 0.125;        
I_3     = 0.0025; 
 
% PID del Arduino
kp_balance = 55;
kd_balance = 0.75;
kp_speed = 10;
ki_speed = 0.26;

% Variables del controlador
int_error_speed = 0;
vel_ref = 0;

% Almacenar señales de control
V_control_all = zeros(1, N);  

% Simulación paso a paso
for k = 1:N
    theta = wrapToPi(x(3,k));
    dtheta = x(4,k);
    vel = x(2,k);
    error_speed = vel - vel_ref;

    % Control velocidad PI
    int_error_speed = int_error_speed + error_speed;
    int_error_speed = max(min(int_error_speed, 3000), -3000);
    speed_control = -kp_speed * error_speed - ki_speed * int_error_speed;

    % Control balance PD
    balance_control = kp_balance * theta + kd_balance * dtheta;

    % Control total (sin giro)
    V_control = balance_control - speed_control;
    V_control = max(min(V_control, 8), -8);  % saturación

    
    V_control_all(k) = V_control;

    % Dinámica (Modelo escogido de un articulo)
    f1 = m_B + 2 * m_W + 2 * (J / r^2);
    f2 = m_B * l * cos(theta);
    f4 = I_2 + m_B * l^2;
    f5 = I_3 + 2 * K + (m_W + J / r^2)*(d^2 / 2) - (I_3 - I_1 - m_B*l^2)*(sin(theta))^2;

    fx1 = x(2,k);
    fx2 = m_B*l*(x(6,k)^2 + x(4,k)^2)*sin(theta) ...
        - (2/r)*c_alpha*(x(2,k)/r - x(4,k)) ...
        + 2*Km*V_control/(R*r) - (2*Km*Ke/(R*r^2))*x(2,k);

    fx3 = x(4,k);
    fx4 = -(I_3 - I_1 - m_B*l^2)*x(6,k)^2*sin(theta)*cos(theta) ...
        + m_B*l*g*sin(theta) ...
        + 2*c_alpha*(x(2,k)/r - x(4,k)) ...
        + (2*Km*Ke/(R*r))*x(2,k) - 2*Km*V_control/R;

    fx5 = x(6,k);
    fx6 = -(m_B*l*x(2,k) - 2*(I_3 - I_1 - m_B*l^2)*x(4,k)*cos(theta))*x(6,k)*sin(theta) ...
        - c_alpha*x(6,k)*d^2/(2*r^2) - Km*Ke*d^2/(2*R*r^2);

    F = [1, 0, 0, 0, 0, 0;
         0, f1, 0, f2, 0, 0;
         0, 0, 1, 0, 0, 0;
         0, f2, 0, f4, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, f5];

    fx = [fx1; fx2; fx3; fx4; fx5; fx6];

    dx = pinv(F) * fx;  

    x(:,k+1) = x(:,k) + T_s * dx;
end


theta = rad2deg(x(3,:));
vel = x(2,:);


if any(abs(theta) > 47)
    fprintf('El robot se cayó durante la simulación\n');
else
    fprintf('El robot se mantuvo en pie durante toda la simulación\n');
end

% Cargar datos desde el archivo CSV 
data = csvread('datos_tumbller.csv'); 
time_data = data(:, 1);   % tiempo
angle_data = data(:, 2);  % ángulo
vel_data = data(:, 3);    % velocidad
control_data = data(:, 4); % señal de control

% Parámetros para conversión de velocidad 
pulses_per_rev = 26;         % Pulsos por revolución del encoder
r = 0.033;                   % Radio de la rueda en metros
wheel_circumference = 2 * pi * r;  % Circunferencia de la rueda

% Conversión de speed_filter a m/s (pulsos cada 5ms a metros por segundo)
vel_data_mps = (vel_data) / pulses_per_rev * wheel_circumference;

min_len = min([length(time_data), length(t), length(V_control_all), length(control_data)]);

% Graficar todas las señales en una sola figura
figure('Name','Simulación Completa');
subplot(3,1,1);
plot(t(1:min_len), theta(1:min_len), 'LineWidth', 1.5); 
hold on;
plot(time_data(1:min_len), angle_data(1:min_len), 'r--', 'LineWidth', 1.5); 
legend('Ángulo Simulación', 'Ángulo CSV');
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
title('Comparación de Ángulo');
grid on;

subplot(3,1,2);
plot(t(1:min_len), vel(1:min_len), 'LineWidth', 1.5); 
hold on;
plot(time_data(1:min_len), vel_data_mps(1:min_len), 'r--', 'LineWidth', 1.5); 
legend('Velocidad Simulación', 'Velocidad CSV');
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
title('Comparación de Velocidad');
grid on;

subplot(3,1,3);
plot(t(1:min_len), V_control_all(1:min_len), 'LineWidth', 1.5); 
hold on;
plot(time_data(1:min_len), control_data(1:min_len) * 8 / 255, 'r--', 'LineWidth', 1.5); 
legend('Señal de Control Simulación', 'Señal de Control CSV');
xlabel('Tiempo [s]');
ylabel('Señal de Control');
title('Comparación de Señales de Control');
grid on;
