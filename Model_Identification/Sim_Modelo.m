clear;% clc;

% Tiempo de simulación
T_total = 11;           % segundos
T_s = 0.005;           % tiempo de muestreo (5 ms)
N = T_total / T_s;     % número de pasos
t = (0:N) * T_s;

% Estados iniciales [x; dx; theta; dtheta; phi; dphi]
x = zeros(6, N+1);
% x(3,1) = deg2rad(35);  % inclinación inicial no se cae
x(3,1) = deg2rad(45);
% Parámetros del robot
g = 9.81; d = 0.164; l = 0.116; r = 0.033;
m_B = 0.384; m_W = 0.2199;
J = 0.00012; K = 0.00006;
I_1 = 0.00516; I_2 = 0.00516;
% Parámetros variables
% Km      = 0.120;       % torque muy bajo
% Ke      = 0.1;       % fuerte frenado por back-emf
% R       = 2;        % más resistencia = menos corriente = menos torque
% c_alpha = 0.010;        % fricción viscosa alta (amortiguación fuerte)
% I_3     = 0.00150;     % inercia yaw más alta = menos giros bruscos
%Funciona
% Km      = 0.15;       % torque muy bajo
% Ke      = 0.10;       % fuerte frenado por back-emf
% R       = 2;        % más resistencia = menos corriente = menos torque
% c_alpha = 0.01;        % fricción viscosa alta (amortiguación fuerte)
% I_3     = 0.001;     % inercia yaw más alta = menos giros bruscos
% Oscilaciín 5 a -5
% Km      = 0.1;       % torque muy bajo
% Ke      = 0.10;       % fuerte frenado por back-emf
% R       = 1;        % más resistencia = menos corriente = menos torque
% c_alpha = 0.01;        % fricción viscosa alta (amortiguación fuerte)
% I_3     = 0.001;     % inercia yaw más alta = menos giros bruscos
% Km      = 0.18;       % torque muy bajo
% Ke      = 0.18;       % fuerte frenado por back-emf
% R       = 1;        % más resistencia = menos corriente = menos torque
% c_alpha = 0.008;        % fricción viscosa alta (amortiguación fuerte)
% I_3     = 0.0028; 

Km      = 0.196;       % torque muy bajo
Ke      = 0.05;       % fuerte frenado por back-emf
R       = 0.244;        % más resistencia = menos corriente = menos torque
c_alpha = 0.125;        % fricción viscosa alta (amortiguación fuerte)
I_3     = 0.001; 



% PID del Arduino
kp_balance = 55;
kd_balance = 0.75;
kp_speed = 10;
ki_speed = 0.26;

% Variables del controlador
int_error_speed = 0;
vel_ref = 0;

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

    % Dinámica
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

    dx = pinv(F) * fx;  % evita problemas de matriz singular
  % solución de ecuaciones

    % Integración por Euler
    x(:,k+1) = x(:,k) + T_s * dx;
end

% Extraer variables
theta = rad2deg(x(3,:));
vel = x(2,:);
pos = x(1,:);

% Ver si se cayó
if any(abs(theta) > 47)
    fprintf('❌ El robot se cayó durante la simulación\n');
else
    fprintf('✅ El robot se mantuvo en pie durante toda la simulación\n');
end

% Graficar
figure('Name','Simulación Discreta Estilo Arduino');
subplot(3,1,1); plot(t, theta); ylabel('\theta [°]'); grid on; title('Ángulo');
subplot(3,1,2); plot(t, vel); ylabel('Velocidad [m/s]'); grid on; title('Velocidad');
subplot(3,1,3); plot(t, pos); ylabel('Posición [m]'); xlabel('Tiempo [s]'); grid on; title('Desplazamiento');
