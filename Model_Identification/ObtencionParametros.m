clear; clc;

Km_values    = 0.18:0.01:0.20;
Ke_values    = 0.04:0.01:0.06;
R_values     = 0.23:0.01:0.24;
c_values     = 0.12:0.01:0.13;
I3_values    = 0.002:0.0005:0.003;

T_s = 0.005;
T_total = 5;
N = round(T_total / T_s);
t = (0:N) * T_s;
x0 = zeros(6,1);
x0(3) = deg2rad(45);

kp_balance = 55;
kd_balance = 0.75;
kp_speed = 10;
ki_speed = 0.26;

resultados_ok = [];
total = numel(Km_values)*numel(Ke_values)*numel(R_values)*numel(c_values)*numel(I3_values);

fprintf("Explorando %d combinaciones...\n", total);

% Crear todas las combinaciones de los parámetros usando ndgrid
[Km_grid, Ke_grid, R_grid, c_grid, I3_grid] = ndgrid(Km_values, Ke_values, R_values, c_values, I3_values);
param_combinations = [Km_grid(:), Ke_grid(:), R_grid(:), c_grid(:), I3_grid(:)];

% Utilizar procesamiento paralelo si se puede
parfor k = 1:size(param_combinations, 1)
    Km = param_combinations(k, 1);
    Ke = param_combinations(k, 2);
    R = param_combinations(k, 3);
    c_alpha = param_combinations(k, 4);
    I3 = param_combinations(k, 5);

    x = zeros(6, N+1);
    x(3,1) = x0(3);
    int_error = 0;
    estable = true;

    amplitudes = [];
    oscillations = 0;
    theta_deg = [];

    for i = 1:N
        theta = wrapToPi(x(3,i));
        dtheta = x(4,i);
        vel = x(2,i);
        error_speed = vel;
        int_error = int_error + error_speed;
        int_error = max(min(int_error, 3000), -3000);

        speed_control = -kp_speed * error_speed - ki_speed * int_error;
        balance_control = kp_balance * theta + kd_balance * dtheta;
        V = balance_control - speed_control;
        V = max(min(V, 8), -8);

        g = 9.81; d = 0.164; l = 0.116; r = 0.033;
        m_B = 0.384; m_W = 0.2199;
        J = 0.00012; K = 0.00006;
        I1 = 0.00516; I2 = 0.00516;

        f1 = m_B + 2*m_W + 2*(J/r^2);
        f2 = m_B * l * cos(theta);
        f4 = I2 + m_B*l^2;
        f5 = I3 + 2*K + (m_W + J/r^2)*(d^2/2) - (I3 - I1 - m_B*l^2)*sin(theta)^2;

        fx1 = x(2,i);
        fx2 = m_B*l*(x(6,i)^2 + x(4,i)^2)*sin(theta) ...
            - (2/r)*c_alpha*(x(2,i)/r - x(4,i)) ...
            + 2*Km*V/(R*r) - (2*Km*Ke/(R*r^2))*x(2,i);
        fx3 = x(4,i);
        fx4 = -(I3 - I1 - m_B*l^2)*x(6,i)^2*sin(theta)*cos(theta) ...
            + m_B*l*g*sin(theta) + 2*c_alpha*(x(2,i)/r - x(4,i)) ...
            + (2*Km*Ke/(R*r))*x(2,i) - 2*Km*V/R;
        fx5 = x(6,i);
        fx6 = -(m_B*l*x(2,i) - 2*(I3 - I1 - m_B*l^2)*x(4,i)*cos(theta))*x(6,i)*sin(theta) ...
            - c_alpha*x(6,i)*d^2/(2*r^2) - Km*Ke*d^2/(2*R*r^2);

        F = [1,0,0,0,0,0;
            0,f1,0,f2,0,0;
            0,0,1,0,0,0;
            0,f2,0,f4,0,0;
            0,0,0,0,1,0;
            0,0,0,0,0,f5];
        fx = [fx1; fx2; fx3; fx4; fx5; fx6];

        dx = pinv(F) * fx;
        x(:,i+1) = x(:,i) + T_s * dx;

        if abs(rad2deg(x(3,i+1))) > 80
            estable = false;
            break;
        end

        theta_deg = [theta_deg, rad2deg(x(3,i+1))];
    end

    if estable
        oscillations = 0;
        amplitudes = [];

        for i = 2:length(theta_deg)
            if (theta_deg(i-1) < 0 && theta_deg(i) > 0) || (theta_deg(i-1) > 0 && theta_deg(i) < 0)
                oscillations = oscillations + 1;
                amplitudes = [amplitudes, max(abs(theta_deg(i-1)), abs(theta_deg(i)))];
            end
        end

        average_amplitude = mean(amplitudes);

        resultados_ok(k,:) = [Km, Ke, R, c_alpha, I3, max(abs(theta_deg)), oscillations, average_amplitude];
        fprintf("Km=%.4f Ke=%.4f R=%.4f c=%.4f I3=%.4f | Max θ=%.1f° | Oscilaciones=%d | Prom. Amplitud=%.2f°\n", ...
            Km, Ke, R, c_alpha, I3, max(abs(theta_deg)), oscillations, average_amplitude);
    end
end

if ~isempty(resultados_ok)
    T = array2table(resultados_ok, ...
      'VariableNames', {'Km','Ke','R','c_alpha','I3','MaxTheta', 'Oscilaciones', 'PromedioAmplitud'});

    T = sortrows(T, {'Oscilaciones', 'MaxTheta', 'PromedioAmplitud'});


    fprintf("\nSe encontraron %d combinaciones con |θ| ≤ 80° todo el tiempo\n", height(T));
    disp(T);

    writetable(T, 'parametros_filtrados_80grados.csv');
else
    fprintf("\nNo se encontró ninguna combinación con |θ| ≤ 80°\n");
end

T = readtable('parametros_filtrados_80grados.csv');

disp("La mejor combinación encontrada es:");
disp(T(1,:));
