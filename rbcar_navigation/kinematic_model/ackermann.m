
T = 1;     % tiempo de simulacion
ts = 0.01;	% tiempo de muestreo
tiempo = 0:ts:T;
n = length(tiempo);

numberofangles = 16;

spatial_resolution = 0.2;
angular_resolution = 2*pi/numberofangles;
midposes = 10; % es el numero de poses que tiene la primitiva, pero como muestreamos a 0.01 durante un segundo, tenemos mas
idx_poses = 1:ceil(n/midposes):n; % son las que vamos a coger para las primitivas
if idx_poses(end) ~= n
    idx_poses(end+1) = n;
end
poses = length(idx_poses);

% modelo de ackermann
v_ack = 2; % velocidad de avance
% a_ack = 0.7; % angulo del volante
% arange_ack = [0 -0.7:0.2:0.7]; % % rango de angulos del volante, asegurarse que está también el 0
arange_ack = [0 0.1 -0.1 0.3 -0.3 0.5 -0.5 0.7 -0.7]; % % rango de angulos del volante, asegurarse que está también el 0
cost_prim =  [1   5    5   5    5   5    5   5    5];
prims_per_angle = length(arange_ack);
d_ack = 1.83; % distancia entre ejes




outfilename = 'rbcar_today.mprim';
fout = stdout; %fopen(outfilename, 'w');
fprintf(fout, 'resolution_m: %f\n', spatial_resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', prims_per_angle*numberofangles);


%figure(1)


for angle_i=1:numberofangles
    %printf('angle %d of %d\n', angle_i, numberofangles)
%    clf(gcf)
%    hold on;
%    grid on;
    for a_ack_i=1:length(arange_ack)
        
        a_ack = arange_ack(a_ack_i);
        % modelo diferencial
        
        [v_dif, w_dif] = ack2dif(v_ack, a_ack, d_ack);
        x = zeros(1,n);
        y = zeros(1,n);
        theta = ones(1,n) * (angle_i-1)*2*pi/numberofangles;
        
        for i=2:n
            x(i) = x(i-1) + v_dif*ts*cos(theta(i-1));
            y(i) = y(i-1) + v_dif*ts*sin(theta(i-1));
            theta(i) = theta(i-1) + w_dif*ts;
        end
        
        
        x = x(idx_poses);
        y = y(idx_poses);
        theta = theta(idx_poses);
        
        x(end) = round2(x(end), spatial_resolution);
        y(end) = round2(y(end), spatial_resolution);
        theta(end) = round2(theta(end), angular_resolution);
        
        x_r = x(end);
        y_r = y(end);
        theta_r = theta(end);
        plot(x,y, 'k.');
        
        
        fprintf(fout, 'primID: %d\n', a_ack_i-1);
        fprintf(fout, 'startangle_c: %d\n', angle_i-1);
        fprintf(fout, 'endpose_c: %d %d %d\n', int32(x_r/spatial_resolution), int32(y_r/spatial_resolution), int32(theta_r/angular_resolution)); % pose final, en quadrats recorreguts
        fprintf(fout, 'additionalactioncostmult: %d\n', cost_prim(a_ack_i));
        fprintf(fout, 'intermediateposes: %d\n', poses);
        for i=1:length(x)
            fprintf(fout, '%.4f %.4f %.4f\n', x(i), y(i), theta(i));
        end
        %     plot(x_r, y_r, 'bo')
        %     fprintf('%f %f\n', x(end), y(end));
    end

    % pause
end
close
