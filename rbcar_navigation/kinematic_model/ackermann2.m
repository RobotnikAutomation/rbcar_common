% como es un modelo cinematico, no tenemos en cuenta velocidades, solo
% distancias recorridas,

T = 1;      % tiempo de simulacion
ts = 0.01;	% tiempo de muestreo, cuanto más pequeño, mejor sale la curva
tiempo = 0:ts:T; % para generar tantas muestras por segundo
n = length(tiempo);

% tamaño de las celdas, se debe de corresponder con la resolución del mapa
spatial_resolution = 0.2; 
% orientación discretizada de las celdas, se corresponde con el
% numberofangles?
angular_resolution = 2*pi/16; 
% numero de poses intermedias para cada primitiva, sin contar la final. Se
% debe de recordar que la final se ajusta a una posición discreta
midposes = 10; 
idx_poses = 1:ceil(n/midposes):n;
if idx_poses(end) ~= n
    idx_poses(end+1) = n;
end
poses = length(idx_poses);

wheelbase_ack = 1.83; % distancia entre ejes, 1.65 rbcar, 0.335 toy
% recto
% distancia lineal recorrida
d_ack_straight =        [0.5 2];
% que en velocidad es
v_ack_straight =        d_ack_straight/T;
% angulo de giro
arange_ack_straight =   [  0  0];
cost_straight =         [  1  1];


% con giros
v_ack_turn = 2; % velocidad de avance
arange_ack_turn = [0.1:0.05:0.6 -0.1:-0.05:-0.6]; % % rango de angulos del volante
% arange_ack_turn = [0.1:0.1:0.7 -0.1:-0.1:-0.7];
cost_turn =  2;
cost_turnback = 15;
% cost_turn =  [       5    5   5    5   5    5   5    5];
% cost_turn =  [       2    2   2    2   2    2   2    2];

% [v_dif_straight, w_dif_straight] = ack2dif(v_ack_straight, arange_ack_straight, wheelbase_ack);
% [v_dif_turn, w_dif_turn] = ack2dif(1, arange_ack_turn, wheelbase_ack);
% [v_dif_turnback, w_dif_turnback] = ack2dif(4, arange_ack_turn, wheelbase_ack);
% 
% v_dif = [v_dif_straight v_dif_turn v_dif_turnback];
% w_dif = [w_dif_straight w_dif_turn w_dif_turnback];
% cost_prim = [cost_straight cost_turn cost_turnback];

v_dif = [];
w_dif = [];
cost_prim = [];

[v, w, c] = ack2dif(v_ack_straight, arange_ack_straight, wheelbase_ack, cost_straight);
v_dif = [v_dif v]; w_dif = [w_dif w]; cost_prim = [cost_prim c];

[v, w, c] = ack2dif(1, arange_ack_turn, wheelbase_ack, cost_turn);
v_dif = [v_dif v]; w_dif = [w_dif w]; cost_prim = [cost_prim c];

% [v, w, c] = ack2dif(4, arange_ack_turn, wheelbase_ack, cost_turn);
% v_dif = [v_dif v]; w_dif = [w_dif w]; cost_prim = [cost_prim c];

%[v, w, c] = ack2dif(-1, arange_ack_turn, wheelbase_ack, cost_turnback);
%v_dif = [v_dif v]; w_dif = [w_dif w]; cost_prim = [cost_prim c];
% 
% [v, w, c] = ack2dif(-4, arange_ack_turn, wheelbase_ack, cost_turnback);
% v_dif = [v_dif v]; w_dif = [w_dif w]; cost_prim = [cost_prim c];


prims_per_angle = length(v_dif);

numberofangles = 16; % potencia de 2 y multiplo de 8? o pareix que ha de ser 16
outfilename = 'polaris_02res.mprim';
fout = stdout; %fopen(outfilename, 'w');
fprintf(fout, 'resolution_m: %f\n', spatial_resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', prims_per_angle*numberofangles);



draw = 0;

if draw
    figure(1);
end
for angle_i=1:numberofangles
    fprintf(stderr, 'angle %d of %d\n', angle_i, numberofangles)
    if draw
        clf(gcf)
        hold on;
        grid on;
        axis square;
    end
    for prim_i=1:prims_per_angle
        x = zeros(1,n);
        y = zeros(1,n);
        theta = ones(1,n) * (angle_i-1)*2*pi/numberofangles;
        
        for i=2:n
            x(i) = x(i-1) + v_dif(prim_i)*ts*cos(theta(i-1));
            y(i) = y(i-1) + v_dif(prim_i)*ts*sin(theta(i-1));
            theta(i) = theta(i-1) + w_dif(prim_i)*ts;
        end
        
        
        x = x(idx_poses);
        y = y(idx_poses);
        theta = theta(idx_poses); % no se le deberia restar la theta inicial, para que realmente sean cambios en la orientacion?
        u = cos(theta) * spatial_resolution * 4;
        v = sin(theta) * spatial_resolution * 4;
        
        if draw
            plot(x,y, 'k.');
            quiver(x(end),y(end),u(end),v(end),'k');
        end

        x(end) = round2(x(end), spatial_resolution);
        y(end) = round2(y(end), spatial_resolution);
        theta(end) = round2(theta(end), angular_resolution);
        x_r = x(end);
        y_r = y(end);
        theta_r = theta(end);
        u_r = cos(theta_r) * spatial_resolution * 4;
        v_r = sin(theta_r) * spatial_resolution * 4;
        if draw
            plot(x_r, y_r, 'bo');
            quiver(x_r,y_r,u_r,v_r,'r');
        end
        
        fprintf(fout, 'primID: %d\n', prim_i-1);
        fprintf(fout, 'startangle_c: %d\n', angle_i-1);
        fprintf(fout, 'endpose_c: %d %d %d\n', int32(x_r/spatial_resolution), int32(y_r/spatial_resolution), int32(theta_r/angular_resolution)); % pose final, en quadrats recorreguts
        fprintf(fout, 'additionalactioncostmult: %d\n', cost_prim(prim_i));
        fprintf(fout, 'intermediateposes: %d\n', poses);
        for i=1:length(x)
            fprintf(fout, '%.4f %.4f %.4f\n', x(i), y(i), theta(i));
        end
            
        %     fprintf('%f %f\n', x(end), y(end));
    end
%     xl = xlim;
%     yl = ylim;
%     set(gca, 'xtick', xl(1):spatial_resolution:xl(2));
%     set(gca, 'ytick', yl(1):spatial_resolution:yl(2));
    xlim([-3 3]);
    ylim([-3 3]);
    if draw
%        pause(2)
    end
end
close
