clear all
close all
clc

bot = AckermannSteeringCar();
bot.Stop_Simulation
pause(1)
load path.mat

bot = AckermannSteeringCar();

R = 0.63407/2;
D = 2.5772;
L = 1.51034/2;

%Config 1
p = bot.Get_Pose();           %Posicion inicial

% p1 = bot.ReferenceFrame_Pose1();
% p1 = p1(1:2,4);
% p2 = bot.ReferenceFrame_Pose2();
% p2 = p2(1:2,4);
% p3 = bot.ReferenceFrame_Pose3();
% p3 = p3(1:2,4);
% p4 = bot.ReferenceFrame_Pose4();
% p4 = p4(1:2,4);
% p5 = bot.ReferenceFrame_Pose5();
% p5 = p5(1:2,4);
% p6 = bot.ReferenceFrame_Pose6();
% p6 = p6(1:2,4);
p7 = bot.ReferenceFrame_Pose7();
p7 = p7(1:2,4);
% pf = bot.ReferenceFrame_Pose8();
% pf = pf(1:2,4);

%% Primer ajuste
x_ref = 6.7;          %VALOR FIJO
y_ref = 3.225  - D/2; %VALOR FIJO
first_distance = 2.6; %OBTENER LA PRIMER MEDICION DE DISTANCIA
distance_offset = 7;
run_distance = 5.5*1.5;   %MEDIR O CALCULAR LA DISTANCIA QUE AVANZÓ
sensor_distance = 0.25;%MEDIR DISTANCIA A LO LARGO ENTRE EL SENSOR ULTRASONICO Y EL REFERENCE FRAME DEL CARRO
x_actual = x_ref - first_distance - L;
y_actual = y_ref + distance_offset - run_distance - sensor_distance;
x_D = 2.825;
y_D = -4.125;
kv = 1;                   %Ganancias
kw = 1;
kx = 1;
ky = 1;
% pe = p;                     %P odometria
% D = 0.29;                   %Distancia entre llantas a lo largo
alpha = 0;                  %Posicion de las llantas delanteras inicial
dt = 0.05;                  %Cambio
S = 5;                     %Tiempo de simulacion
tf = S;
% R = 0.033;                   %Radio de las ruedas
% N = 450/2;                    %Numero de espacios del encoder
% M = Fake_Wheel(N,R,dt);     %Funcion de simulacion de llanta
% c = 0;                      %Inicializacion de contador del encoder
pd_plot = [];               %Inicializacion de graficas
p_plot = [];
pp_plot = [];
q_plot = [];
t_plot = [];
h = D/2;
offset = -pi/2;
p(3) = offset;
p(1) = x_actual; %+ h*cos(p(3));
p(2) = y_actual; %+ h*sin(p(3));

%Punto objetivo (antes Splines)
qx = path(1,end);
qy = path(2,end);
% Spx = Obtener_Splines(tsp,qx);
% Spy = Obtener_Splines(tsp,qy);

for t=dt:dt:(S)
%% Odometria y Punto de Control
    % Se utilizan variables temporales (xc, yc) para evaluar el controlador
    % dejando intacto el estado real 'p' para que no se acumule error.
   theta_c = p(3); % Ya no sumamos el offset aquí repetitivamente
   xc = p(1) + h*cos(theta_c);
   yc = p(2) + h*sin(theta_c);
    
    % Mantenemos el punto objetivo fijo
   xd = qx;
   xpd = 0;
   yd = qy;
   ypd = 0;

    % El error se calcula con respecto al punto temporal adelantado
   e = [xc-xd, yc-yd];
   ux = xpd + kx*(xd-xc);
   uy = ypd + ky*(yd-yc);
    
   v = cos(theta_c)*ux + sin(theta_c)*uy;
   w = -sin(theta_c)*ux/D + cos(theta_c)*uy/D;
    
   v = v*kv;
   w = w*kw;

    % thetad = atan2(yd-yc,xd-xc);
    % rho = sqrt((xd-xc)^2 + (yd-yc)^2);
    % ev = rho;
    % ew = thetad - theta_c;
    % ew = atan2(sin(ew),cos(ew));
    % 
    % v = ev*kv;
    % w = ew*kw;

   alpha = atan2(w*D,v);       % Calculo de posicion de llantas delanteras
    
   cotadeg = 45;                               % Valor de cota en grados de las llantas
   cotarad = deg2rad(cotadeg);                 % Valor de cota en radianes de las llantas
   alpha = max(min(alpha,cotarad),-cotarad);   % Cota de angulo de las llantas
   deg = rad2deg(alpha);                       % UNICAMENTE PARA VERIFICAR QUE SI SE ACOTE CORRECTAMENTE
    
   w = tan(alpha)*v/D;                         % Recalculo de w
                                                
   bot.Set_Joint_Velocity(v,w);
   bot.Set_Direction_Angle(alpha);
   bot.Simulation_Step();
   
   % La cinemática de actualización real del carro en la simulación
   DELTA_p = [v*cos(p(3))*dt; v*sin(p(3))*dt; v*tan(alpha)*dt/D]; 
        
   p(1) = p(1) + DELTA_p(1);
   p(2) = p(2) + DELTA_p(2);
   p(3) = p(3) + DELTA_p(3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% AQUI TERMINA EL PRIMER AJUSTE %

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Estacionado
kv = 1;                   %Ganancias
kw = 100;

kx = 1;
ky = 1;

% pe = p;                     %P odometria
% D = 0.29;                   %Distancia entre llantas a lo largo
alpha = 0;                  %Posicion de las llantas delanteras inicial

dt = 0.05;                  %Cambio
S = 10;                     %Tiempo de simulacion
tf = S;

% R = 0.033;                   %Radio de las ruedas
% N = 450/2;                    %Numero de espacios del encoder
% M = Fake_Wheel(N,R,dt);     %Funcion de simulacion de llanta
% c = 0;                      %Inicializacion de contador del encoder

pd_plot = [];               %Inicializacion de graficas
p_plot = [];
pp_plot = [];
q_plot = [];
t_plot = [];

h = D/2;
offset = pi/2;

p(3) = p(3) + offset;

p(1) = p(1) + h*cos(p(3));
p(2) = p(2) + h*sin(p(3));

%Spline

qx = flip(path(1,:));
qy = flip(path(2,:));
qx = [qx p7(1)];
qy = [qy p7(2)];

tsp = linspace(0,S,length(qx)-1)';
tsp = [tsp; 12];

% Spx = Obtener_Splines(tsp,qx);
% Spy = Obtener_Splines(tsp,qy);
nt = 0;

for t=dt:dt:(S+2)
    if t >= S
        kw = 0.2;
    end
%% Odometria
    p = bot.Get_Pose();

    p(3) = p(3) + offset;

    p(1) = p(1) + h*cos(p(3));
    p(2) = p(2) + h*sin(p(3));

    %Calcular punto
    [xd,xpd] = Evaluar_Splines_L(qx,tsp,t);
    [yd,ypd] = Evaluar_Splines_L(qy,tsp,t);

    e = [p(1)-xd, p(2)-yd];

    ux = xpd + kx*(xd-p(1));
    uy = ypd + ky*(yd-p(2));
    
    v = cos(p(3))*ux + sin(p(3))*uy;
    w = -sin(p(3))*ux/D + cos(p(3))*uy/D;

    v = v*kv;
    w = w*kw;

    alpha = -atan2(w*D,v);       %Voltea el angulo de giro                %Calculo de posicion de llantas delanteras

    cotadeg = 45;                               %Valor de cota en grados de las llantas
    cotarad = deg2rad(cotadeg);                 %Valor de cota en radianes de las llantas
    alpha = max(min(alpha,cotarad),-cotarad);   %Cota de angulo de las llantas
    deg = rad2deg(alpha);                       %UNICAMENTE PARA VERIFICAR QUE SI SE ACOTE CORRECTAMENTE
    w = tan(alpha)*v/D;                         %Recalculo de w, esto debido
                                                %a que alpha no solo
                                                %depende de w, sino que
                                                %tambien de v, es por eso
                                                %que se recalculo despues
                                                %de la cota y no podemos
                                                %acotar directamente w
   nt = v*tan(alpha)/D;
   v = -v;                      %Cambia la velocidad a reversa

   bot.Set_Joint_Velocity(v,w);
   bot.Set_Direction_Angle(alpha);

   bot.Simulation_Step();
    %alpha = atan2(w*d,v)                        %SOLO COMPROBACION DE QUE FUNCIONARON LAS COTAS
    %deg = rad2deg(alpha)                        %SOLO COMPROBACION DE QUE FUNCIONARON LAS COTAS

    %wr = (v + w*d)/R                           %CALCULO DEL DIFERENCIAL, EN ESTE CODIGO NO SE UTILIZA
    %wl = (v - w*d)/R                           %PERO EN LA IMPLEMENTACION SI LO NECESITAREMOS

    % M.Set_Wheel_Velocity(v/R);                  %Asignacion de velocidad a las llantas
    % c_n = M.Read_Enconder_Count();              %Contador de encoder en dt
    % d_c = c_n - c;                              %Diferencia del contador respecto a dt pasado
    % c = c_n;
    % d_d = 2*pi*R*(d_c/N);                       %Velocidad de la rueda
    % vs = d_d;                                   %Velocidad lineal del carrito
    
    % d_pe = [vs*cos(pe(3)) vs*sin(pe(3)) vs*tan(alpha)/D]'; %Vector de cambio [x y th](derivada)
    % pe = pe + d_pe;                             %Obtencion de nuestra nueva posicion
    
    % cla
    % Dibujar_Coche([pe; alpha],D)
    % plot(xd,yd,'.r')
    % axis([-4 4 -4 4])
    % drawnow

    pd_plot = [pd_plot [xd yd]'];
    p_plot = [p_plot p];
    %pp_plot = [pp_plot d_pe];
    q_plot = [q_plot [v w]'];
    t_plot = [t_plot t];
end
bot.Set_Direction_Angle(0);
bot.Set_Joint_Velocity(0,0);
tic
while toc < 10
    bot.Simulation_Step();
end
bot.Stop_Simulation();
figure
hold on
grid on
plot(p_plot(1,:),p_plot(2,:))
title('Trayectoria')

figure
hold on
grid on
plot(t_plot,p_plot)
title('Poses respecto al tiempo')
legend('x','y','theta')

figure
hold on
grid on
plot(t_plot,[p_plot(1,:); pd_plot(1,:);p_plot(2,:); pd_plot(2,:)])
title('Posición actual vs deseada')
legend('x_a','x_d','y_a','y_d')

% figure
% hold on
% grid on
% plot(t_plot,pp_plot)
% title('Velociades respecto al tiempo')
% legend('xp','yp','thetap')

figure
hold on
grid on
plot(t_plot,q_plot)
title('Accion de control')
legend('v','w')

%%
function [qk,qpk] = Evaluar_Splines_L (q,t,time)
    n = numel(t);

    if time>t(n)
        k = n-1;
    else
        k = find(t(1:n-1)<=time & time<=t(2:n),1);
    end

    qk = q(k) + ((q(k+1)-q(k))/(t(k+1)-t(k)))*(time-t(k));
    qpk = ((q(k+1)-q(k))/(t(k+1)-t(k)));
end