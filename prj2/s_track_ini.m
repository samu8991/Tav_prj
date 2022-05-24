clear; clc;
close all;


UIControl_FontSize_bak = get(0, 'DefaultUIControlFontSize');
% Select simulation
set(0, 'DefaultUIControlFontSize', 18);
caso = menu('Choose CoG position','a = L/2-10cm','a = L/2','a = L/2+10cm') %,'Drift: a=b; Ca=Ca_nom/100');       
set(0, 'DefaultUIControlFontSize', UIControl_FontSize_bak);
if isempty(caso)
    caso=1;
end


%% --------- Vehicle and tyre DATA ----------
m=1997.6;       %[kg] mass
L = 2.85;       % [m] wheelbase
a_vet =[L/2-0.1 L/2 L/2+0.1 L/2];  %[m] front wheelbase
a = a_vet(caso);  
b = L-a ;       %[m] rear wheelbase        
Tf=1.54;        %[m] front track
Tr=1.54;        %[m] rear track
Jz=3728;        %[kg m^2] mass moment of inertia 
g = 9.81;       %[m/s^2]
tau_s = 15;     % steering ratio

%----- static loads
FzF = m*g*b/L; 
FzR = m*g*a/L;
Perc_F = FzF /(m*g)*100;
Perc_R = 100-Perc_F;
disp(['static load distribution (%F - %R): ',num2str(round(Perc_F)),'-',num2str(round(Perc_R))])

%------ Cornering stiffness
eval(['load Dati_txt' filesep 'CornStiff_Vs_Fz'])
% interpolate wheel cornering stiffness versus vertical load
CF_w = interp1(Fz_vet,C_alpha_vet,FzF/2);
CR_w = interp1(Fz_vet,C_alpha_vet,FzR/2);
C_med_w = interp1(Fz_vet,C_alpha_vet,m*g/4);

% axle stiffness
CF = 2*CF_w;    % front
if caso==4
    CR = 2*CR_w/100; % /100 for drift simualtion
else
    CR = 2*CR_w;    % rear
end
% % low friction
% mi_low = 0.37;
% CF = CF*mi_low;
% CR = CR*mi_low;

disp('axle cornering stiffness')
disp(['CF = ',num2str(CF),' N/rad'])
disp(['CR = ',num2str(CR),' N/rad'])
disp(' ')
% check under/over steering
if CR*b-CF*a>0
    disp('============ understeering ============')
elseif CF*a-CR*b==0
    disp('============ neutral vehicle ============')
else
    disp('============ oversteering ============')
    V_cr = sqrt(CF*CR*L^2/(m*(a*CF-b*CR)))*3.6;
    disp(['critical speed: ',num2str(round(V_cr*10)/10),' km/h'])
end
% understeering and slip angle gradients
mf = m*b/L; mr = m-mf;
K_us_an = (mf/CF-mr/CR);
K_beta_an = -mr/CR;
% tangent speed (beta=0)
V_beta0 = sqrt(b*L*CR/a/m)*3.6;
disp(' ')
disp(['understeering gradient: K_US = ',num2str(K_us_an),' rad/(m/s^2)'])
disp(['slip agle gradient: K_beta = ',num2str(K_beta_an),' rad/(m/s^2)'])
disp(['tangent speed: V_beta = ',num2str(V_beta0),' km/h'])

%% run Simulink model   
% default parameters
t_end_sim = 20;
delta_vol_max_ramp = 0;
dvol_max = 0;
t_drift_cs_1 = 0; t_drift_cs_2 = 0;
dvol_drift1 = 0; dvol_drift2 = 0; dvol_drift3 = 0;
% eval(['load Dati_txt' filesep 'sweep.txt'])

set(0, 'DefaultUIControlFontSize', 18);
sel_man = menu('select manoeuvre',' Step steer','Ramp steer 15 deg/s','Sine Sweep');
set(0, 'DefaultUIControlFontSize', UIControl_FontSize_bak);
if isempty(sel_man)
    sel_man=1;
end
switch sel_man
    case 1
        %         (Step steer)
        t_end_sim = 10*1;      % [s]
        Vd = 100;                % [km/h]
        dvol_max = input('max steering angle (deg) [default = 10 deg]:');
    case 2
        %        (ramp steer)
        t_end_sim = 36*0+20;
        Vd = 40;                % [km/h]
        delta_vol_max_ramp = input('max steering angle (deg) [default = 200 deg]:');
    case 3
        %        (Sine Sweep)
        t_end_sim=5;
        Vd = 100;               % [km/h]
        dvol_max = 20;              % [deg] steering wheel angle
        freq_delta=input('frequency of the sine sweep comand (Hz) [default= 200 Hz]:')
end
if isempty(dvol_max)==1
    dvol_max=10; % max steering angle
end
if isempty(delta_vol_max_ramp)==1
    delta_vol_max_ramp = 200; % max steering angle
end
freq_delta=200;
% if isempty(freq_delta)==1
%     freq_delta=200;
% end

V = input(['choose speed for Simulink manoeuvre (default = ' num2str(Vd) ' km/h): ']);
if isempty(V)
    V=Vd;       % km/h
end
V=V/3.6; 
Vv=V;

% matrix initialization for Simulink linear model 
% state space matricesi: A,B,C,D
    A_sim=[(-CF-CR)/(m*Vv),(-CF*a+CR*b-m*Vv^2)/(m*Vv^2);
        (-CF*a+CR*b)/Jz,(-CF*a^2-CR*b^2)/(Jz*Vv)];
    B_sim=[CF/(m*Vv) CR/(m*Vv);
        (CF*a/Jz) -(CR*b/Jz)];
    C_sim = [1,0
        0,1
        (-CR-CF)/(m*Vv^2),(-CF*a+CR*b)/(m*Vv^3)
        -1, -a/Vv
        -1, b/Vv
        (-CR-CF)/(m),(-CF*a+CR*b)/(m*Vv)];
    D_sim = [0 0;
        0 0;
        CF/(m*Vv^2) CR/(m*Vv^2)
        1 0
        0 1
        CF/m CR/m];
%

% Run simualtion
dt_sim = 1e-3;          % time step
ay_max = 8;            % [m/s^2] limit acceleration: stop simulation from 10 max
beta_max = 10;          % [deg] limit slip angle: stop simulation from 80 max
%return

%% run simscape model 
sim('s_track_model');    % run Simulink model

%% POST PROCESSING
%--------- Plot Steering angle

figure('Name','steering angle')
hold all; grid on
plot(delta_steer,'LineWidth',2)
xlabel('time [s]'),
hold on; 
plot(L*ro*180/pi*tau_s,'--k'); 

legend('\delta','\delta_0','Fontsize',18,'location','best')
title('Steering Angle \delta_s')

%--------- Plot beta and psi_dot

figure('Name','States')
hold all; grid on
subplot(2,1,1)
plot(beta,'LineWidth',2)
xlabel('time [s]')
hold on
plot(b*ro*180/pi,'--k'); 

legend('\beta','\beta_0','Fontsize',16,'location','best')
title('slip angle \beta [deg]','Fontsize',16)
grid on
subplot(2,1,2)
plot(r,'LineWidth',2)
hold on
plot(delta_steer/180*pi,'LineWidth',2),xlabel('time [s]'),

title('r [deg/s]','Fontsize',16), xlabel('time [s]')
ylabel('')
legend('r [deg/s]','\delta [rad]','Fontsize',16,'location','best')
ylim([-150 150])


% --------- Plot alpha_F e alpha_R 
figure('Name','alphaF e R');  hold all
plot(alfaF*180/pi,'LineWidth',2); plot(alfaR*180/pi,'LineWidth',2); 
xlabel('time [s]'); ylabel('\alpha [deg]'); grid on; 
legend('\alpha_F','\alpha_R','Fontsize',16,'location','best')
title('Tyre slip angles')
hold off
% figure('Name','a_y(t)')

figure('Name', 'a_y(t)');
hold all; grid on
plot(a_y,'LineWidth',2)
title('Lateral Acceleration a_y [m/s^2]','Fontsize',16)
xlabel('time [s]')
ylabel('')
legend('a_y [m/s^2]','Fontsize',16,'location','best')
hold off 
% %% Vehicle Trajectory
F_Size=13;

figure('Name','Vehicle CG location','NumberTitle','off','PaperType','A4');
hold all; grid on
plot(Var_(:,1),Var_(:,2),'Linewidth',2);
title('CG Trajectory'); axis equal
set(gca,'FontName','Times New Roman','FontSize',F_Size)
xlabel('X [m]'); ylabel('Y [m]')
X_G = Var_(:,1);
Y_G = Var_(:,2);


hold on
axis equal
dt_frame = 0.1; % [s] vehicle frame refresh (time) interval 
decim_frame = dt_frame/dt_sim;

% cycle to show vehicle motion
for cont1=1:decim_frame:length(Psi)
    X = X_G(cont1)+[-b a a -b]';
    Y = Y_G(cont1)+[-Tf/2 -Tf/2 Tf/2 Tf/2]';
    vert = [X,Y];
    fac = [1 2 3 4];
    hVeicolo = patch('Faces',fac,'Vertices',vert,'FaceColor','red','FaceAlpha',.5);
    direction = [0 0 1];
    xlim([X(1)-5 X(2)+3])
    ylim([Y(1)-5 Y(3)+3])
    
    x0 = X_G(cont1);
    y0 = Y_G(cont1);
    z0 = 0;
    ORIGIN = [x0,y0,z0];
    rotate(hVeicolo,direction,Psi(cont1,2),ORIGIN);
    pause(0.1)
end

plot(Var_(:,1),Var_(:,2),'Linewidth',2);
axis auto

% %% TF estimation 
if sel_man == 3
    fs=1/dt_sim; % Hz
    t_wind = 5; % s
    nfft=t_wind*fs;	         	% number of points (fft 512)
    nolap=0.9*nfft;				% overlapping percentage (Hanning window) 90%
    [T_ay_d,freq1] = tfestimate(delta_steer.Data,a_y.Data,nfft,nolap,[],fs);
    figure
    bode(T_ay_d);
%     plot(freq1,abs(T_ay_d))
    grid on
%     xlim([0 1000])
%     xlabel('frequency [Hz]')
%     ylabel('a_y/\delta [g/deg]')
end
