% Lukas Jobb
% 04.10.2018
% Field orientated control for a PMSG
clc;
clear;
close all;

TL = 6;             % Load torgue
n_s = 1;            % speed value
Udc = 200;

% machine parameters original
% Rs = 0.0485;        % stator resistance per phase
% Ld = 8.5e-3;        % Inductances 
% Lq = Ld;            % equal for a salient pole machine
% cphi = 0.1194;      % Flux linkage
% Ti = 0.0027;        % Inertia [kg/m^2]
% vd = 0.0004924;     % visous damping
% p = 4;              % pole pairs
% Tf = 0;             % static friction

% machine parameters TI
Rs = 0.424;        % stator resistance per phase
Ld = 0.002;        % Inductances 
Lq = Ld;            % equal for a salient pole machine
cphi = 0.1194;      % Flux linkage
Ti = 0.0281;        % Inertia [kg/m^2]
vd = 0.0004924;     % visous damping
p = 4;              % pole pairs
Tf = 0;             % static friction
Frcition = 0.1233;

% calculations
Tc = Lq/Rs;         % Time constante of the stator for the current
Kc = Rs;          % Gain of the current 
Tcurrent = 0.37;

% Inverter parameter
m = 1;              % modulation
fs = 50*27;         % switiching frequency
pwm_angle = 0;      % initial phase
f = 50;
w = 2*pi*f;
C = 1e-3;           % Dc- capacitor
dt = 0.5;           % Voltage changetime 

% Current mesaurement filter
Tf = 1/(1000);      % Cutoff frequency of the filter

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% current control
% Pole placement
% B = Kc;             % numerator
% A = [1 Tc];         % denumerator
% A_star = [1 10*Tc];
%% Betragsoptimum Strom
Tsum = 1/(2*fs)+Tf;

Ti_i = 4*Tsum;           % integral time constant for the current control 
% Ti_i = Tc;
Kp_i = Tc/(2*Kc*Tsum);   % current controler proportinal gain
Ki_i = Kp_i/Ti_i;        % current controler integral gain
%% Symmetrisches Optimum Strom
% Ti_i = Tc;
% Ki_i = Tc/(2*Kc*Tsum);   % current controler integral gain
% Kp_i = Ki_i*Ti_i;        % current controler proportioal gain

% speed control Betragsoptimum Optimum
Tsum_n = Tsum+Ti_i+Tc;
Ti_n = Ti;                           % integral time constant for the speed control 
Kn = (3*p*cphi)/2;                   % 
Kp_n = Ti_n/(2*Kn*Tsum_n);         % speed controler proportioal gain
Ki_n = Kp_n/Ti_n;                    % speed controler integral gain

% Ti-speedcontrol
% BWc = 2*pi*fs/20;
% % Kp_i = BWc*Lq;
% % Ki_i = Rs/Lq;
% Ki_n = 1/(4^2*Tsum_n)
% Kp_n = 1/(2*Kn*Tsum_n)


%% Windturbin side
v_w = 10;                   % [m/s] wind speed
rho = 1.2041;               % [kg/m^3] air density
Pg = 2.5e3;                 % nominal generator power in W
Vdc = 60;                   % V_dcmax in V
Vph = Vdc/(sqrt(3*2));      % ideal input of diode rectifier in V
I_s = Pg/(3*Vph);           % stator current in A
ng_0 = 900;                 % norminal speed in min^-1
p_g = 32;                   % Polpairs
P_base = 949.9942/Pg;       % Power at 10m/s and best cp in pu
Tg_n =  Pg/(2*pi*ng_0/60);  % nominal torque in Nm
cphi_g = Vph/(ng_0/60);     % machine constant
Phi_g = cphi_g/(2*pi*p_g)*sqrt(2); % Flux linkage
Z_pu = Vph/I_s;             % pu impedance
f1_n = ng_0*p_g/60;         % electrical frequency
Rs_g = 0.05*Z_pu;           % stator resistance in ohm
X_g = 0.3*Z_pu;             % stator inductance impedance in ohm
Ls_g = X_g/(2*pi*f1_n);     % Stator inductance in H
Zs = sqrt(Rs_g^2+X_g^2);    
% n_0 = Vph/cphi_g;         % no load speed [m/s]

% R_line = Vdc^2/Pg;          % To get the full Power at Vdc
R_line = 0.0838;            % Udc = 0 vw =10ms ==> Mpp

% Turbin parameters
R_blades = 1;              % blade length just some value which gives a good power for the turbin at nominal wind
v_opt_10ms =  13.1195;      % optimal speed for maximum power by vw = 10ms
Base_rotational_speed = v_opt_10ms/ng_0*60; % vopt to pu
