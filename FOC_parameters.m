% Lukas Jobb
% 04.10.2018
% Field orientated control for a PMSG
clc;
clear;
close all;

TL = 400;             % Load torgue
n_s = 1;            % speed value

% machine parameters
Rs = 0.0485;        % stator resistance per phase
Ld = 8.5e-3;        % Inductances 
Lq = Ld;           % equal for a salient pole machine
cphi = 0.1194;      % Flux linkage
Ti = 0.0027;        % Inertia [kg/m^2]
vd = 0.0004924;     % visous damping
p = 4;              % pole pairs
Tf = 0;             % static friction

% calculations
Tc = Lq/Rs;         % Time constante of the stator for the current
Kc = 1/Rs;          % Gain of the current 

% PWM parameter
m = 1;              % modulation
fs = 50*27;         % switiching frequency
pwm_angle = 90;     % initial phase
f = 50;

%% current control
% Pole placement
% B = Kc;             % numerator
% A = [1 Tc];         % denumerator
% A_star = [1 10*Tc];
% Betragsoptimum
Tsum = 1/(2*fs);

Ti_i = 4*Tsum;           % integral time constant for the current control 
Ki_i = Tc/(2*Kc*Tsum);   % current controler integral gain
Kp_i = Ki_i*Ti_i;           % current controler proportioal gain

% speed control
Ti_n = 1;           % integral time constant for the speed control 
Kp_n = 1;           % speed controler proportioal gain
Ki_n = Kp_n/Ti_n;   % speed controler integral gain

