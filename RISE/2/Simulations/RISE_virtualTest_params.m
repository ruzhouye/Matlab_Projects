% RISE updated
clear
clc

%% Plant
sys_params.G1 = 5e6;
sys_params.n1 = 200;
sys_params.s1 = 0.00005;
sys_params.G2 = sys_params.G1;
sys_params.n2 = 550;
sys_params.s2 = 0.00005;
sys_params.eta = 0.85;
sys_params.l = 0.025;
sys_params.L1 = 0.5;
sys_params.L2 = 2.8;
sys_params.L3 = 1.2;
sys_params.L4 = 4.5;
sys_params.m2 = 200;
sys_params.Jm1 = 1.16e-4;
sys_params.Rm1 = 1.93;
sys_params.Ku1 = 1.11;
sys_params.K1 = 2;
sys_params.Ke1 = 0.5;
sys_params.Jm2 = 6.04e-4;
sys_params.Rm2 = 1.93;
sys_params.Ku2 = 0.55;
sys_params.K2 = 2;
sys_params.Ke2 = 0.64;
sys_params.theta_1 = [100;100];
sys_params.theta_2 = [140;140];
sys_params.theta_3 = [200;200];
sys_params.C = [15 15;1.5 1.5;300 300];
sys_params.g = 9.81;

%% Controller
% Nominal Plant Parameters
ctr_params.G1 = sys_params.G1*0.95;
ctr_params.n1 = sys_params.n1*0.99;
ctr_params.s1 = sys_params.s1*0.92;
ctr_params.G2 = sys_params.G2*1.06;
ctr_params.n2 = sys_params.n2*1.03;
ctr_params.s2 = sys_params.s2*1.05;
ctr_params.eta = sys_params.eta*0.89;
ctr_params.l = sys_params.l*0.99;
ctr_params.L1 = sys_params.L1*1.2;
ctr_params.L2 = sys_params.L2*0.98;
ctr_params.L3 = sys_params.L3*1.3;
ctr_params.L4 = sys_params.L4*0.92;
ctr_params.m2 = sys_params.m2*1.25;
ctr_params.Jm1 = sys_params.Jm1*1.13;
ctr_params.Rm1 = sys_params.Rm1*0.79;
ctr_params.Ku1 = sys_params.Ku1*0.9;
ctr_params.K1 = sys_params.K1*0.88;
ctr_params.Ke1 = sys_params.Ke1*1.19;
ctr_params.Jm2 = sys_params.Jm2*1.59;
ctr_params.Rm2 = sys_params.Rm2*0.83;
ctr_params.Ku2 = sys_params.Ku2*1.24;
ctr_params.K2 = sys_params.K2*1.08;
ctr_params.Ke2 = sys_params.Ke2*1.19;
ctr_params.theta_1 = sys_params.theta_1*0.96;
ctr_params.theta_2 = sys_params.theta_2*1.18;
ctr_params.theta_3 = sys_params.theta_3*0.88;
ctr_params.C = sys_params.C*1.02;
ctr_params.g = sys_params.g;
% virtual control
ctr_params.sigma_1 = diag([1.5,5]);
ctr_params.rho = diag([10,5]);
ctr_params.Kc1 = diag([20,0.01]);
ctr_params.epsilon = 0.02;
% NN
ctr_params.L = 11;
ctr_params.gamma = 2;
ctr_params.Gamma = 20*eye(ctr_params.L);
ctr_params.c = zeros(11,1); %
ctr_params.b = zeros(ctr_params.L,1);
for i = 1:length(ctr_params.b)
    ctr_params.b(i,1) = 0.5*1.5^(i-1);
end


%% Initial Conditions
x_0 = zeros(4,1); %
W_hat_0 = 0.1*(rand(ctr_params.L,2)-0.5); 














