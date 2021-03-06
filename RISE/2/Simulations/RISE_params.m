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
ctr_params.n1 = sys_params.n1*0.93;
ctr_params.s1 = sys_params.s1*1.07;
ctr_params.G2 = sys_params.G2*1.01;
ctr_params.n2 = sys_params.n2*0.99;
ctr_params.s2 = sys_params.s2*0.99;
ctr_params.eta = sys_params.eta*1.13;
ctr_params.l = sys_params.l*1.01;
ctr_params.L1 = sys_params.L1*0.86;
ctr_params.L2 = sys_params.L2*0.91;
ctr_params.L3 = sys_params.L3*1.01;
ctr_params.L4 = sys_params.L4*1.10;
ctr_params.m2 = sys_params.m2*0.89;
ctr_params.Jm1 = sys_params.Jm1*1.04;
ctr_params.Rm1 = sys_params.Rm1*1.1;
ctr_params.Ku1 = sys_params.Ku1*0.99;
ctr_params.K1 = sys_params.K1*0.90;
ctr_params.Ke1 = sys_params.Ke1*1.01;
ctr_params.Jm2 = sys_params.Jm2*1.03;
ctr_params.Rm2 = sys_params.Rm2*0.98;
ctr_params.Ku2 = sys_params.Ku2*0.87;
ctr_params.K2 = sys_params.K2*0.90;
ctr_params.Ke2 = sys_params.Ke2*1.2;
ctr_params.theta_1 = sys_params.theta_1*1.05;
ctr_params.theta_2 = sys_params.theta_2*1.06;
ctr_params.theta_3 = sys_params.theta_3*0.92;
ctr_params.C = sys_params.C*1.01;
ctr_params.g = sys_params.g;
% virtual control
ctr_params.sigma_1 = diag([20,50]);
ctr_params.rho = diag([0.2,0.2]);
ctr_params.Kc1 = diag([20,1]);
ctr_params.epsilon = 0.05;
% NN
ctr_params.L = 11;
ctr_params.gamma = 2;
ctr_params.Gamma = 5*eye(ctr_params.L);
ctr_params.Beta = 0.5;
ctr_params.c = zeros(11,1); %
ctr_params.b = zeros(ctr_params.L,1);
for i = 1:length(ctr_params.b)
    ctr_params.b(i,1) = 0.5*1.5^(i-1);
end

% Filter
poles = [-1000,-600,-140];  % poles
a = poly(poles);         % polynomial with the poles
aa = -flip(a(2:end));    % flip the coefficient (see linear control theory: pole placement)
ctr_params.Af = [zeros(2,2),eye(2),zeros(2,2);
                 zeros(2,2),zeros(2,2),eye(2);
                 aa(1)*eye(2),aa(2)*eye(2),aa(3)*eye(2)];
ctr_params.Bf = [zeros(2);zeros(2);-aa(1)*eye(2)];
ctr_params.Cf = [eye(2) zeros(2) zeros(2)];
    % Find P for closed-loop stability test
    P = sdpvar(6,6,'symmetric');
    Q = sdpvar(6,6,'symmetric');
    epsilon_squre = sdpvar(1,1);
    kappa_2 = sdpvar(1,1);
    kappa_1 = sdpvar(1,1);
    Psi = [(ctr_params.Af')*P+P*ctr_params.Af+epsilon_squre*(ctr_params.Cf')*ctr_params.Cf+Q, P*(ctr_params.Af+ctr_params.Bf*ctr_params.Cf), -P;
           (ctr_params.Af'+(ctr_params.Cf')*(ctr_params.Bf'))*P, -kappa_1*eye(6), zeros(6);
           -P, zeros(6), -kappa_2*eye(6)];
    Constraints = [kappa_1>=0,kappa_2>=0,P>=0,Q>=0,epsilon_squre>=0,Psi<=0];
    tol = 1e-6;
    opt = sdpsettings('solver', 'sedumi', 'sedumi.eps',tol);
    diagnostics_lmi = optimize(Constraints,[],opt);
    tmin = diagnostics_lmi.problem;
    if tmin == 0
        P_value = value(P);
        Q_value = value(Q);
        epsilon_squre_value = value(epsilon_squre);
        kappa_1_value = value(kappa_1);
        kappa_2_value = value(kappa_2);
    else
        P_value = [];
        Q_value = [];
        epsilon_squre_value = [];
        kappa_1_value = [];
        kappa_2_value = [];
    end
% actual control
ctr_params.sigma_2 = diag([20,20]);
ctr_params.Ks = diag([60,100]);
ctr_params.sigma_3 = diag([2,2]);
ctr_params.beta = 50;


%% Initial Conditions
x_0 = zeros(8,1);
W_hat_0 = 0.1*(rand(ctr_params.L,2)-0.5);
xi_0 = zeros(6,1);


%% Simulations
sim_time = 30;
sim('RISE.slx')
save('RISE_Data','x','ref','u','e1','z1','filter_out','alpha_1','W_hat')

%% Data Analysis
load('RISE_Data.mat')
t = x.time;
x_data = x.data;
ref_data = ref.data;
u_data = u.data;
e1_data = e1.data;
z1_data = z1.data;
filter_out_data = filter_out.data;
alpha_1_data = alpha_1.data;
W_hat_data = W_hat.data;
% plot - x
figure
for i = 1:(size(x_data,2)/2)
    subplot(size(x_data,2)/2,1,i)
    plot(t,x_data(:,1+(i-1)*2),'b-',t,x_data(:,2+(i-1)*2),'r-','LineWidth',1.2)
    hold on
    grid on
    xlabel('Time (s)')
    ylabel(['$x_{',num2str(i),'}$'],'Interpreter','latex')
    xlim([0,20])
    if i == 1
        legend_txt0 = {'$q_{1}$','$q_{2}$'};
    elseif i == 2
        legend_txt0 = {'$\dot{q}_{1}$','$\dot{q}_{2}$'};
    elseif i == 3
        legend_txt0 = {'$q_{m1}$','$q_{m2}$'};
    else
        legend_txt0 = {'$\dot{q}_{m1}$','$\dot{q}_{m2}$'};
    end
    legend(legend_txt0,'Interpreter','latex')
    if i==1
        title('System States')
    end
    if i == 4
        ylim([-200,200])
    end
end

% plot - x1 tracking
figure
for i = 1:2
    legend_txt1 = cell(1,2);
    subplot(2,1,i)
    plot(t,x_data(:,i),'b-',t,ref_data(:,i),'r--','LineWidth',1.2)
    hold on
    grid on
    xlabel('Time (s)')
    ylabel(['$q_{',num2str(i),'} / q_{',num2str(i),'d}$'],'Interpreter','latex')
    xlim([0,20])
    for j = 1:2
        if j==1
            legend_txt1{j} = ['$q_{',num2str(i),'}$'];
        else
            legend_txt1{j} = ['$q_{',num2str(i),'d}$'];
        end
    end
    legend(legend_txt1{1:2},'Interpreter','latex')
    if i==1
        title('x_{1} Tracking Performance')
    end    
end

% plot - e1
figure
for i = 1:2
    subplot(2,1,i)
    plot(t,e1_data(:,i),'b-','LineWidth',1.2)
    hold on
    grid on
    xlabel('Time (s)')
    ylabel(['$e_{1',num2str(i),'}$'],'Interpreter','latex')
    xlim([0,20])
    legend(['$e_{1',num2str(i),'}$'],'Interpreter','latex')
    if i==1
        title('Angular Tracking Error')
    end
end

% plot - z1
figure
for i = 1:2
    subplot(2,1,i)
    plot(t,z1_data(:,i),'b-','LineWidth',1.2)
    hold on
    grid on
    xlabel('Time (s)')
    ylabel(['$z_{1',num2str(i),'}$'],'Interpreter','latex')
    xlim([0,20])
    legend(['$z_{1',num2str(i),'}$'],'Interpreter','latex')
    if i==1
        title('Secondary Angular Tracking Error')
    end
end

% plot - filter
figure
for i = 1:2
    legend_txt2 = cell(1,2);
    subplot(2,1,i)
    plot(t,filter_out_data(:,i),'b-',t,alpha_1_data(:,i),'r--','LineWidth',1.2)
    hold on
    grid on
    xlabel('Time (s)')
    ylabel(['$\alpha_{1',num2str(i),'} / p_{',num2str(i),'}$'],'Interpreter','latex')
    xlim([0,20])    
    for j = 1:2
        if j==1
            legend_txt2{j} = ['$\alpha_{1',num2str(i),'}$'];
        else
            legend_txt2{j} = ['$p_{',num2str(i),'d}$'];
        end
    end
    legend(legend_txt2{1:2},'Interpreter','latex')
    if i==1
        title('Filter Performance')
    end 
end

% plot - u
figure
plot(t,u_data(:,1),'b-',t,u_data(:,2),'r-','LineWidth',1.2)
hold on
grid on
xlabel('Time (s)')
ylabel('$u$','Interpreter','latex')
legend('$u_{1}$','$u_{2}$','Interpreter','latex')
xlim([0,20]) 
title('Control Input')

% plot - W_hat
figure
plot(t,W_hat_data(:,1),'b-','LineWidth',1.2)
hold on
grid on
xlabel('Time (s)')
ylabel('$\Vert\hat{W}\Vert_{2}$','Interpreter','latex')
xlim([0,20]) 
title('NN Weight Estimate')

