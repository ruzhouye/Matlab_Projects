clear
clc
p = [-100,-50,-30];
a = poly(p);
aa = -flip(a(2:end));

A = [zeros(2) eye(2) zeros(2);zeros(2) zeros(2) eye(2);aa(1)*eye(2) aa(2)*eye(2) aa(3)*eye(2)];
B = [zeros(2);zeros(2);-aa(1)*eye(2)];
C = [eye(2) zeros(2) zeros(2)];
D = zeros(2);

SYS = ss(A,B,C,D);

P = sdpvar(6,6,'symmetric');
Q = sdpvar(6,6,'symmetric');
epsilon_squre = sdpvar(1,1);
e2 = sdpvar(1,1);
e1 = 0;
Psi = [(A')*P+P*A+epsilon_squre*(C')*C+Q, P*(A+B*C), -P;
       (A'+(C')*(B'))*P, -e1*eye(6), zeros(6);
       -P, zeros(6), -e2*eye(6)];
Constraints = [e2>=0,P>=0,Q>=0,epsilon_squre>=0,Psi<=0];
tol = 1e-6;
opt = sdpsettings('solver', 'sedumi', 'sedumi.eps',tol);
diagnostics_lmi = optimize(Constraints,[],opt);
tmin = diagnostics_lmi.problem;
if tmin == 0
    P_value = value(P);
    Q_value = value(Q);
    epsilon_squre_value = value(epsilon_squre);
    e1_value = value(e1);
    e2_value = value(e2);
else
    P_value = [];
    Q_value = [];
    epsilon_squre_value = [];
    e1_value = [];
    e2_value = [];
end
