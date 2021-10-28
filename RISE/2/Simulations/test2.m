clear
clc

p = [-20,-10,-6];
a = poly(p);
aa = -flip(a(2:end));

A = [zeros(2) eye(2) zeros(2);zeros(2) zeros(2) eye(2);aa(1)*eye(2) aa(2)*eye(2) aa(3)*eye(2)];
B = [zeros(2);zeros(2);-aa(1)*eye(2)];
C = [eye(2) zeros(2) zeros(2)];
D = zeros(2);

setlmis([])
P = lmivar(1,[6 1]);
Q = lmivar(1,[6 1]);
epsilon_squre = lmivar(1,[1 1]);
lmiterm([-1 1 1 P],1,1)
lmiterm([-2 1 1 Q],1,1)
lmiterm([-3 1 1 epsilon_squre],1,1)
lmiterm([4 1 1 P],A',1,'s')
lmiterm([4 1 1 epsilon_squre],1,(C')*C)
lmiterm([4 1 1 Q],1,1)

lmiterm([4 1 2 P],1,A+B*C)
lmiterm([4 1 3 P],1,-1)
lmisys = getlmis;

[tmin,xfeas] = feasp(lmisys,[0,1e2,1e9,10,1],0);
PP = dec2mat(lmisys,xfeas,P)
QQ = dec2mat(lmisys,xfeas,Q)
EPSILON_SQURE = dec2mat(lmisys,xfeas,epsilon_squre)
