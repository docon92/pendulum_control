%% LQR
% we take the full model, add integrator states and directly
%calculate optimal gains:
m = 1.0;
M = 5.0;
l = 1.0;
g = 9.807;

c1 = (M+m)*g/(M*l);
c2 = -m*g/M;
c3 = -1.0/(M*l);
c4 = 1.0/M;


A = [0 1 0 0
     c1 0 0 0
     0 0 0 1
     c2 0 0 0]
B = [0;c3;0;c4]

A_aug = [A zeros(4,1)
         0 0 -1 0 0]
B_aug = [B;0]
C = [1 0 0 0
     0 1 0 0
     0 0 0 1];

rank(ctrb(A_aug,B_aug))
   
Q_aug = diag([100 0 0.1 0 100]);
M = sqrtm(Q_aug);
rank(obsv(A_aug,M))
R = 1;
K = lqr(A_aug,B_aug,Q_aug,R)

Bp = [0.2;0;0;0;0];
Cp = [1 0 0 0 0
      0 0 1 0 0];

Dp = 0;
A_CL = A_aug-B_aug*K;

 sysCL = ss(A_CL,Bp,Cp,Dp);
 step(sysCL)
 impulse(sysCL)
