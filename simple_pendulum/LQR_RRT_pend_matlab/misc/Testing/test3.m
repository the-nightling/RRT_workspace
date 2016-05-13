clear;
clf;
g = 9.8;
b = 0;
l = 1;
m = 1;

A = [0 1; g/l -b/(m*l*l)];
B = [0; 1/(m*l*l)];
C = [1 1];
D = [0];

Q = diag([1 10]);
R = 1;

[K,S] = lqr(A,B,Q,R);

Ac = [(A-B*K)];
Bc = [0;1];
Cc = [C];
Dc = [D];

states = {'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:20;
r =0.0*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t, [-pi 0]);

%plot(t,y(:,1));
plot(x(:,1),x(:,2));
