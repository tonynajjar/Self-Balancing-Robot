M=1;
g=9.81;
l=0.15;
r=0.065;
Jw=3.14*r^4/2;
W=0.06;
H=2*l;
Jp=M*(W^2/12+H^2/3);

A1=(-M^2*r*l^2*g)/((M*r^2+Jw)*(M*l^2+Jp)+M*r*l^2);
A2=(M*r^2+Jw)*(M*g*l)/(-(M*r^2+Jw)*(M*l^2+Jp)+(M*r*l)^2);

B1=1/((M*r^2+Jw)-((M*r*l)^2)/(M*l^2+Jp));
B2=1/(-(M*r^2+Jw)*(M*l^2+Jp)/(M*r*l)+M*r*l);

A=[0 0 1 0; 0 0 0 1; 0 A1 0 0; 0 A2 0 0];
B=[0;0;B1;B2];
C=eye(4);
D=[0;0;0;0];


C = pid(10,1)
G = ss(A,B,C,D);
closedLoop = feedback(G*C,1);