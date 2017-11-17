M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

%Unstable
%Kp = 10;
%Ki = 0;
%Kd = 0;

%Underdamped
%Kp = 100;
%Ki = 1;
%Kd = 1;

%Overdamped
%Kp = 100;
%Ki = 1;
%Kd = 20;
%axis([0, 2.5, -0.2, 0.2]);

C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);
t=0:0.01:10;
impulse(T,t)

title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 100, Ki = 1, Kd = 20'});
