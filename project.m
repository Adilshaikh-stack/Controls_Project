
%% declarig the parameters in the transfer fucntion of cart and pendulum
M = 0.6;
m = 0.2;
b = 0.2;
I = 0.003;
g = 9.8;
l = 0.25;
c = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
%% pendulum transfer function
G_pend = (m*l*s/c)/(s^3 + (b*(I + m*l^2))*s^2/c - ((M + m)*m*g*l)*s/c - b*m*g*l/c)

%% Response of pendulum position 
% definning PID controller

Kp = 1; %proportional gain
Ki = 0; %integral gain
Kd = 0; %differnetial gain

%PID controller
Cntroler = pid(Kp,Ki,Kd)
%System feedback loop
sys = feedback(G_pend,Cntroler)

%impulse response of the system input  = u(s) = 1 netwon force
figure(1)
impulse(sys)
title({'Response of the pendulum with the impulse diturbance under ';' PID: Kp = 1, Ki = 0, Kd = 0'})
ylabel('Amplitude (radian)')
grid on

%% 
Kp = 100; %proportional gain
Ki = 1; %integral gain
Kd = 1; %differnetial gain

Cntroler = pid(Kp,Ki,Kd)
sys = feedback(G_pend,Cntroler)

figure(2)
impulse(sys)
title({'Response of the pendulum with the impulse diturbance under ';' PID: Kp = 100, Ki = 1, Kd = 1'})
ylabel('Amplitude (radian)')
grid on

%%
Kp = 400; %proportional gain
Ki = 1; %integral gain
Kd = 1; %differnetial gain

Cntroler = pid(Kp,Ki,Kd)
sys = feedback(G_pend,Cntroler)
figure(3)
t = 0:0.01:10
impulse(sys,t)
title({'Response of the pendulum with the impulse diturbance under ';' PID: Kp = 400, Ki = 1, Kd = 1'})
ylabel('Amplitude (radian)')
grid on

%%
Kp = 100; %proportional gain
Ki = 0; %integral gain
Kd = 30; %differnetial gain

Cntroler = pid(Kp,Ki,Kd)
sys = feedback(G_pend,Cntroler)
figure(4)
impulse(sys)
title({'Response of the pendulum with the impulse diturbance under ';' PID: Kp = 100, Ki = 0, Kd = 30'})
ylabel('Amplitude (radian)')
grid on

%% Response of carts postion under PID 
G_cart = (((I+m*l^2)/c)*s^2 - (m*g*l/c))/(s^4 + (b*(I + m*l^2))*s^3/c - ((M + m)*m*g*l)*s^2/c - b*m*g*l*s/c);
Kp = 100; %proportional gain
Ki = 0; %integral gain
Kd = 30; %differnetial gain
Cntroler = pid(Kp,Ki,Kd)

sys = feedback(1,G_pend*Cntroler)*G_cart;
t = 0:0.01:5;
figure(5)
impulse(sys, t);
title({'Response of Cart Position to an Impulse Disturbance under ';' PID: Kp = 100, Ki = 0, Kd = 30'});
grid on


