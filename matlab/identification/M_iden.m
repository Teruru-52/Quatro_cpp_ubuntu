% M系列信号によるシステム同定
%% M系列信号入力と応答
data = csvread('data.csv');

input = data(:, 1);
output = data(:, 2);

figure(1);
subplot(2,1,1);
plot(input,'LineWidth',2);
grid on
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$u_\omega$ [V]','Interpreter','latex','FontSize',20);
xlim([0 1270]);
ylim([-1.7 1.7]);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;

subplot(2,1,2);
plot(output,'LineWidth',2);
grid on;
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',20);
xlim([0 1270]);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;
%% システム同定
systemIdentification

%% 
Ts = 0.001;
Num = tf1.Numerator;
Den = tf1.Denominator;
% sys = tf(Num, Den)
ff = Den / Num;
tf1
% dis_tf1 = c2d(sys,Ts,'zoh')
%%
Ts = 0.001;
Num2 = tf2.Numerator;
Den2 = tf2.Denominator;
% sys2 = tf(Num2, Den2)
ff2 = Den2 / Num2
tf2
% dis_tf2 = c2d(sys2,Ts)
%%
pidTuner()