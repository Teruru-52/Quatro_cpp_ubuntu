%% M系列信号の生成
close all
clear

N = 127;

input = idinput(N+1,'prbs',[0,1],[-3.0,3.0]);
stairs(0:N,input,'LineWidth',2)
xlim([0 127]);
ylim([-4 4]);
xlabel('Time n','FontSize',20);
ylabel('input [V]','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;
grid on
%% plot
out = csvread('m_output.csv');
in = csvread('m_input.csv');

out2 = out * pi /180; % [deg/sec] to [rad/sec]
L = length(out);
t = 0:0.02:(L-1)*0.02;

figure(1);
plot(t,in);
grid on;

figure(2);
plot(t,out2);
grid on;