% ステップ応答によるシステム同定
%% ステップ応答
close all
clear

output = csvread('data.csv');

figure(1);
plot(output,'LineWidth',3);
grid on
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;
%% 
pidTuner()