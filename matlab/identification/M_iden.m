% M系列信号によるシステム同定
%% 入力信号の確認
figure(1);
input = csvread('input.csv');
stairs(0:127,input,'LineWidth',2);

hold on;
grid on;

N = 127;
input = idinput(N+1,'prbs',[0,1],[-3.0,3.0]);
stairs(0:N,input,'LineWidth',1)

%% M系列信号入力と応答
input_iden = csvread('input_iden.csv');
output_iden = csvread('output_iden.csv');

for i = 1:1:1271
    if input_iden(i) > 0
        input_iden(i) = 3.0;
    else
        input_iden(i) = -3.0;
    end
end
figure(2);
plot(input_iden);
grid on;

figure(3);
plot(output_iden);
grid on;
xlabel('Time n','FontSize',20);
ylabel('output [rad/s]','FontSize',20);
xlim([0 1271]);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;
grid on
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