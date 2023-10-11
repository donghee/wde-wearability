clc
clear 
close all

%% import - result.txt로 계산데이터 저장

G1 = importdata('result.txt');

%% input - 원하는 각도 1도 단위로 입력(베이스링크, 팔꿈치)

device_baselink_angle = 0
device_elbow_angle = 175

%% 상수 73kg, 1741mm 남성

L_h_u = 0.2817; % 신체 상완 길이(m)
L_h_f = 0.2689; % 신체 전완 길이(m)
L_d_u = 0.2; % 디바이스 상완 길이(m)
L_d_f = 0.2; % 디바이스 전완 길이(m)

m_h_u = 1.9783; % 신체 상완 질량(kg)
m_h_f = 1.1826; % 신체 전완 질량(kg)
m_d_u = 2; % 디바이스 상완 질량(kg)
m_d_f = 2; % 디바이스 전완 질량(kg)

G_h_u = 0.4228; % 신체 상완 무게중심 위치
G_h_f = 0.4574; % 신체 전완 무게중심 위치
G_d_u = 0.5; % 로봇 상완 무게중심 위치
G_d_f = 0.5; % 로봇 전완 무게중심 위치

K_u = 10000; % 신체 상완 길이방향 스프링강성(N/m)
K_f = 10000; % 신체 전완 길이방향 스프링강성(N/m)
K_t_u = 10000; % 신체 상완 비틀림 스프링강성(N*m/rad)
K_t_f = 10000; % 신체 전완 비틀림 스프링강성(N*m/rad)
q_u_0 = 0; % 신체 상완 비틀림 스프링 각도 초기값(rad)
q_f_0 = 0; % 신체 전완 비틀림 스프링 각도 초기값(rad)

g = 9.81; % 중력가속도(m/s^2)

l_s_u_0 = L_h_u - L_d_u ; % 신체 상완 스프링 길이 초기값(m) 
l_s_f_0 = L_h_f - L_d_f ; % 신체 전완 스프링 길이 초기값(m)

%% 각 변수들 상관관계(수학적 모델링에서 도출되는 수식)

% result에 저장된 값 찾아서 호출
q_u = deg2rad(G1(find(G1(:,1)==device_baselink_angle&G1(:,2)==device_elbow_angle),4));  
q_f = deg2rad(G1(find(G1(:,1)==device_baselink_angle&G1(:,2)==device_elbow_angle),5)); 

d_d = deg2rad(device_elbow_angle);

l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u);
l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u);

h_d = d_d - q_u - q_f;

baselink_angle = deg2rad(device_baselink_angle);

%% calculate safety -

Mu = K_t_u*q_u;
Mf = K_t_f*q_f;
Fu = K_u*(l_s_u - l_s_u_0);
Ff = K_u*(l_s_f - l_s_f_0);
AS = rad2deg(h_d) - rad2deg(d_d); % elbow joint 유사성

if(abs(Fu)>10)
    SFu = 10^(-10);
else
    SFu = (-1/10)*abs(Fu)+1;
end

if(abs(Mu)>10)
    SMu = 10^(-10);
else
    SMu = (-1/10)*abs(Mu)+1;
end

if(abs(Ff)>10)
    SFf = 10^(-10);
else
    SFf = (-1/10)*abs(Ff)+1;
end

if(abs(Mf)>10)
    SMf = 10^(-10);
else
    SMf = (-1/10)*abs(Mf)+1;
end

%

if(abs(AS)>4)
    Sa = 10^(-10);
else
    Sa = (-1/4)*abs(AS)+1;
end

%

St  = SFu*SMu*SFf*SMf*Sa;
if(St<=10^(-10))
    Stotal = 0;
else
    Stotal = 0.1*(10 + log(St));
end

%% figure 1 - human posture prediction

x = (L_d_f*cos(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*cos(baselink_angle))/sin(q_f - d_d + q_u);
y = (L_d_f*sin(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(baselink_angle))/sin(q_f - d_d + q_u);

x_d_u = L_d_u * cos(baselink_angle + 0); % 디바이스 상완 x축 좌표값
y_d_u = L_d_u * sin(baselink_angle + 0); % 디바이스 상완 y축 좌표값

x_d_f = L_d_f * cos(baselink_angle + d_d); % 디바이스 전완 x축 좌표값
y_d_f = L_d_f * sin(baselink_angle + d_d); % 디바이스 전완 y축 좌표값

x_h_f = x + L_h_f * cos(baselink_angle + q_u + h_d); % 인체 전완 x축 좌표값
y_h_f = y + L_h_f * sin(baselink_angle + q_u + h_d); % 인체 전완 y축 좌표값

x_h_u = x + L_h_u * cos(baselink_angle + q_u); % 인체 상완 x축 좌표값
y_h_u = y + L_h_u * sin(baselink_angle + q_u); % 인체 전완 y축 좌표값

set(figure(1),'position',[50,550,400,250])
set(gca,'GridColor','black')
     
title('Posture Predicton')

xlabel('X[m]');
ylabel('Y[m]');
xticks(-0.4:0.1:0.4)
xlim([-0.4 0.4]);
yticks(-0.1:0.1:0.4)
ylim([-0.1 0.4]);

hold on;
grid on;

line([0 x_d_f],[0 y_d_f],'color','black','LineWidth',1); 
line([x x_h_f],[y y_h_f],'color','green','LineWidth',1);
line([0 x_d_u],[0 y_d_u],'color','black','LineWidth',1);
line([x x_h_u],[y y_h_u],'color','green','LineWidth',1);

plot(x,y,'g.','MarkerSize',30);
plot(x_d_f,y_d_f,'g.','MarkerSize',30);
plot(x_d_u,y_d_u,'g.','MarkerSize',30);
plot(0,0,'black.','MarkerSize',30);

legend({'device','human'})

txt1 = ['device elbow angle : ' num2str(round(rad2deg(d_d),4)) '°'];
txt2 = ['human elbow angle : ' num2str(round(rad2deg(h_d),4)) '°'];
text(-0.39,0.375,txt1)
text(-0.39,0.325,txt2)

%% figure 2 - interaction force

set(figure(2),'position',[500,550,400,400])
set(gca,'GridColor','black')
title('Interaction Force')

colororder({'red','blue'})

yyaxis left
ylabel('moment[Nm]');
yticks(-10:2:10)
ylim([-10 10])

yyaxis right
ylabel('force[N]')
yticks(-10:2:10)
ylim([-10 10])

hold on;
grid on;

X1 = categorical({'Mu','Fu','Mf','Ff'});
X1 = reordercats(X1,{'Mu','Fu','Mf','Ff'});
Y1 = round([Mu Fu Mf Ff],2);

bar1 = bar(X1,Y1,'FaceColor','flat');

xtips1 = bar1(1).XEndPoints;
ytips1 = bar1(1).YEndPoints;
labels1 = string(bar1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom');

bar1.CData(1,:) = [1 0 0];
bar1.CData(2,:) = [0 0 1];
bar1.CData(3,:) = [1 0 0];
bar1.CData(4,:) = [0 0 1];

yline(0,'color','black','lineStyle','-')

%% figure 3 -  result of device safety

set(figure(3),'position',[950,550,400,400])
set(gca,'GridColor','black')
title('Result of Device Safety')

hold on;
grid on;

X2 = categorical({'Mu','Fu','Mf','Ff','AS','Total'});
X2 = reordercats(X2,{'Mu','Fu','Mf','Ff','AS','Total'});
Y2 = [SMu SFu SMf SFf Sa Stotal];

bar2 = bar(X2,Y2,'FaceColor','flat');

xtips1 = bar2(1).XEndPoints;
ytips1 = bar2(1).YEndPoints;
labels1 = string(bar2(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom');

ylabel('Safety Score')
yticks(0:0.1:1)
ylim([0 1])

bar2.CData(1,:) = [1 0 0];
bar2.CData(2,:) = [0 0 1];
bar2.CData(3,:) = [1 0 0];
bar2.CData(4,:) = [0 0 1];
bar2.CData(5,:) = [0 1 0];
bar2.CData(6,:) = [0 0 0];

%% figure 4 - spider plot

figure(4)
set(figure(4),'position',[1400,550,400,400])

% Initialize data points
D1 = [SMu SFu SMf SFf Sa];
D2 = [0 0 0 0 0]; % 나중에 여러 그래프를 동시에 plot하고 싶을 때 사용 
D3 = [0 0 0 0 0]; 
P = [D1; D2; D3];

% Spider plot
spider_plot(P,...
    'AxesLabels', {'Mu', 'Fu', 'Mf', 'Ff', 'A.S.'},...
    'AxesInterval', 4,...
    'AxesPrecision', 1,...
    'AxesDisplay', ['one'],...
    'AxesLimits', [0, 0, 0, 0, 0; 1, 1, 1, 1, 1],...
    'FillOption', 'on',...
    'FillTransparency', 0.2,...
    'Color', [1, 0, 0; 0, 0, 0; 0, 0, 0],...
    'LineStyle', {'-', '-', '-'},...
    'LineWidth', [1, 1, 1],...
    'LineTransparency', 1,...
    'Marker', {'o', 'o', 'o'},...
    'MarkerSize', [10, 10, 10],...
    'MarkerTransparency', 1,...
    'AxesFont', 'Times New Roman',...
    'LabelFont', 'Times New Roman',...
    'AxesFontSize', 8,...
    'LabelFontSize', 10,...
    'Direction', 'clockwise',...
    'AxesDirection', {'noraml', 'normal', 'normal', 'normal', 'normal'},...
    'AxesLabelsOffset', 0.1,...
    'AxesDataOffset', 0.1,...
    'AxesScaling', 'linear',...
    'AxesColor', [0.6, 0.6, 0.6],...
    'AxesLabelsEdge', 'none',...
    'AxesOffset', 0,...
    'AxesZoom', 0.6,...
    'AxesHorzAlign', 'quadrant',...
    'AxesVertAlign', 'quadrant',...
    'PlotVisible', 'on',...
    'AxesTickLabels', 'data',...
    'AxesInterpreter', 'tex',...
    'BackgroundColor' , 'w',...
    'MinorGrid', 'off',...
    'MinorGridInterval', 2,...
    'AxesZero', 'off',...
    'AxesZeroColor', 'k',...
    'AxesZeroWidth', 2,...
    'AxesRadial', 'on',...
    'AxesAngular', 'on',...
    'AxesShaded', 'off',...
    'AxesShadedLimits', [],...
    'AxesShadedColor', 'g',...
    'AxesShadedTransparency', 0.2,...
    'AxesLabelsRotate', 'off');

hold on;

txt3 = ['Total Safety Score : ' num2str(Stotal)];
text(-0.7,-1.2,txt3)

% Title and legend settings
title(sprintf('Result of Device Safety'));
%legend_str = {'ideal'};
%legend(legend_str, 'Location', 'northwest');

%% print

disp('------------------------------')
disp('Output Parameter')

disp(sprintf('device elbow angle : %d°',d_d))
disp(sprintf('human elbow angle : %d°',h_d))

disp(sprintf('Interaction Force(M.u.) : %dNm',Mu))
disp(sprintf('Interaction Force(F.u.) : %dN',Fu))
disp(sprintf('Interaction Force(M.f.) : %dNm',Mf))
disp(sprintf('Interaction Force(F.f.) : %dN',Ff))

disp(sprintf('Safety Score(M.u.) : %d',SMu))
disp(sprintf('Safety Score(F.u.) : %d',SFu))
disp(sprintf('Safety Score(M.f.) : %d',SMf))
disp(sprintf('Safety Score(F.f.) : %d',SFf))
disp(sprintf('Safety Score(AS) : %d',Sa))
disp(sprintf('Safety Score(Total) : %d',Stotal))