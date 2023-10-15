clc
clear
close all

%% import - result.txt로 계산데이터 저장

G1 = importdata('result.txt');

%% input - 원하는 각도 1도 단위로 입력(고관절, 무릎)

human_hip_angle = 30
human_knee_angle = 30

%% 상수 73kg, 1741mm 남성

L_hw = 0.1457; % 인체 허리 길이(m)
L_ht = 0.4222; % 인체 허벅지 길이(m)
L_hs = 0.4340; % 인체 정강이 길이(m)
L_rw = 0.1; % 로봇 허리 길이(m)
L_rt = 0.4222; % 로봇 허벅지 길이(m)
L_rs = 0.35; % 로봇 정강이 길이(m)
L_rtw = 0.2111; % 인체 허리 쪽 허벅지 길이(m)
L_rts = 0.2111; % 인체 정강이 쪽 허벅지 길이(m)

m_hw = 8.1541; % 인체 허리 무게(kg)
m_ht = 10.3368; % 인체 허벅지 무게(kg)
m_hs = 3.7518; % 인체 정강이 무게(kg) 
m_rw = 5.0000; % 로봇 허리 무게(kg) 
m_rt = 4.2220; % 로봇 허벅지 무게(kg) 
m_rs = 3.5000; % 로봇 정강이 무게(kg)

G_hw = 0.3885; % 인체 허리 무게중심
G_ht = 0.4095; % 인체 허벅지 무게중심
G_hs = 0.4459; % 인체 정강이 무게중심 
G_rw = 0.5; % 로봇 허리 무게중심
G_rt = 0.5; % 로봇 허벅지 무게중심
G_rs = 0.5; % 로봇 정강이 무게중심

K_w = 10000; % 인체 허리 길이방향 스프링강성(N/m)
K_t = 10000; % 인체 허벅지 길이방향 스프링강성(N/m)
K_s = 5000; % 인체 정강이 길이방향 스프링강성(N/m)
K_tw = 5000; % 인체 허리 비틀림 스프링강성(Nm/rad)
K_tt = 5000; % 인체 허벅지 비틀림 스프링강성(Nm/rad)
K_ts = 5000; % 인체 정강이 비틀림 스프링강성(Nm/rad)
g = 9.81; % 중력가속도 m/s^2

l_w_0 = L_hw - L_rw; % 인체 허리 길이방향 스프링 초기값(m) 
l_t_0 = L_rtw; % 인체 허벅지 길이방향 스프링 초기값(m)
l_s_0 = L_hs - L_rs; % 인체 정강이 길이방향 스프링 초기값(m)

%% 각 변수들 상관관계(수학적 모델링에서 도출되는 수식)

% result에 저장된 값 찾아서 호출
q_w = deg2rad(G1(find(G1(:,1)==human_hip_angle&G1(:,3)==human_knee_angle),5));  
q_t = deg2rad(G1(find(G1(:,1)==human_hip_angle&G1(:,3)==human_knee_angle),6));  

q_hh = deg2rad(human_hip_angle);  % 엉덩이 관절 각도
q_hk = deg2rad(human_knee_angle); % 무릎 관절 각도

q_s = -asin((L_rts*sin(q_hk + q_t) - L_ht*sin(q_hk) + (sin(q_hk)*(L_rtw*sin(q_hh + q_t) + L_rw*sin(q_w)))/sin(q_hh))/L_rs);
l_t = (L_rtw*sin(q_hh + q_t) + L_rw*sin(q_w))/sin(q_hh);
l_w = -(L_rw*sin(q_hh + q_w) - L_hw*sin(q_hh) + L_rtw*sin(q_t))/sin(q_hh);
l_s = -(L_rs*sin(q_hk + q_s) - L_hs*sin(q_hk) + L_rts*sin(q_t))/sin(q_hk);

q_rh = q_s + q_t + q_hh;
q_rk = q_w + q_t + q_hk;

%% calculate safety -

Mw = K_tw*q_w;
Mt = K_tt*q_t;
Ms = K_ts*q_s;

Fw = K_w*(l_w - l_w_0);
Ft = K_t*(l_t - l_t_0);
Fs = K_s*(l_s - l_s_0);

ASh = rad2deg(q_hh) - rad2deg(q_rh); % hip joint 유사성
ASk = rad2deg(q_hk) - rad2deg(q_rk); % knee joint 유사성

if(abs(Mw)>100)
    SMw = 10^(-10);
else
    SMw = (-1/100)*abs(Mw)+1;
end

if(abs(Mt)>100)
    SMt = 10^(-10);
else
    SMt = (-1/100)*abs(Mt)+1;
end

if(abs(Ms)>100)
    SMs = 10^(-10);
else
    SMs = (-1/100)*abs(Ms)+1;
end

if(abs(Fw)>100)
    SFw = 10^(-10);
else
    SFw = (-1/100)*abs(Fw)+1;
end

if(abs(Ft)>100)
    SFt = 10^(-10);
else
    SFt = (-1/100)*abs(Ft)+1;
end

if(abs(Fs)>100)
    SFs = 10^(-10);
else
    SFs = (-1/100)*abs(Fs)+1;
end

%

if(abs(ASh)>5)
    Sah = 10^(-10);
else
    Sah = (-1/5)*abs(ASh)+1;
end

if(abs(ASk)>5)
    Sak = 10^(-10);
else
    Sak = (-1/5)*abs(ASk)+1;
end

%

St  = SMw*SMt*SMs*SFw*SFt*SFs*Sah*Sak;
if(St<=10^(-10))
    Stotal = 0;
else
    Stotal = 0.1*(10 + log(St));
end

%% figure 1 - human posture prediction

x_hw = 0;
y_hw = L_hw;

x_rw = 0;
y_rw = L_hw - l_w;

x_hh = 0;
y_hh = 0;

x_rh = L_rw*cos(3*pi/2 + q_w); 
y_rh = L_hw - l_w + L_rw*sin(3*pi/2 + q_w);

xx_rh = l_t*cos(5*pi/2 - q_hh) + L_rtw*cos(3*pi/2 - q_hh - q_t);
yy_rh = l_t*sin(5*pi/2 - q_hh) + L_rtw*sin(3*pi/2 - q_hh - q_t);

x_hk = L_ht*cos(5*pi/2 - q_hh);
y_hk = L_ht*sin(5*pi/2 - q_hh);

x_rk = l_t*cos(5*pi/2 - q_hh) + L_rts*cos(5*pi/2 - q_hh - q_t);
y_rk = l_t*sin(5*pi/2 - q_hh) + L_rts*sin(5*pi/2 - q_hh - q_t);

xx_rk = L_ht*cos(5*pi/2 - q_hh) + (L_hs - l_s)*cos(3*pi/2 + q_hk - q_hh) + L_rs*cos(pi/2 + q_hk + q_s - q_hh);
yy_rk = L_ht*sin(5*pi/2 - q_hh) + (L_hs - l_s)*sin(3*pi/2 + q_hk - q_hh) + L_rs*sin(pi/2 + q_hk + q_s - q_hh);

x_hs = L_ht*cos(5*pi/2 - q_hh) + L_hs*cos(3*pi/2 + q_hk - q_hh);
y_hs = L_ht*sin(5*pi/2 - q_hh) + L_hs*sin(3*pi/2 + q_hk - q_hh);

x_rs = L_ht*cos(5*pi/2 - q_hh) + (L_hs - l_s)*cos(3*pi/2 + q_hk - q_hh);
y_rs = L_ht*sin(5*pi/2 - q_hh) + (L_hs - l_s)*sin(3*pi/2 + q_hk - q_hh);

set(figure(1),'position',[0,500,500,450])
set(gca,'GridColor','black')
     
title('Posture Predicton')

xlabel('X[m]');
ylabel('Y[m]');
xticks(-1.0:0.2:1.0)
xlim([-1 1]);
yticks(-1.0:0.2:1.0)
ylim([-1 1]);

hold on;
grid on;

line([x_hw x_hh],[y_hw y_hh],'color','g','LineWidth',1);
line([x_rw x_rh],[y_rw y_rh],'color','black','LineWidth',1);

line([x_hh x_hk],[y_hh y_hk],'color','g','LineWidth',1);
line([x_rh x_rk],[y_rh y_rk],'color','black','LineWidth',1);

line([x_hk x_hs],[y_hk y_hs],'color','g','LineWidth',1);
line([x_rk x_rs],[y_rk y_rs],'color','black','LineWidth',1);

plot(x_hw,y_hw,'g.','MarkerSize',30);
plot(x_hh,y_hh,'g.','MarkerSize',30);
plot(x_hk,y_hk,'g.','MarkerSize',30);
plot(x_hs,y_hs,'g.','MarkerSize',30);

plot(x_rw,y_rw,'black.','MarkerSize',30);
plot(x_rh,y_rh,'black.','MarkerSize',30);
plot(x_rk,y_rk,'black.','MarkerSize',30);
plot(x_rs,y_rs,'black.','MarkerSize',30);

legend({'device','human'})

txt1 = ['human hip angle : ' num2str(round(rad2deg(q_hh),4)) '°'];
txt2 = ['human knee angle : ' num2str(round(rad2deg(q_hk),4)) '°'];
txt3 = ['device hip angle : ' num2str(round(rad2deg(q_rh),4)) '°'];
txt4 = ['device knee angle : ' num2str(round(rad2deg(q_rk),4)) '°'];
text(-0.95,0.96,txt1)
text(-0.95,0.86,txt2)
text(-0.95,0.76,txt3)
text(-0.95,0.66,txt4)

%% figure 2 - interaction force 

figure(2)
set(figure(2),'position',[550,500,350,450])
set(gca,'GridColor','black')
title('Interaction Force')

colororder({'red','blue'})

yyaxis left
ylabel('Moment[Nm]');
yticks(-100:20:100)
ylim([-100 100])

yyaxis right
ylabel('Force[N]')
yticks(-100:20:100)
ylim([-100 100])

hold on;
grid on;

X1 = categorical({'Mw','Fw','Mt','Ft','Ms','Fs'});
X1 = reordercats(X1,{'Mw','Fw','Mt','Ft','Ms','Fs'});
Y1 = round([Mw Fw Mt Ft Ms Fs],2);

bar1 = bar(X1,Y1,'FaceColor','flat');

xtips1 = bar1(1).XEndPoints;
ytips1 = bar1(1).YEndPoints;
labels1 = string(bar1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom');

bar1.CData(1,:) = [1 0 0];
bar1.CData(2,:) = [0 0 1];
bar1.CData(3,:) = [1 0 0];
bar1.CData(4,:) = [0 0 1];
bar1.CData(5,:) = [1 0 0];
bar1.CData(6,:) = [0 0 1];

yline(0,'color','black','lineStyle','-')

%% figure 3 - result of device safety

set(figure(3),'position',[950,500,450,450])
set(gca,'GridColor','black')
title('Result of Device Safety')

hold on;
grid on;

X2 = categorical({'Mw','Fw','Mt','Ft','Ms','Fs','ASh','ASk','Total'});
X2= reordercats(X2,{'Mw','Fw','Mt','Ft','Ms','Fs','ASh','ASk','Total'});
Y2 = [SMw SFw SMt SFt SMs SFs Sah Sak Stotal];

bar2 = bar(X2,Y2,"FaceColor",'flat');

ylabel('Safety Score');
yticks(0:0.1:1);
ylim([0 1]);

bar2.CData(1,:) = [1 0 0];
bar2.CData(2,:) = [0 0 1];
bar2.CData(3,:) = [1 0 0];
bar2.CData(4,:) = [0 0 1];
bar2.CData(5,:) = [1 0 0];
bar2.CData(6,:) = [0 0 1];
bar2.CData(7,:) = [1 0 1];
bar2.CData(8,:) = [0 1 1];
bar2.CData(9,:) = [0 0 0];

%% figure 4 - result of deivce safety(spider plot)

figure(4)
set(figure(4),'position',[1450,500,450,450])

% Initialize data points
D1 = [SMw SFw SMt SFt SMs SFs Sah Sak];
D2 = [0 0 0 0 0 0 0 0]; % 나중에 여러 그래프를 동시에 plot하고 싶을 때 사용 
D3 = [0 0 0 0 0 0 0 0]; 
P = [D1; D2; D3];

% Spider plot
spider_plot(P,...
    'AxesLabels', {'Mw', 'Fw', 'Mt', 'Ft', 'Ms' ,'Fs', 'ASh' , 'ASk'},...
    'AxesInterval', 4,...
    'AxesPrecision', 1,...
    'AxesDisplay', ['one'],...
    'AxesLimits', [0, 0, 0, 0, 0, 0, 0, 0; 1, 1, 1, 1, 1, 1, 1, 1],...
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
    'AxesDirection', {'noraml', 'normal', 'normal', 'normal', 'normal', 'normal', 'normal', 'normal'},...
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

txt5 = ['Total Safety Score : ' num2str(Stotal)];
text(-0.7,-1.4,txt5)

% Title and legend settings
title(sprintf('Result of Device Safety'));
%legend_str = {'ideal'};
%legend(legend_str, 'Location', 'northwest');

%% print

disp('------------------------------')
disp('Output Parameter')

disp(sprintf('human hip angle : %d°',rad2deg(q_hh)));
disp(sprintf('human knee angle : %d°',rad2deg(q_hk)));
disp(sprintf('device hip angle : %d°',rad2deg(q_rh)));
disp(sprintf('device knee angle : %d°',rad2deg(q_rk)));

disp(sprintf('Interaction Force(Mw) : %dNm',Mw));
disp(sprintf('Interaction Force(Fw) : %dNm',Fw));
disp(sprintf('Interaction Force(Mt) : %dNm',Mt));
disp(sprintf('Interaction Force(Ft) : %dNm',Ft));
disp(sprintf('Interaction Force(Ms) : %dNm',Ms));
disp(sprintf('Interaction Force(Fs) : %dNm',Fs));

disp(sprintf('Safety Score(Mw) : %d',SMw))
disp(sprintf('Safety Score(Fw) : %d',SFw))
disp(sprintf('Safety Score(Mt) : %d',SMt))
disp(sprintf('Safety Score(Ft) : %d',SFt))
disp(sprintf('Safety Score(Ms) : %d',SMs))
disp(sprintf('Safety Score(Fs) : %d',SFs))
disp(sprintf('Safety Score(ASh) : %d',Sah))
disp(sprintf('Safety Score(ASk) : %d',Sak))
disp(sprintf('Safety Score(Total) : %d',Stotal))
