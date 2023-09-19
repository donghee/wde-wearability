clc
clear all
close all

%% 각도변수 

% q_u = 상완 접촉부 비틀림 각도(디바이스 상완과 인체 상완 사이 각도)
% q_f = 전완 접촉부 비틀림 각도(디바이스 전완과 인체 전완 사이 각도)

syms q_u q_f  

%% 전역

global a_u b_u a_f b_f m_d_u m_d_f q_u_0 q_f_0 K_u K_f L_h_u L_h_f m_h_u m_h_f g
global L_d_u L_d_f K_t_u K_t_f l_s_u_0 l_s_f_0 d_d h_d a G_h_u G_h_f G_d_u G_d_f
global i_q_u i_q_f

%% 베이스링크 각도 INPUT

a = deg2rad(0) % deg 단위로 원하는 값 넣어주세요. ex) 0 45 90 180

%% 디바이스 각도 INPUT

d_d = deg2rad(175) ; % deg 단위로 원하는 값 넣어주세요. ex) 0 45 90 180

%% 디바이스 상수 - 디바이스에서 주어지는 값

a_u = 0.175; % 디바이스 관절에서 상완 접촉부까지 최소거리(m)
b_u = 0.225;  % 디바이스 관절에서 상완 접촉부까지 최대거리(m)
a_f = 0.175; % 디바이스 관절에서 전완 접촉부까지 최소거리(m)
b_f = 0.225;  % 디바이스 관절에서 전완 접촉부까지 최대거리(m)

Real_L_d_u = 0.31; % 실제 디바이스 상완 길이(m)
Real_L_d_f = 0.31; % 실제 디바이스 상완 길이(m)

m_d_u = 2; % 디바이스 상완 질량(kg)
m_d_f = 2; % 디바이스 전완 질량(kg)

%% 신체상수 - 신체정보에서 주어지는 값

q_u_0 = 0; % 신체 상완 비틀림 스프링 각도 초기값(rad)
q_f_0 = 0; % 신체 전완 비틀림 스프링 각도 초기값(rad)
K_u = 10000; % 신체 상완 길이방향 스프링강성(N/m)
K_f = 10000; % 신체 전완 길이방향 스프링강성(N/m)
K_t_u = 10000; % 신체 상완 비틀림 스프링강성(N*m/rad)
K_t_f = 10000; % 신체 전완 비틀림 스프링강성(N*m/rad)
L_h_u = 0.2817; % 신체 상완 길이(m)
L_h_f = 0.2689; % 신체 전완 길이(m)
m_h_u = 1.9783; % 신체 상완 질량(kg)
m_h_f = 1.1826; % 신체 전완 질량(kg)

%% 무게중심 위치 - 인체관련 논문 참고.

G_h_u = 0.4228; % 신체 상완 무게중심 위치
G_h_f = 0.4574; % 신체 전완 무게중심 위치
G_d_u = 0.5; % 로봇 상완 무게중심 위치
G_d_f = 0.5; % 로봇 전완 무게중심 위치

g = 9.81; % 중력가속도(m/s^2)

%% 접촉부 중심 위치 - 디바이스 상수를 통해 계산. 결국 상수값.

L_d_u = (a_u + b_u) / 2; % 디바이스 상단 길이(m)
L_d_f = (a_f + b_f) / 2; % 디바이스 하단 길이(m)

%% 스프링길이 초기값 설정(고정부 착용위치)

% 착용오차를 정의할 수 있음.
% offset < 0 --> -
% offset > 0 --> +
    
l_s_u_0 = L_h_u - L_d_u ; % 신체 상완 스프링 길이 초기값(m) 
l_s_f_0 = L_h_f - L_d_f - 0.01; % 신체 전완 스프링 길이 초기값(m)

%% Newton Raphson Method - 뉴턴-랩슨법. ff_function과 jj_function을 통해 계산.
% 적절한 초기값이 대입되도록 반복

i_q_u = 5; % q_u의 초기값
i_q_f = 5; % q_f의 초기값
test = 0; % 발산 여부를 의미

while(test == 0) % test값이 1이 되도록 반복
  
    X=[deg2rad(i_q_u) deg2rad(i_q_f)]'; % 바뀐 초기값 입력.
    i=0;
    err=max(abs(ff_func(X)));
    tol=1e-4;  % 근사해의 오차가 0.0001이하로 나오도록
    fprintf('i        error\n')
    
    while err>tol % 근사해 찾는 반복문
        i=i+1; 
        X1=X - jj_func(X)\ff_func(X);
        X=X1;   
        err=max(abs(ff_func(X)));
        fprintf('%d       %f\n',i,err)
    
        error(i,1) = err;
    end

    % 아래는 계산과정에서 발산한적이 있었는지 판단
    for n = 1:1:i-1
        d(n,1) = error(n+1,1) - error(n,1);
    end
    
    D = d < 0;
    if(all(D(:)) == 1)
        test = 1 % test값이 1이면 계산과정 중 발산하지 않음.
    else 
        test = 0 % test값이 0이면 계산과정 중 발산함.
        if(i_q_u >= -5)
            i_q_u = i_q_u - 0.1; % q_u 초기값을 0.1씩 감소
        elseif(i_q_u < -5)
            i_q_f = i_q_f - 0.1; % q_f 초기값을 0.1씩 감소
            i_q_u = 5;
        end
        error = 0; d = 0; D = 0; % 변수값들 초기화
        i_q_u, i_q_f % 개선된 초기값 출력
    end

end

%% 계산결과

% 최적화를 통해 출력된 각도값
q_u = X(1); 
q_f = X(2);

% 각도 변수 관계: 인체팔꿈치 각도 = 디바이스팔꿈치 각도 - 상완비틀림각도 - 전완비틀림각도
h_d = d_d - q_u - q_f;

% 상완,전완 접촉부 인장스프링 길이
l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u);
l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u);

% 인체 팔꿈치 좌표값(x,y)
x = (L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*cos(a))/sin(q_f - d_d + q_u);
y = (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u);

%% 시각화 - 각 좌표점 위치를 통해 시각화

x_d_u = L_d_u * cos(a + 0); % 디바이스 상완 x축 좌표값
y_d_u = L_d_u * sin(a + 0); % 디바이스 상완 y축 좌표값

x_d_f = L_d_f * cos(a + d_d); % 디바이스 전완 x축 좌표값
y_d_f = L_d_f * sin(a + d_d); % 디바이스 전완 y축 좌표값

x_h_f = x + L_h_f * cos(a + q_u + h_d); % 인체 전완 x축 좌표값
y_h_f = y + L_h_f * sin(a + q_u + h_d); % 인체 전완 y축 좌표값

x_h_u = x + L_h_u * cos(a + q_u); % 인체 상완 x축 좌표값
y_h_u = y + L_h_u * sin(a + q_u); % 인체 전완 y축 좌표값

figure(1)
set(figure(1),'position',[700,300,400,250])
set(gca,'GridColor','black')

xlabel('X[m]');
ylabel('Y[m]');
xticks(-0.4:0.1:0.4)
xlim([-0.4 0.4]);
yticks(-0.1:0.1:0.4)
ylim([-0.1 0.4]);
hold on;
grid on;
line([0 x_d_f],[0 y_d_f],'color','black','LineWidth',1); 
line([0 x_d_u],[0 y_d_u],'color','black','LineWidth',1);
line([x x_h_f],[y y_h_f],'color','green','LineWidth',1);
line([x x_h_u],[y y_h_u],'color','green','LineWidth',1);
plot(x,y,'g.','MarkerSize',30);
%text(x,y,'H.E.J','Color','green');
plot(x_d_f,y_d_f,'g.','MarkerSize',30);
text(x_d_f,y_d_f,'B');
plot(x_d_u,y_d_u,'g.','MarkerSize',30);
text(x_d_u,y_d_u,'A');
plot(0,0,'black.','MarkerSize',30);
%text(0,0,'D.E.J');

%% 각도 및 변형량 출력

deg_q_u = round(rad2deg(q_u),4)
deg_q_f = round(rad2deg(q_f),4)
deg_device = round(rad2deg(d_d),4)
deg_human = round(rad2deg(h_d),4)
mm_def_l_s_u = round(1000 * (l_s_u - l_s_u_0) , 4)
mm_def_l_s_f = round(1000 * (l_s_f - l_s_f_0) , 4)

%% 안전성 점수

Mu = K_t_u*q_u;
Mf = K_t_f*q_f;
Fu = K_u*(l_s_u - l_s_u_0);
Ff = K_u*(l_s_f - l_s_f_0);
A = rad2deg(h_d) - rad2deg(d_d);

% 안전성에 대한 정의 
% 100N,100Nm 이상의 하중이 측정되면 안전성을 10^(-10)으로 정의
if(abs(Fu)>100)
    SFu = 10^(-10);
else
    SFu = (-1/100)*abs(Fu)+1;
end

if(abs(Mu)>100)
    SMu = 10^(-10);
else
    SMu = (-1/100)*abs(Mu)+1;
end

if(abs(Ff)>100)
    SFf = 10^(-10);
else
    SFf = (-1/100)*abs(Ff)+1;
end

if(abs(Mf)>100)
    SMf = 10^(-10);
else
    SMf = (-1/100)*abs(Mf)+1;
end

% 팔꿈치 각도에 따른 안전성 점수
% 4도 이상의 팔꿈치 각도차가 보이면 안전성을 10^(-10)으로 정의
if(rad2deg(h_d)>180)
    Sq = 10^(-10);
elseif(abs(A)>4)
    Sq = 10^(-10);
else
    Sq = (-1/4)*abs(A)+1;
end

% 안전성 종합점수 최소 0점, 최대 1점
St  = SFu*SMu*SFf*SMf*Sq;
if(St<=10^(-10))
    Stotal = 0;
else
    Stotal = 0.1*(10 + log(St));
end

safety_Fu = round(SFu,4)
safety_Mu = round(SMu,4)
safety_Ff = round(SFf,4)
safety_Mf = round(SMf,4)
safety_Total = round(Stotal,4)
