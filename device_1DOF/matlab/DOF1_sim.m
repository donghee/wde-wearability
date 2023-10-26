clc
clear all
close all

%% 각도변수

% q_u = 상완 접촉부 비틀림 각도(디바이스 상완과 인체 상완 사이 각도)
% q_f = 전완 접촉부 비틀림 각도(디바이스 전완과 인체 전완 사이 각도)

syms q_u q_f

%% 전역

global m_d_u m_d_f m_h_u m_h_f g q_u_0 q_f_0 K_u K_f K_t_u K_t_f
global L_d_u L_d_f L_h_u L_h_f l_s_u_0 l_s_f_0 d_d h_d G_h_u G_h_f G_d_u G_d_f
global baselink_angle

%% min각도 max각도 INPUT

interval = 1; % 해석 간격(deg 단위)

minimum_elbow_angle = 30; % deg 단위로 원하는 값 넣어주세요. ex) 0 45 90 180
maximum_elbow_angle = 180; % deg 단위로 원하는 값 넣어주세요. ex) 0 45 90 180

B = [0:interval:360];

E = [minimum_elbow_angle:interval:maximum_elbow_angle];
NE = find(E == 180);
E(:,NE) = [];

d_d = deg2rad(minimum_elbow_angle);

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

%% 스프링길이 초기값 설정(고정부 착용위치)

% 길게 착용했다면 -
% 짧게 착용했다면 +

l_s_u_0 = L_h_u - L_d_u ; % 신체 상완 스프링 길이 초기값(m)
l_s_f_0 = L_h_f - L_d_f ; % 신체 전완 스프링 길이 초기값(m)

%% Newton Raphson Method - 뉴턴-랩슨법 min ~ max 각도 전체해석

ID = fopen('result.txt','w'); %result 라는 파일을 생성하고 글을 쓸수 있도록 권한 지정

for n = B
    X=[deg2rad(0.1) deg2rad(0.1)]'; % 초기값 입력
    baselink_angle = deg2rad(n);

    for m = E
        d_d = deg2rad(m);  % 디바이스 각도

        i=0;
        err=max(abs(ff_func(X)));
        tol=1e-4;
        fprintf('i        F_iter\n')

        while err>tol
            i=i+1;
            X1=X - jj_func(X)\ff_func(X);
            X=X1;
            err=max(abs(ff_func(X)));
            fprintf('%d       %f\n',i,err)
        end

        % 최적화를 통해 출력된 각도값
        q_u = X(1);
        q_f = X(2);

        % 접촉부 인장스프링 길이
        l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u);
        l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u);

        % 각도 변수 관계
        h_d = d_d - q_u - q_f;

        %% 각도 및 변형량

        deg_baselink_angle = round(rad2deg(baselink_angle),4);

        deg_q_u = round(rad2deg(q_u),4);
        deg_q_f = round(rad2deg(q_f),4);

        deg_device = round(rad2deg(d_d),4);
        deg_human = round(rad2deg(h_d),4);

        mm_def_l_s_u = round(1000 * (l_s_u - l_s_u_0) , 4);
        mm_def_l_s_f = round(1000 * (l_s_f - l_s_f_0) , 4);

        %% print

        ID = fopen('result.txt','a'); %result라는 텍스트문서에 정보값 저장
        fprintf(ID,'%f %2f %3f %4f %5f %6f %7f\n', deg_baselink_angle, deg_device, deg_human, deg_q_u, deg_q_f, mm_def_l_s_u, mm_def_l_s_f);

        fclose('all');
    end
end