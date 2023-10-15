clc
clear 
close all

%% 각도변수 

% q_w = 허리 접촉부 비틀림 각도(디바이스 허리부와 인체 허리 사이 각도)
% q_t = 종아리 접촉부 비틀림 각도(디바이스 종아리부과 인체 종아리 사이 각도)

syms q_w q_t

%% 전역

global L_hw L_ht L_hs L_rw L_rt L_rs L_rtw L_rts
global l_w l_t l_s q_s q_rh q_rk q_hh q_hk
global m_hw m_ht m_hs m_rw m_rt m_rs
global G_hw G_ht G_hs G_rw G_rt G_rs
global K_w K_t K_s K_tw K_tt K_ts g
global l_w_0 l_t_0 l_s_0 

%% min각도 max각도 INPUT

interval = 1; % 해석 간격(deg 단위)

minimum_hip_angle = 30;
maximum_hip_angle = 330;

minimum_knee_angle = 30;
maximum_knee_angle = 180; % 0,180,360은 singularity 자세

H = [minimum_hip_angle:interval:maximum_hip_angle];
NH = find(H == 180);
H(:,NH) = [];

K = [minimum_knee_angle:interval:maximum_knee_angle];
NK = find(K == 180);
K(:,NK) = [];

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

%% 스프링길이 초기값 설정(고정부 착용위치)

% 길게 착용했다면 -
% 짧게 착용했다면 +
    
l_w_0 = L_hw - L_rw; % 인체 허리 길이방향 스프링 초기값(m) 
l_t_0 = L_rtw; % 인체 허벅지 길이방향 스프링 초기값(m)
l_s_0 = L_hs - L_rs; % 인체 정강이 길이방향 스프링 초기값(m)

%% Newton Raphson Method

ID = fopen('result.txt','w');

for n = H
    X=[deg2rad(1) deg2rad(1)]';
    
    for m = K
    
    q_hh = deg2rad(n);  % 엉덩이 관절 각도
    q_hk = deg2rad(m); % 무릎 관절 각도
    
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
q_w = X(1);
q_t = X(2);

% 접촉부 인장스프링 길이
q_s = -asin((L_rts*sin(q_hk + q_t) - L_ht*sin(q_hk) + (sin(q_hk)*(L_rtw*sin(q_hh + q_t) + L_rw*sin(q_w)))/sin(q_hh))/L_rs);
l_t = (L_rtw*sin(q_hh + q_t) + L_rw*sin(q_w))/sin(q_hh);
l_w = -(L_rw*sin(q_hh + q_w) - L_hw*sin(q_hh) + L_rtw*sin(q_t))/sin(q_hh);
l_s = -(L_rs*sin(q_hk + q_s) - L_hs*sin(q_hk) + L_rts*sin(q_t))/sin(q_hk);

% 각도 변수 관계
q_rh = q_s + q_t + q_hh;
q_rk = q_w + q_t + q_hk;

%% 각도 및 변형량

deg_q_hh = round(rad2deg(q_hh),4);
deg_q_rh = round(rad2deg(q_rh),4);
deg_q_hk = round(rad2deg(q_hk),4);
deg_q_rk = round(rad2deg(q_rk),4);

deg_q_w = round(rad2deg(q_w),4);
deg_q_t = round(rad2deg(q_t),4);
deg_q_s = round(rad2deg(q_s),4);

mm_def_l_w = round(1000 * (l_w - l_w_0) , 4);
mm_def_l_t = round(1000 * (l_t - l_t_0) , 4);
mm_def_l_s = round(1000 * (l_s - l_s_0) , 4);

%% print

ID = fopen('result.txt','a'); %result라는 텍스트문서에 정보값 저장
fprintf(ID,'%f %2f %3f %4f %5f %6f %7f %8f %9f %10f\n', deg_q_hh, deg_q_rh, deg_q_hk, deg_q_rk, deg_q_w, deg_q_t, deg_q_s, mm_def_l_w, mm_def_l_t, mm_def_l_s);

fclose('all');

    end
end