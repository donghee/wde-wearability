import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians, degrees

def ff_func(X):
    global a_u, b_u, a_f, b_f, m_d_u, m_d_f, q_u_0, q_f_0, K_u, K_f, K_u_nor, K_f_nor, L_h_u, L_h_f, m_h_u, m_h_f, g
    global L_d_u, L_d_f, K_t_u, K_t_f, l_s_u_0, l_s_f_0, d_d, a, G_h_u, G_h_f, G_d_u, G_d_f

    q_u = X[0]
    q_f = X[1]

    # matlab
    #f1 = (conj(K_t_u)*(2*conj(q_u) - 2*conj(q_u_0)))/2 - conj(K_f)*((cos(conj(q_u))*conj(L_d_u) + cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_f) - conj(L_d_f)*cos(conj(d_d) - conj(q_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u)) - (cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(q_u))*conj(L_d_u) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_f) + conj(L_d_f)*sin(conj(d_d) - conj(q_u))))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2)*(conj(l_s_f_0) - (sin(conj(q_u))*conj(L_d_u) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_f) + conj(L_d_f)*sin(conj(d_d) - conj(q_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u))) + conj(K_u)*((cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(q_f))*conj(L_d_f) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_u) + conj(L_d_u)*sin(conj(d_d) - conj(q_f))))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2 - (cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_u))/sin(conj(q_f) - conj(d_d) + conj(q_u)))*(conj(l_s_u_0) - (sin(conj(q_f))*conj(L_d_f) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_u) + conj(L_d_u)*sin(conj(d_d) - conj(q_f)))/sin(conj(q_f) - conj(d_d) + conj(q_u))) + conj(g)*conj(m_h_f)*((cos(conj(a) + conj(q_u))*conj(L_d_u)*sin(conj(d_d) - conj(q_f)) + sin(conj(q_f))*cos(conj(a) + conj(q_u))*conj(L_d_f) + sin(conj(a))*cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u))/sin(conj(q_f) - conj(d_d) + conj(q_u)) - (cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(a) + conj(q_u))*conj(L_d_u)*sin(conj(d_d) - conj(q_f)) + sin(conj(q_f))*sin(conj(a) + conj(q_u))*conj(L_d_f) + sin(conj(a))*sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2) + conj(g)*conj(m_h_u)*((cos(conj(a) + conj(q_u))*conj(L_d_u)*sin(conj(d_d) - conj(q_f)) + sin(conj(q_f))*cos(conj(a) + conj(q_u))*conj(L_d_f) + sin(conj(a))*cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u))/sin(conj(q_f) - conj(d_d) + conj(q_u)) + cos(conj(a) + conj(q_u))*conj(G_h_u)*conj(L_h_u) - (cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(a) + conj(q_u))*conj(L_d_u)*sin(conj(d_d) - conj(q_f)) + sin(conj(q_f))*sin(conj(a) + conj(q_u))*conj(L_d_f) + sin(conj(a))*sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2);
    #f2 = (conj(K_t_f)*(2*conj(q_f) - 2*conj(q_f_0)))/2 - conj(K_u)*((cos(conj(q_f))*conj(L_d_f) + cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_u) - conj(L_d_u)*cos(conj(d_d) - conj(q_f)))/sin(conj(q_f) - conj(d_d) + conj(q_u)) - (cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(q_f))*conj(L_d_f) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_u) + conj(L_d_u)*sin(conj(d_d) - conj(q_f))))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2)*(conj(l_s_u_0) - (sin(conj(q_f))*conj(L_d_f) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_u) + conj(L_d_u)*sin(conj(d_d) - conj(q_f)))/sin(conj(q_f) - conj(d_d) + conj(q_u))) + conj(K_f)*((cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(q_u))*conj(L_d_u) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_f) + conj(L_d_f)*sin(conj(d_d) - conj(q_u))))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2 - (cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_f))/sin(conj(q_f) - conj(d_d) + conj(q_u)))*(conj(l_s_f_0) - (sin(conj(q_u))*conj(L_d_u) + sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_h_f) + conj(L_d_f)*sin(conj(d_d) - conj(q_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u))) + conj(g)*conj(m_h_u)*((cos(conj(q_f))*sin(conj(a) + conj(q_u))*conj(L_d_f) - sin(conj(a) + conj(q_u))*conj(L_d_u)*cos(conj(d_d) - conj(q_f)) + sin(conj(a))*cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u))/sin(conj(q_f) - conj(d_d) + conj(q_u)) - (cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(a) + conj(q_u))*conj(L_d_u)*sin(conj(d_d) - conj(q_f)) + sin(conj(q_f))*sin(conj(a) + conj(q_u))*conj(L_d_f) + sin(conj(a))*sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2) - conj(g)*conj(m_h_f)*(cos(conj(a) + conj(d_d) - conj(q_f))*conj(G_h_f)*conj(L_h_f) - (cos(conj(q_f))*sin(conj(a) + conj(q_u))*conj(L_d_f) - sin(conj(a) + conj(q_u))*conj(L_d_u)*cos(conj(d_d) - conj(q_f)) + sin(conj(a))*cos(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u))/sin(conj(q_f) - conj(d_d) + conj(q_u)) + (cos(conj(q_f) - conj(d_d) + conj(q_u))*(sin(conj(a) + conj(q_u))*conj(L_d_u)*sin(conj(d_d) - conj(q_f)) + sin(conj(q_f))*sin(conj(a) + conj(q_u))*conj(L_d_f) + sin(conj(a))*sin(conj(q_f) - conj(d_d) + conj(q_u))*conj(L_d_u)))/sin(conj(q_f) - conj(d_d) + conj(q_u))^2);

    f1 = (np.conj(K_t_u)*(2*np.conj(q_u) - 2*np.conj(q_u_0)))/2 - np.conj(K_f)*((cos(np.conj(q_u))*np.conj(L_d_u) + cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_f) - np.conj(L_d_f)*cos(np.conj(d_d) - np.conj(q_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)) - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(q_u))*np.conj(L_d_u) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_f) + np.conj(L_d_f)*sin(np.conj(d_d) - np.conj(q_u))))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2)*(np.conj(l_s_f_0) - (sin(np.conj(q_u))*np.conj(L_d_u) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_f) + np.conj(L_d_f)*sin(np.conj(d_d) - np.conj(q_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))) + np.conj(K_u)*((cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(q_f))*np.conj(L_d_f) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_u) + np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f))))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2 - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_u))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)))*(np.conj(l_s_u_0) - (sin(np.conj(q_f))*np.conj(L_d_f) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_u) + np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))) + np.conj(g)*np.conj(m_h_f)*((cos(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(q_f))*cos(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) + sin(np.conj(a))*cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)) - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(q_f))*sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) + sin(np.conj(a))*sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2) + np.conj(g)*np.conj(m_h_u)*((cos(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(q_f))*cos(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) + sin(np.conj(a))*cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)) + cos(np.conj(a) + np.conj(q_u))*np.conj(G_h_u)*np.conj(L_h_u) - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(q_f))*sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) + sin(np.conj(a))*sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2);
    f2 = (np.conj(K_t_f)*(2*np.conj(q_f) - 2*np.conj(q_f_0)))/2 - np.conj(K_u)*((cos(np.conj(q_f))*np.conj(L_d_f) + cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_u) - np.conj(L_d_u)*cos(np.conj(d_d) - np.conj(q_f)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)) - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(q_f))*np.conj(L_d_f) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_u) + np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f))))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2)*(np.conj(l_s_u_0) - (sin(np.conj(q_f))*np.conj(L_d_f) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_u) + np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))) + np.conj(K_f)*((cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(q_u))*np.conj(L_d_u) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_f) + np.conj(L_d_f)*sin(np.conj(d_d) - np.conj(q_u))))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2 - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_f))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)))*(np.conj(l_s_f_0) - (sin(np.conj(q_u))*np.conj(L_d_u) + sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_h_f) + np.conj(L_d_f)*sin(np.conj(d_d) - np.conj(q_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))) + np.conj(g)*np.conj(m_h_u)*((cos(np.conj(q_f))*sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) - sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*cos(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(a))*cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)) - (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(q_f))*sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) + sin(np.conj(a))*sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2) - np.conj(g)*np.conj(m_h_f)*(cos(np.conj(a) + np.conj(d_d) - np.conj(q_f))*np.conj(G_h_f)*np.conj(L_h_f) - (cos(np.conj(q_f))*sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) - sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*cos(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(a))*cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u)) + (cos(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*(sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_u)*sin(np.conj(d_d) - np.conj(q_f)) + sin(np.conj(q_f))*sin(np.conj(a) + np.conj(q_u))*np.conj(L_d_f) + sin(np.conj(a))*sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))*np.conj(L_d_u)))/sin(np.conj(q_f) - np.conj(d_d) + np.conj(q_u))**2);

    F = [f1, f2]
    #print(F)
    return np.array(F)


def jj_func(X):
    global a_u, b_u, a_f, b_f, m_d_u, m_d_f, q_u_0, q_f_0, K_u, K_f, K_u_nor, K_f_nor, L_h_u, L_h_f, m_h_u, m_h_f, g
    global L_d_u, L_d_f, K_t_u, K_t_f, l_s_u_0, l_s_f_0, d_d, a, G_h_u, G_h_f, G_d_u, G_d_f

    q_u=X[0]
    q_f=X[1]

    # matlab
    #j11 = K_t_u + K_f*((L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^2)^2 + K_u*((cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^2 - (L_h_u*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u))^2 + K_u*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u))*(L_h_u - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u) + (2*L_h_u*cos(q_f - d_d + q_u)^2)/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^3) - g*m_h_u*((2*cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 + G_h_u*L_h_u*sin(a + q_u) - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) - g*m_h_f*((2*cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) + K_f*((2*cos(q_f - d_d + q_u)*(L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^3)*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u));
    #j12 = K_u*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u))*(L_h_u - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f)))/sin(q_f - d_d + q_u)^2 + (L_h_u*cos(q_f - d_d + q_u)^2)/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^3) - g*m_h_u*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) - g*m_h_f*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) + K_f*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u))*(L_h_f - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u)))/sin(q_f - d_d + q_u)^2 + (L_h_f*cos(q_f - d_d + q_u)^2)/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^3) - K_u*((L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^2)*((cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^2 - (L_h_u*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u)) - K_f*((L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^2)*((cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^2 - (L_h_f*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u));
    #j21 = K_u*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u))*(L_h_u - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f)))/sin(q_f - d_d + q_u)^2 + (L_h_u*cos(q_f - d_d + q_u)^2)/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^3) - g*m_h_u*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) - g*m_h_f*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) + K_f*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u))*(L_h_f - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u)))/sin(q_f - d_d + q_u)^2 + (L_h_f*cos(q_f - d_d + q_u)^2)/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^3) - K_u*((L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^2)*((cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^2 - (L_h_u*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u)) - K_f*((L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^2)*((cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^2 - (L_h_f*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u));
    #j22 = K_t_f + K_u*((L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^2)^2 + K_f*((cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^2 - (L_h_f*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u))^2 - g*m_h_f*((2*cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 + G_h_f*L_h_f*sin(a + d_d - q_f) - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) + K_f*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u))*(L_h_f - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u) + (2*L_h_f*cos(q_f - d_d + q_u)^2)/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)^3) - g*m_h_u*((2*cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)^3) + K_u*((2*cos(q_f - d_d + q_u)*(L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f)))/sin(q_f - d_d + q_u)^2 - (2*cos(q_f - d_d + q_u)^2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)^3)*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u));

    j11 = K_t_u + K_f*((L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**2)**2 + K_u*((cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**2 - (L_h_u*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u))**2 + K_u*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u))*(L_h_u - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u) + (2*L_h_u*cos(q_f - d_d + q_u)**2)/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**3) - g*m_h_u*((2*cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 + G_h_u*L_h_u*sin(a + q_u) - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) - g*m_h_f*((2*cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) + K_f*((2*cos(q_f - d_d + q_u)*(L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**3)*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u));
    j12 = K_u*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u))*(L_h_u - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f)))/sin(q_f - d_d + q_u)**2 + (L_h_u*cos(q_f - d_d + q_u)**2)/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**3) - g*m_h_u*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) - g*m_h_f*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) + K_f*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u))*(L_h_f - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u)))/sin(q_f - d_d + q_u)**2 + (L_h_f*cos(q_f - d_d + q_u)**2)/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**3) - K_u*((L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**2)*((cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**2 - (L_h_u*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u)) - K_f*((L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**2)*((cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**2 - (L_h_f*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u));
    j21 = K_u*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u))*(L_h_u - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f)))/sin(q_f - d_d + q_u)**2 + (L_h_u*cos(q_f - d_d + q_u)**2)/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**3) - g*m_h_u*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) - g*m_h_f*((L_d_u*cos(a + q_u)*cos(d_d - q_f) - L_d_f*cos(a + q_u)*cos(q_f) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) - (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 + (cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) + K_f*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u))*(L_h_f - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u) + (cos(q_f - d_d + q_u)*(L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u)))/sin(q_f - d_d + q_u)**2 + (L_h_f*cos(q_f - d_d + q_u)**2)/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**3) - K_u*((L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**2)*((cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**2 - (L_h_u*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u)) - K_f*((L_h_f*cos(q_f - d_d + q_u) + L_d_u*cos(q_u) - L_d_f*cos(d_d - q_u))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**2)*((cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**2 - (L_h_f*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u));
    j22 = K_t_f + K_u*((L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f))/sin(q_f - d_d + q_u) - (cos(q_f - d_d + q_u)*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**2)**2 + K_f*((cos(q_f - d_d + q_u)*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**2 - (L_h_f*cos(q_f - d_d + q_u))/sin(q_f - d_d + q_u))**2 - g*m_h_f*((2*cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 + G_h_f*L_h_f*sin(a + d_d - q_f) - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) + K_f*(l_s_f_0 - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u))*(L_h_f - (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u) + (2*L_h_f*cos(q_f - d_d + q_u)**2)/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u)))/sin(q_f - d_d + q_u)**3) - g*m_h_u*((2*cos(q_f - d_d + q_u)*(L_d_f*sin(a + q_u)*cos(q_f) - L_d_u*sin(a + q_u)*cos(d_d - q_f) + L_d_u*cos(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a)))/sin(q_f - d_d + q_u)**3) + K_u*((2*cos(q_f - d_d + q_u)*(L_h_u*cos(q_f - d_d + q_u) + L_d_f*cos(q_f) - L_d_u*cos(d_d - q_f)))/sin(q_f - d_d + q_u)**2 - (2*cos(q_f - d_d + q_u)**2*(L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f)))/sin(q_f - d_d + q_u)**3)*(l_s_u_0 - (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u));

    J = [[j11, j12], [j21, j22]]
    #print(J)
    return np.array(J)

# 디바이스 상수 - 디바이스에서 주어지는 값
a_u = 0.175
b_u = 0.225
a_f = 0.175
b_f = 0.225

#신체상수 - 신체정보에서 주어지는 값
m_d_u = 2
m_d_f = 2

q_u_0 = 0
q_f_0 = 0
K_u = 10000
K_f = 10000
K_t_u = 10000
K_t_f = 10000
L_h_u = 0.2817
L_h_f = 0.2689
m_h_u = 1.9783
m_h_f = 1.1826

# 무게중심 위치 - 인체관련 논문 참고.
G_h_u = 0.4228
G_h_f = 0.4574
G_d_u = 0.5
G_d_f = 0.5

g = 9.81

# 접촉부 중심 위치 - 디바이스 상수를 통해 계산. 결국 상수값.
L_d_u = (a_u + b_u) / 2
L_d_f = (a_f + b_f) / 2

# 스프링길이 초기값 설정(고정부 착용위치)
l_s_u_0 = L_h_u - L_d_u
l_s_f_0 = L_h_f - L_d_f - 0.01

# 베이스링크 각도 INPUT
a = radians(0) # deg 단위로 원하는 값 넣어주세요. ex) 0 45 90 180
#a = radians(45)


# 디바이스 각도 INPUT
d_d = radians(190)
#d_d = radians(135) # deg 단위로 원하는 값 넣어주세요. ex) 0 45 90 180

# Newton Raphson Method
# 적절한 초기값이 대입되도록 반복
i_q_u = 5 # q_u의 초기값
i_q_f = 5 # q_f의 초기값
tol = 1e-4

while True:
    i = 0
    X = np.array([radians(i_q_u), radians(i_q_f)])
    err = max(np.abs(ff_func(X)))
    errors = []

    while True:
        i += 1
        X1 = X - np.linalg.inv(jj_func(X)).dot(ff_func(X))
        #X1 = X - np.linalg.solve(jj_func(X), ff_func(X))
        X = X1
        err = max(np.abs(ff_func(X)))
        print(f'i: {i}, error: {err}')
        errors.append(err)
        if err <= tol:
            break

    if np.all(np.diff(errors) < 0):
        print('Did not diverge')
        break
    else:
        print(f'Diverged in {len(errors)} iterations')
        if i_q_u >= -5:
            i_q_u = i_q_u - 0.1 
        else:
            i_q_f = i_q_f - 0.1
            i_q_u = 5
        print(f'i_q_u: {i_q_u}, i_q_f: {i_q_f}')

# 계산결과
# 최적화를 통해 출력된 각도값
q_u, q_f = X

# 각도 변수 관계: 인체팔꿈치 각도 = 디바이스팔꿈치 각도 - 상완비틀림각도 - 전완비틀림각도
h_d = d_d - q_u - q_f

# 상완,전완 접촉부 인장스프링 길이
l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u)
l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u)

# 인체 팔꿈치 좌표값(x,y)
x = (L_d_f*cos(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*cos(a))/sin(q_f - d_d + q_u)
y = (L_d_f*sin(a + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(a + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(a))/sin(q_f - d_d + q_u)

# 시각화 - 각 좌표점 위치를 통해 시각화
x_d_u = L_d_u * cos(a + 0)
y_d_u = L_d_u * sin(a + 0)
x_d_f = L_d_f * cos(a + d_d)
y_d_f = L_d_f * sin(a + d_d)
x_h_f = x + L_h_f * cos(a + q_u + h_d)
y_h_f = y + L_h_f * sin(a + q_u + h_d)
x_h_u = x + L_h_u * cos(a + q_u)
y_h_u = y + L_h_u * sin(a + q_u)

plt.figure(figsize=(5, 3))
plt.grid(color='black')
plt.xlabel('X[m]')
plt.ylabel('Y[m]')
plt.xticks(np.arange(-0.4, 0.5, 0.1))
plt.xlim([-0.4, 0.4])
plt.yticks(np.arange(-0.1, 0.5, 0.1))
plt.ylim([-0.1, 0.4])
plt.plot([0, x_d_f], [0, y_d_f], color='black', linewidth=1)
plt.plot([0, x_d_u], [0, y_d_u], color='black', linewidth=1)
plt.plot([x, x_h_f], [y, y_h_f], color='green', linewidth=1)
plt.plot([x, x_h_u], [y, y_h_u], color='green', linewidth=1)
plt.plot(x, y, 'g.', markersize=15)
plt.plot(x_d_f, y_d_f, 'g.', markersize=15)
plt.text(x_d_f, y_d_f, 'B')
plt.plot(x_d_u, y_d_u, 'g.', markersize=15)
plt.text(x_d_u, y_d_u, 'A')
plt.plot(0, 0, 'black', markersize=15)
#plt.show()

# 각도 및 변형량 출력
deg_q_u = round(degrees(q_u), 4)
deg_q_f = round(degrees(q_f), 4)
deg_device = round(degrees(d_d), 4)
deg_human = round(degrees(h_d), 4)
mm_def_l_s_u = round(1000 * (l_s_u - l_s_u_0), 4)
mm_def_l_s_f = round(1000 * (l_s_f - l_s_f_0), 4)
print(f'deg_q_u = {deg_q_u}')
print(f'deg_q_f = {deg_q_f}')
print(f'deg_device = {deg_device}')
print(f'deg_human = {deg_human}')
print(f'mm_def_l_s_u = {mm_def_l_s_u}')
print(f'mm_def_l_s_f = {mm_def_l_s_f}')

# 안전성 점수
Mu = K_t_u*q_u
Mf = K_t_f*q_f
Fu = K_u*(l_s_u - l_s_u_0)
Ff = K_u*(l_s_f - l_s_f_0)
A = degrees(h_d) - degrees(d_d)

# 안전성에 대한 정의
SFu = (-1/100)*abs(Fu)+1
SMu = (-1/ 100)*abs(Mu)+1
SFf = (-1/100)*abs(Ff)+1
SMf = (-1/100)*abs(Mf)+1

# 팔꿈치 각도에 따른 안전성 점수
if(degrees(h_d) > 180):
    Sq = 1**(-10)
else:
    Sq = (-1/4)*abs(A)+1

# 안전성 종합점수
St  = SFu*SMu*SFf*SMf*Sq
Stotal  = 0.1*(10 + np.log(St))

safety_Fu = round(SFu,4)
safety_Mu = round(SMu,4)
safety_Ff = round(SFf,4)
safety_Mf = round(SMf,4)
safety_Total = round(Stotal,4)

print(f'safety_Fu = {safety_Fu}')
print(f'safety_Mu = {safety_Mu}')
print(f'safety_Ff = {safety_Ff}')
print(f'safety_Mf = {safety_Mf}')
print(f'safety_Total = {safety_Total}')

plt.show()
