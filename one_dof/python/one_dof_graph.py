import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from math import sin, cos, radians, degrees
from math import pi
from matplotlib.path import Path
from matplotlib.spines import Spine
from matplotlib.transforms import Affine2D

# INPUT

device_baselink_angle = 0
device_elbow_angle = 120

def print_to_str(*args, **kwargs):
    import io
    output = io.StringIO()
    print(*args, file=output, **kwargs)
    contents = output.getvalue()
    output.close()
    return contents

def safety(device_baselink_angle=0, device_elbow_angle=120):
    # constant value 73kg, 1741mm, male
    
    L_h_u = 0.2817 # human upper arm length[m]
    L_h_f = 0.2689 # human forearm length[m]
    L_d_u = 0.2 # device upper arm length[m]
    L_d_f = 0.2 # device forarm length[m]

    m_h_u = 1.9783 # human upper arm weight[kg]
    m_h_f = 1.1826 # human forearm weight[kg]
    m_d_u = 2 # device upper arm weight[kg]
    m_d_f = 2 # device forearm weight[kg]

    G_h_u = 0.4228 # human upper arm gravity ratio
    G_h_f = 0.4574 # human forearm gravity ratio
    G_d_u = 0.5 # robot upper arm gravity ratio
    G_d_f = 0.5 # robot forearm gravity ratio

    K_u = 10000 # human upper arm shear stiffness[N/m]
    K_f = 10000 # human forearm shear stiffness[N/m]
    K_t_u = 10000 # human upper arm torsion stiffness[N/m]
    K_t_f = 10000 # human forearm torsion stiffness[N/m]
    q_u_0 = 0 # initial value of human upper arm deg[deg]
    q_f_0 = 0 # initial value of human forearm deg[deg]

    g = 9.81 # gravitational acceleration[m/s^2]


# import data

    file = open("result.txt","r")
    lines = np.loadtxt(file)
    selected_rows = np.where((lines[:, 0] == device_baselink_angle) & (lines[:, 1] == device_elbow_angle))[0][0]

    q_u = radians(lines[selected_rows, 3])
    q_f = radians(lines[selected_rows, 4])

    d_d = radians(device_elbow_angle)

    l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u)
    l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u)

    h_d = d_d - q_u - q_f

    baselink_angle = radians(device_baselink_angle)


# initial value of spring length
        
    l_s_u_0 = L_h_u - L_d_u  # initial value of human upper arm spring[m] 
    l_s_f_0 = L_h_f - L_d_f  # initial value of human forearm srping[m]


## calculate safety socre

    Mu = K_t_u*q_u
    Mf = K_t_f*q_f
    Fu = K_u*(l_s_u - l_s_u_0)
    Ff = K_u*(l_s_f - l_s_f_0)
    AD = degrees(h_d) - degrees(d_d)

    if(abs(Fu)>10):
        SFu = 10**(-10)
    else:
        SFu = (-1/10)*abs(Fu)+1

    if(abs(Mu)>10):
        SMu = 10**(-10)
    else:
        SMu = (-1/10)*abs(Mu)+1

    if(abs(Ff)>10):
        SFf = 10**(-10)
    else:
        SFf = (-1/10)*abs(Ff)+1

    if(abs(Mf)>10):
        SMf = 10**(-10)
    else:
        SMf = (-1/10)*abs(Mf)+1

    if(abs(AD)>5):
        SA = 10**(-10)
    else:
        SA = (-1/5)*abs(AD)+1

    St  = SFu*SMu*SFf*SMf*SA
    if(St<=10**(-10)):
        Stotal = 0
    else:
        Stotal = 0.1*(10 + np.log(St))

    x = (L_d_f*cos(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*cos(baselink_angle))/sin(q_f - d_d + q_u)
    y = (L_d_f*sin(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(baselink_angle))/sin(q_f - d_d + q_u)

    x_d_u = L_d_u * cos(baselink_angle + 0) 
    y_d_u = L_d_u * sin(baselink_angle + 0) 

    x_d_f = L_d_f * cos(baselink_angle + d_d)
    y_d_f = L_d_f * sin(baselink_angle + d_d)

    x_h_f = x + L_h_f * cos(baselink_angle + q_u + h_d)
    y_h_f = y + L_h_f * sin(baselink_angle + q_u + h_d)

    x_h_u = x + L_h_u * cos(baselink_angle + q_u)
    y_h_u = y + L_h_u * sin(baselink_angle + q_u)

    return """<h4>
           [Interaction force] </br>
           -device elbow angle %f </br>
           -human elbow angle %f </br>
           -upperarm(moment) %f </br>
           -upperarm(shear force) %f </br>
           -forearm(moment) %f </br>
           -forearm(shear force) %f </br>
           [Safety score] </br>
           -upperarm(moment) %f </br>
           -upperarm(shear force) %f </br>
           -forearm(moment) %f </br>
           -forearm(shear force) %f </br>
           -angle similarity %f </br>
           -Total %f </br>
            """ % (round(degrees(d_d), 2),
                  round(degrees(h_d), 2),
                  round(Mu, 2),
                  round(Fu, 2),
                  round(Mf, 2),
                  round(Ff, 2),
                  round(SMu, 2),
                  round(SFu, 2),
                  round(SMf, 2),
                  round(SFf, 2),
                  round(SA, 2),
                  round(Stotal, 2)
                  )
    
from io import BytesIO

def safety_fig(device_baselink_angle=0, device_elbow_angle=120):
    # constant value 73kg, 1741mm, male
    
    L_h_u = 0.2817 # human upper arm length[m]
    L_h_f = 0.2689 # human forearm length[m]
    L_d_u = 0.2 # device upper arm length[m]
    L_d_f = 0.2 # device forarm length[m]

    m_h_u = 1.9783 # human upper arm weight[kg]
    m_h_f = 1.1826 # human forearm weight[kg]
    m_d_u = 2 # device upper arm weight[kg]
    m_d_f = 2 # device forearm weight[kg]

    G_h_u = 0.4228 # human upper arm gravity ratio
    G_h_f = 0.4574 # human forearm gravity ratio
    G_d_u = 0.5 # robot upper arm gravity ratio
    G_d_f = 0.5 # robot forearm gravity ratio

    K_u = 10000 # human upper arm shear stiffness[N/m]
    K_f = 10000 # human forearm shear stiffness[N/m]
    K_t_u = 10000 # human upper arm torsion stiffness[N/m]
    K_t_f = 10000 # human forearm torsion stiffness[N/m]
    q_u_0 = 0 # initial value of human upper arm deg[deg]
    q_f_0 = 0 # initial value of human forearm deg[deg]

    g = 9.81 # gravitational acceleration[m/s^2]


# import data

    file = open("result.txt","r")
    lines = np.loadtxt(file)
    selected_rows = np.where((lines[:, 0] == device_baselink_angle) & (lines[:, 1] == device_elbow_angle))[0][0]

    q_u = radians(lines[selected_rows, 3])
    q_f = radians(lines[selected_rows, 4])

    d_d = radians(device_elbow_angle)

    l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u)
    l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u)

    h_d = d_d - q_u - q_f

    baselink_angle = radians(device_baselink_angle)


# initial value of spring length
        
    l_s_u_0 = L_h_u - L_d_u  # initial value of human upper arm spring[m] 
    l_s_f_0 = L_h_f - L_d_f  # initial value of human forearm srping[m]


## calculate safety socre

    Mu = K_t_u*q_u
    Mf = K_t_f*q_f
    Fu = K_u*(l_s_u - l_s_u_0)
    Ff = K_u*(l_s_f - l_s_f_0)
    AD = degrees(h_d) - degrees(d_d)

    if(abs(Fu)>10):
        SFu = 10**(-10)
    else:
        SFu = (-1/10)*abs(Fu)+1

    if(abs(Mu)>10):
        SMu = 10**(-10)
    else:
        SMu = (-1/10)*abs(Mu)+1

    if(abs(Ff)>10):
        SFf = 10**(-10)
    else:
        SFf = (-1/10)*abs(Ff)+1

    if(abs(Mf)>10):
        SMf = 10**(-10)
    else:
        SMf = (-1/10)*abs(Mf)+1

    if(abs(AD)>5):
        SA = 10**(-10)
    else:
        SA = (-1/5)*abs(AD)+1

    St  = SFu*SMu*SFf*SMf*SA
    if(St<=10**(-10)):
        Stotal = 0
    else:
        Stotal = 0.1*(10 + np.log(St))

    x = (L_d_f*cos(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*cos(baselink_angle))/sin(q_f - d_d + q_u)
    y = (L_d_f*sin(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(baselink_angle))/sin(q_f - d_d + q_u)

    x_d_u = L_d_u * cos(baselink_angle + 0) 
    y_d_u = L_d_u * sin(baselink_angle + 0) 

    x_d_f = L_d_f * cos(baselink_angle + d_d)
    y_d_f = L_d_f * sin(baselink_angle + d_d)

    x_h_f = x + L_h_f * cos(baselink_angle + q_u + h_d)
    y_h_f = y + L_h_f * sin(baselink_angle + q_u + h_d)

    x_h_u = x + L_h_u * cos(baselink_angle + q_u)
    y_h_u = y + L_h_u * sin(baselink_angle + q_u)

# print
    print('[Interaction force]')
    print('-device elbow angle   : ',str(round(degrees(d_d),4)))
    print('-human elbow angle    : ',str(round(degrees(h_d),4)))
    print('[Interaction force]')
    print('-upperarm(moment)     : ',str(round(Mu,4)))
    print('-upperarm(shear force): ',str(round(Fu,4)))
    print('-forearm(moment)      : ',str(round(Mf,4)))
    print('-forearm(shear force) : ',str(round(Ff,4)))
    print('[Safety score]')
    print('-upperarm(moment)     : ',str(round(SMu,4)))
    print('-upperarm(shear force): ',str(round(SFu,4)))
    print('-forearm(moment)      : ',str(round(SMf,4)))
    print('-forearm(shear force) : ',str(round(SFf,4)))
    print('-angle similarity     : ',str(round(SA,4)))
    print('-Total                : ',str(round(Stotal,4)))


# figure 1 - human posture prediction

    plt.clf()
    plt.title('Posture')
    plt.grid(color='black')
    plt.xlabel('X[m]')
    plt.ylabel('Y[m]')
    plt.xticks(np.arange(-0.4, 0.4, 0.1))
    plt.xlim([-0.4, 0.4])
    plt.yticks(np.arange(-0.1, 0.4, 0.1))
    plt.ylim([-0.1, 0.4])
    plt.plot([0, x_d_f], [0, y_d_f], color='black', linewidth=1)
    plt.plot([0, x_d_u], [0, y_d_u], color='black', linewidth=1)
    plt.plot([x, x_h_f], [y, y_h_f], color='green', linewidth=1)
    plt.plot([x, x_h_u], [y, y_h_u], color='green', linewidth=1)
    plt.plot(x, y, 'g.', markersize=15)
    plt.plot(x_d_f, y_d_f, 'g.', markersize=15)
# plt.text(x_d_f, y_d_f, 'B')
    plt.plot(x_d_u, y_d_u, 'g.', markersize=15)
# plt.text(x_d_u, y_d_u, 'A')
    plt.plot(0, 0, 'k.', markersize=15)

    img = BytesIO()
    plt.savefig(img, format='png', dpi=100)
    img.seek(0)
    return img

def safety_fig2(device_baselink_angle=0, device_elbow_angle=120):
    # constant value 73kg, 1741mm, male
    
    L_h_u = 0.2817 # human upper arm length[m]
    L_h_f = 0.2689 # human forearm length[m]
    L_d_u = 0.2 # device upper arm length[m]
    L_d_f = 0.2 # device forarm length[m]

    m_h_u = 1.9783 # human upper arm weight[kg]
    m_h_f = 1.1826 # human forearm weight[kg]
    m_d_u = 2 # device upper arm weight[kg]
    m_d_f = 2 # device forearm weight[kg]

    G_h_u = 0.4228 # human upper arm gravity ratio
    G_h_f = 0.4574 # human forearm gravity ratio
    G_d_u = 0.5 # robot upper arm gravity ratio
    G_d_f = 0.5 # robot forearm gravity ratio

    K_u = 10000 # human upper arm shear stiffness[N/m]
    K_f = 10000 # human forearm shear stiffness[N/m]
    K_t_u = 10000 # human upper arm torsion stiffness[N/m]
    K_t_f = 10000 # human forearm torsion stiffness[N/m]
    q_u_0 = 0 # initial value of human upper arm deg[deg]
    q_f_0 = 0 # initial value of human forearm deg[deg]

    g = 9.81 # gravitational acceleration[m/s^2]


# import data

    file = open("result.txt","r")
    lines = np.loadtxt(file)
    selected_rows = np.where((lines[:, 0] == device_baselink_angle) & (lines[:, 1] == device_elbow_angle))[0][0]

    q_u = radians(lines[selected_rows, 3])
    q_f = radians(lines[selected_rows, 4])

    d_d = radians(device_elbow_angle)

    l_s_u = (L_h_u*sin(q_f - d_d + q_u) + L_d_f*sin(q_f) + L_d_u*sin(d_d - q_f))/sin(q_f - d_d + q_u)
    l_s_f = (L_h_f*sin(q_f - d_d + q_u) + L_d_u*sin(q_u) + L_d_f*sin(d_d - q_u))/sin(q_f - d_d + q_u)

    h_d = d_d - q_u - q_f

    baselink_angle = radians(device_baselink_angle)


# initial value of spring length
        
    l_s_u_0 = L_h_u - L_d_u  # initial value of human upper arm spring[m] 
    l_s_f_0 = L_h_f - L_d_f  # initial value of human forearm srping[m]


## calculate safety socre

    Mu = K_t_u*q_u
    Mf = K_t_f*q_f
    Fu = K_u*(l_s_u - l_s_u_0)
    Ff = K_u*(l_s_f - l_s_f_0)
    AD = degrees(h_d) - degrees(d_d)

    if(abs(Fu)>10):
        SFu = 10**(-10)
    else:
        SFu = (-1/10)*abs(Fu)+1

    if(abs(Mu)>10):
        SMu = 10**(-10)
    else:
        SMu = (-1/10)*abs(Mu)+1

    if(abs(Ff)>10):
        SFf = 10**(-10)
    else:
        SFf = (-1/10)*abs(Ff)+1

    if(abs(Mf)>10):
        SMf = 10**(-10)
    else:
        SMf = (-1/10)*abs(Mf)+1

    if(abs(AD)>5):
        SA = 10**(-10)
    else:
        SA = (-1/5)*abs(AD)+1

    St  = SFu*SMu*SFf*SMf*SA
    if(St<=10**(-10)):
        Stotal = 0
    else:
        Stotal = 0.1*(10 + np.log(St))

    x = (L_d_f*cos(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*cos(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*cos(baselink_angle))/sin(q_f - d_d + q_u)
    y = (L_d_f*sin(baselink_angle + q_u)*sin(q_f) + L_d_u*sin(d_d - q_f)*sin(baselink_angle + q_u) + L_d_u*sin(q_f - d_d + q_u)*sin(baselink_angle))/sin(q_f - d_d + q_u)

    x_d_u = L_d_u * cos(baselink_angle + 0) 
    y_d_u = L_d_u * sin(baselink_angle + 0) 

    x_d_f = L_d_f * cos(baselink_angle + d_d)
    y_d_f = L_d_f * sin(baselink_angle + d_d)

    x_h_f = x + L_h_f * cos(baselink_angle + q_u + h_d)
    y_h_f = y + L_h_f * sin(baselink_angle + q_u + h_d)

    x_h_u = x + L_h_u * cos(baselink_angle + q_u)
    y_h_u = y + L_h_u * sin(baselink_angle + q_u)

# figure 2 - interaction force

    #plt.clf()
    fig2, ax1 = plt.subplots()
    plt.title('Interaction Force')
    plt.grid(color='black')
    ax1.set_ylabel('moment[Nm]', color = 'red')
    ax1.set_ylim([-10, 10])
    ax2 = ax1.twinx()
    ax2.set_ylabel('force[N]', color = 'blue')
    ax2.set_ylim([-10, 10])
    x1 = np.arange(4)
    X1 = ['Mu', 'Fu', 'Mf', 'Ff']
    Y1 = [Mu, Fu, Mf, Ff]
    C1 = ['red', 'blue', 'red', 'blue']
    plt.bar(x1, Y1, color=C1)
    plt.xticks(x1, X1)

    img2 = BytesIO()
    plt.savefig(img2, format='png', dpi=100)
    img2.seek(0)
    return img2

