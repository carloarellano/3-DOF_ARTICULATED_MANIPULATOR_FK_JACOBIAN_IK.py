## 3-DOF ARTICULATED MANIPULATOR - Forward Kinematics and Jacobian Matrix
import numpy as np
import math
import PySimpleGUI as sg
import pandas as pd

import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath
from spatialmath import SE3
import matplotlib.pyplot as plt

# GUI code
sg.theme('Green')

#excel code
EXCEL_FILE = '3-DOF ARTICULATED MANIPULATOR.xlsx'
df = pd.read_excel(EXCEL_FILE)

#LAYOUT CODE
main_layout = [
    [sg.Push(), sg.Text('3-DOF ARTICULATED MANIPULATOR CALCULATOR', font=("Century Gothic", 20)), sg.Push()],
    [sg.Push(), sg.Text('Forward Kinematics, Jacobian Matrix and Inverse Kinmatics', font=("Century Gothic", 15)), sg.Push()],
    [sg.Text('Fill out the following fields: (inputs were only an example)', font=("Century Gothic", 10))],
    [sg.Text('      Link Lengths              Joint Variables', font=("Century Gothic", 10)), sg.Push()],
    [sg.Text('a1 ='), sg.InputText('1', key='a1', size=(10,1)), sg.Text('Theta 1 ='), sg.InputText('90',key='T1', size=(10,1)),
        sg.Push(),  sg.Push(), sg.Button('Solve Inverse Kinematics', font=("Century Gothic", 10),
        size=(35,0), button_color=('Black','Orange')), sg.Push(), sg.Push()],


    [sg.Text('a2 ='), sg.InputText('1', key='a2', size=(10,1)), sg.Text('Theta 2 ='), sg.InputText('30',key='T2', size=(10,1)),
        sg.Push()],
    
    
    [sg.Text('a3 ='), sg.InputText('1', key='a3', size=(10,1)), sg.Text('Theta 3 ='), sg.InputText('60' ,key='T3', size=(10,1)),
        sg.Push(),sg.Push(), sg.Button('Jacobian Matrix (J)', font=("Century Gothic", 10), 
        size=(16,0), button_color=('Black','Yellow')),
        sg.Button('Determinant of J', font=("Century Gothic", 10),
        size=(16,0), button_color=('Black','Yellow')),
        sg.Button('Inverse of J', font=("Century Gothic", 10),
        size=(16,0), button_color=('Black','Yellow')),
        sg.Button('Transpose of J', font=("Century Gothic", 10),
        size=(16,0), button_color=('Black','Yellow')), sg.Push()],
    
    [sg.Button('Click this before solving Forward Kinematics', font=("Century Gothic", 10),
        size=(40,0), button_color=('Black','White')),sg.Push()],
    [sg.Button('Solve Forward Kinematics', tooltip = ' Select: Click this before solving Forward Kinematics', font=("Century Gothic", 10)),
     sg.Push(), sg.Push(), sg.Text('3-DOF ARTICULATED MANIPULATOR AND ITS KINEMATIC DIAGRAM', font=("Century Gothic", 10)), sg.Push()],
    
    [sg.Frame('H0_3 Transformation Matrix = ',[[
        sg.Output(size=(60,10))]]),
        sg.Push(), sg.Image('articulatedmanipulator.png'), sg.Push(), sg.Image('3dof.png')],

    [sg.Frame('Position Vector: ',[[
        sg.Text('X ='), sg.InputText(key='X', size=(25,1)),
        sg.Text('Y ='), sg.InputText(key='Y', size=(25,1)),
        sg.Text('Z ='), sg.InputText(key='Z', size=(25,1))]])],

    [sg.Text()],
    [sg.Submit(), sg.Button('Clear Input', font=("Century Gothic", 10)), sg.Exit()]

    
]
#main window code
window = sg.Window('3-DOF ARTICULATED MANIPULATOR - Forward Kinematics', main_layout, resizable=True)
def clear_input():
    for key in values:
        window[key](' ')
    return None

#inverse kinematics window
def inverse_window():
    sg.theme('Green')
    EXCEL_FILE = '3-DOF ARTICULATED MANIPULATOR.xlsx'
    IK_df = pd.read_excel(EXCEL_FILE)

    IK_layout = [
        [sg.Push(), sg.Text('Inverse Kinematics', font=("Century Gothic", 16)), sg.Push()],
        [sg.Text('Fill out the following fields: (inputs were only an example)', font=("Century Gothic", 10))],
        [sg.Text('a1 ='), sg.InputText('1', key='a1', size=(10,1)), sg.Text('mm', font=("Century Gothic", 10)), 
        sg.Push(), sg.Text('X ='), sg.InputText('-3.88e-17', key='X', size=(10,1)), sg.Text('mm', font=("Century Gothic", 10))],
        [sg.Text('a2 ='), sg.InputText('1', key='a2', size=(10,1)), sg.Text('mm', font=("Century Gothic", 10)),
        sg.Push(), sg.Text('Y ='), sg.InputText('0.866', key='Y', size=(10,1)), sg.Text('mm', font=("Century Gothic", 10))],
        [sg.Text('a3 ='), sg.InputText('1', key='a3', size=(10,1)), sg.Text('mm', font=("Century Gothic", 10)),
        sg.Push(), sg.Text('Z ='), sg.InputText('2.5', key='Z', size=(10,1)), sg.Text('mm', font=("Century Gothic", 10))],
#3.88e-17
        [sg.Button('Solve Inverse Kinematics',  font=("Century Gothic", 10))],
        
        [sg.Frame('Joint Variable: ',[[
            sg.Text('Theta 1 ='), sg.InputText(key='Th1', size=(10,1)),
            sg.Text('Theta 2 ='), sg.InputText(key='Th2', size=(10,1)),
            sg.Text('Theta 3 ='), sg.InputText(key='Th3', size=(10,1)),
            ]])],

        [sg.Text()],
        [sg.Submit(), sg.Button('Clear input', font=("Century Gothic", 10)), sg.Exit()]
    ]

    IK_window = sg.Window('3-DOF ARTICULATED MANIPULATOR - Inverse Kinematics', IK_layout, resizable=True)
    def clear_input2():
        for key in values:
            IK_window[key](' ')
        return None
    
    while True:
        event, values = IK_window.read()
        if event == sg.WIN_CLOSED or event == 'Exit':
            break
        elif event =='Clear input':
            clear_input2()

        elif event == 'Solve Inverse Kinematics':
            #link lengths in mm
            a1 = float(values['a1'])
            a2 = float(values['a2'])
            a3 = float(values['a3'])

            #joint variable thetas in degrees
            X = float(values['X'])
            Y = float(values['Y'])
            Z = float(values['Z'])

            try:
                Th1 = np.arctan(Y/X)
            except:
                Th1 = -1 #(NOT A NUMBER)
                sg.popup('WARNING!', font=("Century Gothic", 15))
                sg.popup('The value in Theta 1 causes an error. Please restart the program.', font=("Century Gothic", 10))
                break

            #IK = Arti_Elbow.ikine_min(H0_3)
            #Th1 = |((np.arctan(Y/X))*180.0/np.pi)|
            #r1 = math.sqrt((Y**2)+(X**2))
            #r2 = Z-a1        #3

            #Phi1 = (np.arctan(r2/r1))       #4
            #r3 = math.sqrt(r2**2+r1**2)     #5
            #Phi2 = np.arccos(((a3**2-a2**2-r3**2))/(-2.0*a2*r3))        #6
            #Th2 = ((Phi1+Phi2)*180.0/np.pi)      #7

            #Phi3 = np.arccos(((r3**2-a2**2-a3**2))/(-2.0*a2*a3))
            #Th3 = ((Phi3 - 180)*180.0/np.pi)            #9

            #print("Theta 1 =", np.around(Th1, 3))
            #print("r1 =", np.around(r1, 3))
            #print("r2 =", np.around(r2, 3))
            #print("Phi 1 =", np.around(Phi1, 3))
            #print("r3 =", np.around(r3, 3))
        # print("Phi 2 =", np.around(Phi2, 3))
        #  print("Theta 2 =", np.around(Th2, 3))
        # print("Phi 3 =", np.around(Phi3, 3))
        #  print("Theta 3 =", np.around(Th3, 3))
            ##print('IK = ', np.around(IK,3))


            Arti_Elbow = DHRobot([
                RevoluteDH(a1,0,(90/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
                RevoluteDH(0,a2,0,0,qlim=[(-20/180)*np.pi,(90/180)*np.pi]),
                RevoluteDH(0,a3,0,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
            ], name='Articulated')

            #T= SE3(X, Y, Z) * SE3.OA([0,0,-1],[-1,0,0])

            T51= SE3(X, Y, Z) * SE3.OA([0, -1, 0], [1, 0, 0])
            IKineT51 = Arti_Elbow.ikine_LM(T51)
            
            #IK = Arti_Elbow.ikine_LM(T)
           # sg.popup('IKine = ', IKine)
            #q_pickup = Arti_Elbow.q
            #print(robot.fkine(q_pickup)) 


            #Th1_ = np.arctan(Y/X)
            #r1 = math.sqrt((Y**2)+(X**2))
            #r2 = Z-a1        #3

            #Phi1 = (np.arctan(r2/r1))       #4
            #r3 = math.sqrt((r2**2)+(r1**2))     #5
            #Phi2 = np.arccos((((a3**2)-(a2**2)-(r3**2)))/(-2*(a2*r3)))        #6
            #Th2_ = ((Phi1+Phi2))      #7

            #Phi3 = np.arccos((((r3**2)-(a2**2)-(a3**2)))/(-2*(a2*a3)))
            #Th3_ = ((Phi3 - 180))            #9

            #Th1 = Th1_*180/np.pi
            #Th2 = Th2_*180/np.pi
            #Th3 = Th3_*180/np.pi

            #q_pickup = IK.q
            q_pickupT51_ = IKineT51.q
            #print(q_pickup)

            Th1_ = q_pickupT51_ [0]
            Th1 = Th1_*180/np.pi
            #print (np.around(Th1,3))

            Th2_ = q_pickupT51_ [1]
            Th2 = Th2_*180/np.pi
            #print (np.around(Th2,3))

            Th3_ = q_pickupT51_ [2]
            Th3 = Th3_*180/np.pi
            #print (np.around(Th3,3))
        
            Th1 = IK_window['Th1'].Update(np.around(Th1,3))
            Th2 = IK_window['Th2'].Update(np.around(Th2,3))
            Th3 = IK_window['Th3'].Update(np.around(Th3,3))

            #Th1 = float(input('Th1 = '))
            #Th2 = float(input('Th2 = '))
            #Th3 = float(input('Th3 = '))

            #Th1 = Th1*180/np.pi
            #Th2 = Th2*180/np.pi
            #Th3 = Th3*180/np.pi

            #print('Th1 = ', np.around(Th1,3))
            #print('Th2 = ', np.around(Th2,3))
            #print('Th3 = ', np.around(Th3,3))

        if event == 'Submit':
            IK_df = IK_df.append(values, ignore_index=True)
            IK_df.to_excel(EXCEL_FILE, index=False)
            sg.popup('Data saved!', font=("Century Gothic", 12))

    IK_window.close()

disable_J=window['Jacobian Matrix (J)']
disable_DetJ=window['Determinant of J']
disable_InJ=window['Inverse of J']
disable_TJ=window['Transpose of J']
#disable_IK=window['Inverse Kinematics']

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    if event =='Clear Input':
        clear_input()

    elif event == 'Click this before solving Forward Kinematics':
        disable_J.update(disabled=True)
        disable_DetJ.update(disabled=True)
        disable_InJ.update(disabled=True)
        disable_TJ.update(disabled=True)
        #disable_IK.update(disabled=True)
        
#Forward kinematics code
    elif event == 'Solve Forward Kinematics':
        #link lengths in cm
        a1 = float(values['a1'])
        a2 = float(values['a2'])
        a3 = float(values['a3'])

        #joint variable thetas in degrees
        T1 = float(values['T1'])
        T2 = float(values['T2'])
        T3 = float(values['T3'])

        #joint variable thetas in radians
        T1 = (T1/180.0)*np.pi
        T2 = (T2/180.0)*np.pi
        T3 = (T3/180.0)*np.pi

        #if joint variable are ds, don't need to convert

        ## DH Parameter Table - DHPT (ONLY EDIT for every new manipulator)
        #Rows = no. of HTM
        #Columns + no. of parameters
        #Theta, alpha, r, d

        DHPT = [[(T1),(90.0/180.0)*np.pi,0,(a1)],
                [(T2),(0.0/180.0)*np.pi,(a2),0],
                [(T3),(0.0/180.0)*np.pi,(a3),0]]
        
        # np.trigo function (DHPT[ROW][COLUMN])

        i = 0
        H0_1 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),  np.sin(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0])*np.cos(DHPT[i][1]), -np.cos(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0,0,0,1]]

        i = 1
        H1_2 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),  np.sin(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0])*np.cos(DHPT[i][1]), -np.cos(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0,0,0,1]]

        i = 2
        H2_3 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),  np.sin(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0])*np.cos(DHPT[i][1]), -np.cos(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0,0,0,1]]

        #transformation matrices from base to end effector
       
        #dot product of H0_3 = H0_1*H1_2*H2_3
        H0_2 = np.dot(H0_1, H1_2)
        H0_3 = np.dot(H0_2, H2_3)

        #transformation matrix of the manipulator
        print("H0_3 =")
        print(np.around(np.matrix(H0_3),3))
        print(" ")
        
        #position vector x y z
        X0_3 = H0_3[0,3]
        print("X =", X0_3)
        print("X =", np.around(X0_3,3))
        print(" ")

        Y0_3 = H0_3[1,3]
        print("Y =", Y0_3)
        print("Y =", np.around(Y0_3,3))
        print(" ")

        Z0_3 = H0_3[2,3]
        print("Z =", Z0_3)
        print("Z =", np.around(Z0_3,3))
        print(" ")

        X0_3 = window['X'].Update(X0_3)
        Y0_3 = window['Y'].Update(Y0_3)
        Z0_3 = window['Z'].Update(Z0_3)


        disable_J.update(disabled=False)
        #disable_IK.update(disabled=False)

    elif event == 'Submit':
        df = df.append(values, ignore_index=True)
        df.to_excel(EXCEL_FILE, index=False)
        sg.popup('Data saved!', font=("Century Gothic", 12))
        #clear_input()

    elif event == 'Jacobian Matrix (J)':
        Z_1=[[0],[0],[1]]

        try:
            H0_1=np.matrix(H0_1)
        except:
            H0_1= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 15))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break

        #R 1-3, C1
        #J1=[[1,0,0],[0,1,0],[0,0,1]]  #FOR PRISMATIC TO!
        #J1=np.dot(J1,Z_1)
        #J1=np.matrix(J1)
        
        J1a=[[1,0,0],[0,1,0],[0,0,1]]
        J1a=np.dot(J1a,Z_1)   #r00 (001)

        J1b_1=H0_3[0:3,3:]
        J1b_1=np.matrix(J1b_1)

        J1b_2=[[0],[0],[0]]
        J1b_2=np.matrix(J1b_2)

        J1b= J1b_1 - J1b_2    #d03-d00

        J1=[[(J1a[1,0]*J1b[2,0])-(J1a[2,0]*J1b[1,0])],
            [(J1a[2,0]*J1b[0,0])-(J1a[0,0]*J1b[2,0])],
            [(J1a[0,0]*J1b[1,0])-(J1a[1,0]*J1b[0,0])]]

        #R1-3, C2
        J2a=H0_1[0:3,0:3]
        J2a=np.dot(J2a,Z_1)  #r01 (001)

        J2b_1=H0_3[0:3,3:]
        J2b_1=np.matrix(J2b_1)

        J2b_2=H0_1[0:3,3:]
        J2b_2=np.matrix(J2b_2)

        J2b= J2b_1 - J2b_2     #d03 -d01

        J2=[[(J2a[1,0]*J2b[2,0])-(J2a[2,0]*J2b[1,0])],
            [(J2a[2,0]*J2b[0,0])-(J2a[0,0]*J2b[2,0])],
            [(J2a[0,0]*J2b[1,0])-(J2a[1,0]*J2b[0,0])]]

        #R1-3, C3
        J3a=H0_2[0:3,0:3] 
        J3a=np.dot(J3a,Z_1)    #r02 (001)    r02=r01*r12

        J3b_1=H0_3[0:3,3:]
        J3b_1=np.matrix(J3b_1)

        J3b_2=H0_2[0:3,3:]
        J3b_2=np.matrix(J3b_2)

        J3b= J3b_1 - J3b_2     #d03 - d02

        J3=[[(J3a[1,0]*J3b[2,0])-(J3a[2,0]*J3b[1,0])],
            [(J3a[2,0]*J3b[0,0])-(J3a[0,0]*J3b[2,0])],
            [(J3a[0,0]*J3b[1,0])-(J3a[1,0]*J3b[0,0])]]

        #J4=[[0],[0],[0]]   #FOR PRISMATIC TO!
        #J4=np.matrix(J4)

        J4=[[1,0,0],[0,1,0],[0,0,1]]
        J4=np.dot(J4,Z_1)   #r00
        J4=np.matrix(J4)

        J5=H0_1[0:3,0:3]
        J5=np.dot(J5,Z_1)    #r01
        J5=np.matrix(J5)

        J6=H0_2[0:3,0:3]
        J6=np.dot(J6,Z_1)     #r02
        J6=np.matrix(J6)

        JM1=np.concatenate((J1,J2,J3),1)
        JM2=np.concatenate((J4,J5,J6),1)

        J=np.concatenate((JM1,JM2),0)
        sg.popup("Jacobian Matrix = ", np.around(J,3))

        DJ=np.linalg.det(JM1)
        if DJ==0.0 or DJ==-0.0:
            disable_InJ.update(disabled=True)
            sg.popup('WARNING! Jacobian Matrix is Non-Invertible.', font=("Century Gothic", 12))

        elif DJ != 0.0 or DJ!=-0.0:
             disable_InJ.update(disabled=False)

        #IJ=np.linalg.inv(JM1)
        #TJ=np.transpose(JM1)

        #disable_J.update(disabled=True)
        disable_DetJ.update(disabled=False)
        #disable_InJ.update(disabled=False)
        disable_TJ.update(disabled=False)

    elif event == 'Determinant of J':
        #let JM1 become 3x3 pos matrix
        try:
            JM1=np.concatenate((J1,J2,J3),1)
        except:
            JM1= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 12))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break
        DJ=np.linalg.det(JM1)
        sg.popup('Determinant of J = ', np.around(DJ,3))

        if DJ==0.0 or DJ==-0.0:
            #disable_J.update(disabled=True)
            disable_InJ.update(disabled=True)
            sg.popup('WARNING! Jacobian Matrix is Non-Invertible.', font=("Century Gothic", 12))

    elif event == 'Inverse of J':
        try:
            JM1=np.concatenate((J1,J2,J3),1)
        except:
            JM1= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 12))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break
        IJ=np.linalg.inv(JM1)
        sg.popup('Inverse of J = ', np.around(IJ,3))

    elif event == 'Transpose of J':
        try:
            J=np.concatenate((JM1,JM2),0)
        except:
            J= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 12))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break1
        TJ=np.transpose(JM1)
        sg.popup('Transpose of J = ', np.around(TJ,3))
  
    elif event == 'Solve Inverse Kinematics':
        inverse_window()

        
window.close()
