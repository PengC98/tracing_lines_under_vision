# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math
import roslib;
import rospy
from geometry_msgs.msg import Twist
from time import sleep
import PID
import time
lower_black=np.array([0,0,0])
upper_black=np.array([180,255,46])
cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_EXPOSURE,-10.0)
kernel=np.ones((4,4),np.uint8)
linear_high_speed=1.5
linear_low_speed=0.5
P=0.2
I=0
D=0

def detect_black(roi):
    x=0
    y=0
    w=0
    h=0
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,lower_black,upper_black)
    thresh=cv2.GaussianBlur(mask,(3,3),0)
    #thresh=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    _,img=cv2.threshold(thresh,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    img,contours,hierarchy=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if contours is None:
        pass
    else:
        for i in contours:
            x,y,w,h=cv2.boundingRect(i)
            cv2.rectangle(roi,(x,y),(x+w,y+h),(0,255,),2)
    cv2.imshow("mask",mask)
    return x+w/2,y+h/2

def detect_line(roi):
    angle=0
    im_gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    #im_gray=cv2.medianBlur(im_gray,3)
    im_gray=cv2.GaussianBlur(im_gray,(3,3),0)
    #im_gray=cv2.bilateralFilter(im_gray,6,200,20)
    _,thresh=cv2.threshold(im_gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    thresh=cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel)#cv2.erode(thresh,kernel,iterations=1)
    edges = cv2.Canny(thresh, 40, 100, apertureSize = 3)
    lines=cv2.HoughLines(edges,1.1,math.pi/180,12)
    if lines is None:
        angle=360
        pass
    else:
        line1=lines[:,0,:]
        temp=0
        i=0
        for rho,theta in line1[:]:
            t=theta
            if theta>1.57:
                t=t-math.pi
            temp+=t
            i+=1
            a=np.cos(theta)
            b=np.sin(theta)
            x0=a*rho
            y0=b*rho
            x1=int(x0+1000*(-b))
            x2=int(x0-1000*(-b))
            y1=int(y0+1000*(a))
            y2=int(y0-1000*(a))
            #print x1,y1,x2,y2
            cv2.line(roi,(x1,y1),(x2,y2),(0,0,255),1)
        temp=temp/i
        angle=(temp/math.pi)*180
    return angle

def tracing_the_line(frame):
    detect=frame[190:200,80:240]
    detect1=frame[180:190,80:240]
    detect2=frame[150:160,40:280]
    detect3=frame[140:150,40:280]
    detect4=frame[100:120,80:240]
    detect5=frame[65:80,80:240]
    x1,y1=detect_black(detect)
    x2,y2=detect_black(detect1)
    x3,y3=detect_black(detect2)
    x4,y4=detect_black(detect3)
    angle2=detect_line(detect5)
    if x2==x1:
        angle=90
    else :
        angle=180*math.atan((y2+10-y1)/(x2-x1))/math.pi
    if angle>0:
        angle=90-angle
    if angle<0:
        angle=-(angle+90)
    if x3==x4:
        angle1=90
    else :
        angle1=180*math.atan((y3+10-y4)/(x4-x3))/math.pi
    if angle1>0:
        angle1=90-angle1
    if angle1<0:
        angle1=-(angle1+90)
    return angle,angle1,angle2

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('robot')
    angle=0
    angle1=0
    angle2=0
    pid=PID.PID(P,I,D)
    pid.SetPoint=0.0
    pid.setSampleTime(0.01)
    feedback=0
    while True:
        ret,frame=cap.read()
        angle,angle1,angle2=tracing_the_line(frame)
        control_angle=0.9*angle+0.1*angle1
        pid.update(control_angle)
        output=pid.output
        
        if angle2!=360:
            if angle2<89:
                angle2=angle2
            if angle2>90:
                angle2=angle2-180
        

        angle_last=angle
        angle=0
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = output
        pub.publish(twist)
        cv2.imshow("f",frame)
        #cv2.imshow("img",img)
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
