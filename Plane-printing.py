#!/usr/bin/python
# -*- coding: utf-8 -*-
#######################################################
import wiringpi as gpio
from wiringpi import GPIO
import math
import serial

##X：移动轴
##Y：旋转轴
##Z：上升轴
##C：补偿轴compensation

##以下数据按照32细分，丝杆导程5mm计算
X_STEPS_PER_MM=1280.0
X_MM_PER_STEP=float(1.0/X_STEPS_PER_MM)

##32个脉冲1.8°，6400脉冲小齿轮一圈，齿轮比15，转盘一圈96000个脉冲
Y_STEPS_PER_Rotation=96000.0
Y_STEPS_PER_ARC=Y_STEPS_PER_Rotation/(2*math.pi)##电机转1弧度需要多少脉冲
Y_ARC_PER_STEP=float(2*math.pi/Y_STEPS_PER_Rotation)

####真实Z_STEPS_PER_MM是6630.4，为了方便计算近似为6400
Z_STEPS_PER_MM=6400

##C补偿数据按照32细分，丝杆导程8mm计算
C_STEPS_PER_CIR=6400.0
C_STEPS_PER_MM=C_STEPS_PER_CIR/8

#our maximum feedrates，默认值，最大进给速度
FAST_XY_FEEDRATE=3000
FAST_Z_FEEDRATE=3000

#Units in curve section
CURVE_SECTION_INCHES=0.019685
CURVE_SECTION_MM=0.5

#Set to one if sensor outputs inverting (ie: 1 means open, 0 means closed)
#RepRap opto endstops are *not* inverting.
SENSORS_INVERTING=0 ##传感器初始值


X_STEP_PIN1=8   ###脉冲引脚
X_DIR_PIN1=9    ###方向引脚
X_ENABLE_PIN1=7

X_STEP_PIN2=0   ###脉冲引脚
X_DIR_PIN2=2    ###方向引脚
X_ENABLE_PIN2=3

Y_STEP_PIN=30
Y_DIR_PIN=21
Y_ENABLE_PIN=22

Z_STEP_PIN=23
Z_DIR_PIN=24
Z_ENABLE_PIN=25

C_DIR_PIN=27
C_ENABLE_PIN=28
C_STEP_PIN=29

feedrate_micros = 200 ###脉冲延迟时间
Line_delay=600
length=80.0

serial_count=0      ###数组
fpx=0
fpy=0
fpz=0


Extrude_old_state=0
Extrude_new_state=0
ser=serial.Serial("/dev/ttyUSB0",9600,timeout=0.5)
ser.open
PRESURE_0_9="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x46\x46\x35\x03\x04"
PRESURE_1="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x30\x46\x34\x03\x04"
PRESURE_1_1="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x31\x46\x33\x03\x04"
PRESURE_1_2="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x32\x46\x32\x03\x04"
PRESURE_1_3="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x33\x46\x31\x03\x04"
PRESURE_1_4="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x34\x46\x30\x03\x04"
PRESURE_1_5="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x35\x45\x46\x03\x04"
PRESURE_1_6="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x36\x45\x45\x03\x04"
PRESURE_1_7="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x37\x45\x44\x03\x04"
PRESURE_1_8="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x38\x45\x43\x03\x04"
PRESURE_1_9="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x31\x39\x45\x42\x03\x04"
PRESURE_2="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x30\x46\x33\x03\x04"
PRESURE_2_1="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x31\x46\x32\x03\x04"
PRESURE_2_2="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x32\x46\x31\x03\x04"
PRESURE_2_3="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x33\x46\x30\x03\x04"
PRESURE_2_4="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x34\x45\x46\x03\x04"
PRESURE_2_5="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x35\x45\x45\x03\x04"
PRESURE_2_6="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x32\x36\x45\x44\x03\x04"
PRESURE_3="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x33\x30\x46\x32\x03\x04"
PRESURE_4="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x34\x30\x46\x31\x03\x04"
PRESURE_5="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x35\x30\x46\x30\x03\x04"
PRESURE_6="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x36\x30\x45\x46\x03\x04"
PRESURE_7="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x37\x30\x45\x45\x03\x04"
PRESURE_8="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x38\x30\x45\x44\x03\x04"
PRESURE_9="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x30\x39\x30\x45\x43\x03\x04"
PRESURE_10="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x30\x30\x46\x34\x03\x04"
PRESURE_11="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x31\x30\x46\x33\x03\x04"
PRESURE_12="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x32\x30\x46\x32\x03\x04"
PRESURE_13="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x33\x30\x46\x31\x03\x04"
PRESURE_14="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x34\x30\x46\x30\x03\x04"
PRESURE_15="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x35\x30\x45\x46\x03\x04"
PRESURE_20="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x30\x30\x46\x33\x03\x04"
PRESURE_21="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x31\x30\x46\x32\x03\x04"
PRESURE_22="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x32\x30\x46\x31\x03\x04"
PRESURE_23="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x33\x30\x46\x30\x03\x04"
PRESURE_24="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x34\x30\x45\x46\x03\x04"
PRESURE_25="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x35\x30\x45\x45\x03\x04"
PRESURE_30="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x33\x30\x30\x46\x32\x03\x04"
PRESURE_35="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x33\x35\x30\x45\x44\x03\x04"
PRESURE_40="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x34\x30\x30\x46\x31\x03\x04"
PRESURE_45="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x34\x35\x30\x45\x43\x03\x04"
PRESURE_50="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x35\x30\x30\x46\x30\x03\x04"
PRESURE_55="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x35\x35\x30\x45\x42\x03\x04"
PRESURE_60="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x36\x30\x30\x45\x46\x03\x04"
PRESURE_70="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x37\x30\x30\x45\x45\x03\x04"
PRESURE_80="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x38\x30\x30\x45\x44\x03\x04"

PRESURE_steady="\x05\x02\x30\x34\x4D\x54\x20\x20\x42\x42\x03\x04"##设置为连续点胶模式
PRESURE_start_end="\x05\x02\x30\x34\x44\x49\x20\x20\x43\x46\x03\x04"##开启和关闭点胶
order_string=ser.write("\x05\x02\x30\x34\x4D\x54\x20\x20\x42\x42\x03\x04")

PRESURE1=PRESURE_35
PRESURE2=PRESURE_40
PRESURE3=PRESURE_50



#---------定义结构体---------#

class directions:
	def __init__(self, xpart0,ypart0,zpart0,cpart0):
		self.x = xpart0
		self.y = ypart0
		self.z = zpart0
		self.c = cpart0

direction = directions(0,0,0,0)

class currentunits:
	def __init__(self, xpart1,ypart1,zpart1):
		self.x = xpart1
		self.y = ypart1
		self.z = zpart1

current_units = currentunits(0.0,0.0,0.0)

class targetunits:
	def __init__(self, xpart2,ypart2,zpart2):
		self.x = xpart2
		self.y = ypart2
		self.z = zpart2

target_units = targetunits(0.0,0.0,0.0)

def  init_steppers():

	#-----------引脚初始化--------#
	gpio.wiringPiSetup()  #初始化

	gpio.pinMode(X_STEP_PIN1,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(X_DIR_PIN1,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(X_ENABLE_PIN1,GPIO.OUTPUT) # 把pin25设置为输出模式
	##gpio.pinMode(X_MIN_PIN,GPIO.INPUT) # 把pin25设置为输入模式
	##gpio.pinMode(X_MAX_PIN,GPIO.INPUT) # 把pin25设置为输入模式

	gpio.pinMode(X_STEP_PIN2,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(X_DIR_PIN2,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(X_ENABLE_PIN2,GPIO.OUTPUT) # 把pin25设置为输出模式

	gpio.pinMode(Y_STEP_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(Y_DIR_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(Y_ENABLE_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	##gpio.pinMode(Y_MIN_PIN,GPIO.INPUT) # 把pin25设置为输入模式
	##gpio.pinMode(Y_MAX_PIN,GPIO.INPUT) # 把pin25设置为输入模式

	gpio.pinMode(Z_STEP_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(Z_DIR_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(Z_ENABLE_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	##gpio.pinMode(Z_MIN_PIN,GPIO.INPUT) # 把pin25设置为输入模式
	##gpio.pinMode(Z_MAX_PIN,GPIO.INPUT) # 把pin25设置为输入模式

	gpio.pinMode(C_STEP_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(C_DIR_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式
	gpio.pinMode(C_ENABLE_PIN,GPIO.OUTPUT) # 把pin25设置为输出模式	

	gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW) #低电平使能
	gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW) #低电平使能
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW)
	gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)

	#order_string=ser.write(PRESURE_steady)
	#order_string=ser.write(PRESURE)

def set_target(x,y,z):
	target_units.x = x
	target_units.y = y
	target_units.z = z

def set_position(x,y,z):
	current_units.x = x
	current_units.y = y
	current_units.z = z

def getcode(key,lines):  #search_string  未经测试函数
	length=len(lines)
	i=lines.find(key)
	i=i+1
	a=''
	while (lines[i]!=' '):
		a=a+lines[i]
		i=i+1
		if i==length:
			number=float(a)
			return number
	number=float(a)
	return number

def calculate_angle(x,y):
	if x>0 and y==0:
		angle=0
	elif x>0 and y>0:
		angle=float(math.atan(abs(y/x)))
	elif x>0 and y<0:
		angle=float((3*math.pi)/2+math.atan(abs(x/y)))
	elif x==0 and y>0:
		angle=float(math.pi/2)
	elif x==0 and y<0:
		angle=float(3*math.pi/2)
	elif x<0 and y>0:
		angle=float(math.pi/2+math.atan(abs(x/y)))
	elif x<0 and y==0:
		angle=float(math.pi)
	elif x<0 and y<0:
		angle=float(math.pi+math.atan(abs(y/x)))
	else:
		angle=0
	return angle

def calculate_radius(x,y):
	r=math.sqrt(x**2+y**2)
	return r

def calculate_direction():##计算出固定方向，垂足在中间情况在dda_move中讨论
	direction.y = int(angle_end >= angle_real)

	if r_vertical>=r_start and r_vertical>=r_end:
		direction.x=int(r_end>=r_start)
	elif r_vertical<=r_start and r_vertical<=r_end:
		direction.x=int(r_end>=r_start)

def do_step(step_pin):######步进电机走一步
	gpio.digitalWrite(step_pin,GPIO.HIGH)
	gpio.delayMicroseconds(5)
	gpio.digitalWrite(step_pin,GPIO.LOW)###发一个脉冲

def dda_move_ME1(r_pass):
	gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)
	global delta_length_old,delta_length_new,r_old,r_now
	p=0
	delta_length_steps=0
	#r_real=float(r_start+r_r*X_MM_PER_STEP)
	r_now=r_pass
	#print 'r_old',r_old,'r_now',r_now
	# print 'delta_length_old',delta_length_old,'delta_length_new',delta_length_new
	# print 'delta_length',delta_length,'delta_length_steps',delta_length_steps
	if abs(r_now-r_old)>0.05:
		#delta_length_new=length-math.sqrt(length**2-r_real**2)
		#delta_length=abs(delta_length_new-delta_length_old)
		#delta_length_steps=delta_length*C_STEPS_PER_MM
		if 0<abs(r_now)<=10:
			delta_length_steps=abs(10*(r_now-r_old))
		elif 10<abs(r_now)<=12:
			delta_length_steps=abs(50*(r_now-r_old))
		elif 12<abs(r_now)<=14:
			delta_length_steps=abs(100*(r_now-r_old))
		elif 14<abs(r_now)<=15:
			delta_length_steps=abs(100*(r_now-r_old))
		elif 15<abs(r_now)<=18:
			delta_length_steps=abs(100*(r_now-r_old))
		# elif 15<r_now<22:
		#  	delta_length_steps=abs((abs(r_now)*16+1.0*r_now**2)-(abs(r_old)*16+1.0*r_old**2))
		# elif 22<abs(r_now)<30:
		# 	delta_length_steps=abs((abs(r_now)*16+0.6*r_now**2)-(abs(r_old)*16+0.6*r_old**2))
		direction.c=int(abs(r_now)>=abs(r_old))
		gpio.digitalWrite(C_DIR_PIN,direction.c)
		gpio.delayMicroseconds(10)
		while True:
			do_step(C_STEP_PIN)
			gpio.delayMicroseconds(5)
			p=p+1
			if p>=delta_length_steps:
				# delta_length_old=delta_length_new
				#print '######################r_now*************************',r_now,r_old
				delta_length_steps=0
				r_old=r_now
				break

def dda_move_ME2(r_pass):
	gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)
	global delta_length_old,delta_length_new,r_old,r_now
	p=0
	delta_length_steps=0
	#r_real=float(r_start+r_r*X_MM_PER_STEP)
	r_now=r_pass
	#print 'r_old',r_old,'r_now',r_now
	# print 'delta_length_old',delta_length_old,'delta_length_new',delta_length_new
	# print 'delta_length',delta_length,'delta_length_steps',delta_length_steps
	if abs(r_now-r_old)>0.05:
		#delta_length_new=length-math.sqrt(length**2-r_real**2)
		#delta_length=abs(delta_length_new-delta_length_old)
		#delta_length_steps=delta_length*C_STEPS_PER_MM
		if 18>r_now>=14:
			direction.c=0
			delta_length_steps=abs(10*(r_now-r_old))
		elif 14>r_now>=12:
			direction.c=0
			delta_length_steps=abs(50*(r_now-r_old))
		elif 12>r_now>=10:
			direction.c=0
			delta_length_steps=abs(100*(r_now-r_old))
		elif 10>r_now>=0:
			direction.c=0
			delta_length_steps=abs(100*(r_now-r_old))
		elif 0>r_now>=-10:
			direction.c=1
			delta_length_steps=abs(10*(r_now-r_old))
		elif -10>r_now>=-12:
			direction.c=1
			delta_length_steps=abs(50*(r_now-r_old))
		elif -12>r_now>=-14:
			direction.c=1
			delta_length_steps=abs(100*(r_now-r_old))
		elif -14>r_now>=-18:
			direction.c=1
			delta_length_steps=abs(100*(r_now-r_old))

		# elif 15<r_now<22:
		#  	delta_length_steps=abs((abs(r_now)*16+1.0*r_now**2)-(abs(r_old)*16+1.0*r_old**2))
		# elif 22<abs(r_now)<30:
		# 	delta_length_steps=abs((abs(r_now)*16+0.6*r_now**2)-(abs(r_old)*16+0.6*r_old**2))
		gpio.digitalWrite(C_DIR_PIN,direction.c)
		gpio.delayMicroseconds(10)
		while True:
			do_step(C_STEP_PIN)
			gpio.delayMicroseconds(5)
			p=p+1
			if p>=delta_length_steps:
				# delta_length_old=delta_length_new
				#print '######################r_now*************************',r_now,r_old
				delta_length_steps=0
				r_old=r_now
				break
def dda_move_ME3(r_pass):
	gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)
	global delta_length_old,delta_length_new,r_old,r_now
	p=0
	delta_length_steps=0
	#r_real=float(r_start+r_r*X_MM_PER_STEP)
	r_now=r_pass
	#print 'r_old',r_old,'r_now',r_now
	# print 'delta_length_old',delta_length_old,'delta_length_new',delta_length_new
	# print 'delta_length',delta_length,'delta_length_steps',delta_length_steps
	if abs(r_now-r_old)>0.05:
		#delta_length_new=length-math.sqrt(length**2-r_real**2)
		#delta_length=abs(delta_length_new-delta_length_old)
		#delta_length_steps=delta_length*C_STEPS_PER_MM
		if 0<abs(r_now)<=10:
			delta_length_steps=abs(10*(r_now-r_old))
		elif 10<abs(r_now)<=12:
			delta_length_steps=abs(50*(r_now-r_old))
		elif 12<abs(r_now)<=14:
			delta_length_steps=abs(90*(r_now-r_old))
		elif 14<abs(r_now)<=15:
			delta_length_steps=abs(90*(r_now-r_old))

		# elif 15<r_now<22:
		#  	delta_length_steps=abs((abs(r_now)*16+1.0*r_now**2)-(abs(r_old)*16+1.0*r_old**2))
		# elif 22<abs(r_now)<30:
		# 	delta_length_steps=abs((abs(r_now)*16+0.6*r_now**2)-(abs(r_old)*16+0.6*r_old**2))
		direction.c=int(abs(r_now)>=abs(r_old))
		gpio.digitalWrite(C_DIR_PIN,direction.c)
		gpio.delayMicroseconds(10)
		while True:
			do_step(C_STEP_PIN)
			gpio.delayMicroseconds(5)
			p=p+1
			if p>=delta_length_steps:
				# delta_length_old=delta_length_new
				#print '######################r_now*************************',r_now,r_old
				delta_length_steps=0
				r_old=r_now
				break

def move_arc(micro_delay):
	gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW)
	gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW)
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW)
	gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)

	# ##当有一个点在（0，0）时，C为真
	# a=(current_units.x or current_units.y)
	# b=(fpx or fpy)
	# c=not(a and b)
	# print 'c',c

	global angle_end,angle_vertical,P_current,r_start

	angle_delta_steps=long((abs(P_target-P_current)*math.pi/180)*Y_STEPS_PER_ARC)
	#print 'angle_delta_steps',angle_delta_steps

	angle_real=angle_start

	# if abs(angle_end-angle_start)>math.pi:
	# 	if angle_vertical>math.pi:
	# 		angle_vertical=angle_vertical-math.pi
	# 	if angle_start>angle_end:
	# 		angle_real=angle_start-2*math.pi
	# 	else:
	# 		angle_end=angle_end-2*math.pi

	r_r=0
	angle_angle=0
	P_current=0
	F_error=0##判断条件
	F_error_flag=0##判断条件

	x_real=current_units.x
	y_real=current_units.y
	r_real=r_start

	direction.y=int(P_current>=P_target)
	gpio.digitalWrite(Y_DIR_PIN,direction.y)##实际小于终点角度，LOW表示逆时针

	#print 'x_real',x_real,'y_real',y_real,'r_real',r_real
	#fw=open('ZC.txt','w')

	while True:
		do_step(Y_STEP_PIN)
		angle_angle=angle_angle+1
		gpio.delayMicroseconds(micro_delay)
		angle_real=float(angle_start+angle_angle*(2.0/96000.0*math.pi))
		x_real=r_real*math.cos(angle_real)
	 	y_real=r_real*math.sin(angle_real)
		F_error_flag=(x_real-i)**2+(y_real-j)**2-R**2
		if F_error_flag>=0:
			angle_real=float(angle_start+2.0*angle_angle/96000.0*math.pi)
			gpio.digitalWrite(X_DIR_PIN1,GPIO.LOW)##HIGH朝有隔板的电机，r增大的方向
			gpio.delayMicroseconds(40)
			while True:
				do_step(X_STEP_PIN1)
				#print '###########################################'
				r_r=r_r-1
				r_real=float(r_start+r_r*X_MM_PER_STEP)
	 			x_real=r_real*math.cos(angle_real)
	 			y_real=r_real*math.sin(angle_real)
	 			#print 'r_real',r_real,'r_start',r_start,'x_real',x_real,'y_real',y_real,'angle_real',angle_real,'angle_angle',angle_angle
	 			# fw.write(str(x_real)+","+str(y_real)+"\n")
				F_error=(x_real-i)**2+(y_real-j)**2-R**2
				gpio.delayMicroseconds(40)
				#gpio.delayMicroseconds(micro_delay)
				dda_move_ME(r_real)
				if F_error<0:
					break
		else:
			angle_real=float(angle_start+2.0*angle_angle/96000.0*math.pi)
			gpio.digitalWrite(X_DIR_PIN1,GPIO.HIGH)
			gpio.delayMicroseconds(40)
			while True:
				do_step(X_STEP_PIN1)
				#print '*******************************************'
				r_r=r_r+1
				r_real=float(r_start+r_r*X_MM_PER_STEP)
	 			x_real=r_real*math.cos(angle_real)
	 			y_real=r_real*math.sin(angle_real)
	 			#print 'r_real',r_real,'x_real',x_real,'y_real',y_real,'angle_real',angle_real,'angle_angle',angle_angle
				F_error=(x_real-i)**2+(y_real-j)**2-R**2
				gpio.delayMicroseconds(40)
				#gpio.delayMicroseconds(micro_delay)
				dda_move_ME(r_real)
				if F_error>0:
					break
		if angle_angle>angle_delta_steps or abs(r_real)>30:
			r_start=r_real
			angle_delta_steps=0
			current_units.x=r_end*round(math.cos(angle_real),15)
			current_units.y=r_end*round(math.sin(angle_real),15)
			#print 'current_units.x',current_units.x,'current_units.y',current_units.y
			print 'r_real',r_real,'current_units.x',current_units.x,'current_units.y',current_units.y
			break
	# gpio.digitalWrite(X_DIR_PIN1,GPIO.LOW)
	# gpio.digitalWrite(Y_DIR_PIN,direction.y)
	# gpio.delayMicroseconds(5)
	# while True:
	# 	do_step(Y_STEP_PIN)
	# 	angle_angle=angle_angle+1
	# 	angle_real=float(angle_real+angle_angle*Y_ARC_PER_STEP)
	# 	gpio.delayMicroseconds(micro_delay*2)

	# 	x_real=r_real*math.cos(angle_real)
	# 	y_real=r_real*math.sin(angle_real)
	# 	F_error_flag=(x_real-i)**2+(y_real-j)**2-R**2

	# 	if F_error_flag>=0:
	# 		print '*****************************************************'
	# 		gpio.digitalWrite(X_STEP_PIN1,GPIO.LOW)##HIGH朝有隔板的电机，r增大的方向
	# 		gpio.delayMicroseconds(5)
	# 		while True:
	# 			do_step(X_STEP_PIN1)
	# 			gpio.delayMicroseconds(micro_delay)
	# 			r_r=r_r-1
	# 			r_real=r_start+r_r*X_MM_PER_STEP
	# 			x_real=r_real*math.cos(angle_real)
	# 			y_real=r_real*math.sin(angle_real)
	# 			F_error=(x_real-i)**2+(y_real-j)**2-R**2
	# 			if F_error<=0:
	# 				break
	# 	elif F_error_flag<0:
	# 		print '######################################################'
	# 		gpio.digitalWrite(X_STEP_PIN1,GPIO.HIGH)
	# 		gpio.delayMicroseconds(5)
	# 		while True:
	# 			do_step(X_STEP_PIN1)
	# 			gpio.delayMicroseconds(micro_delay)
	# 			r_r=r_r+1
	# 			r_real=r_start+r_r*X_MM_PER_STEP
	# 			x_real=r_real*math.cos(angle_real)
	# 			y_real=r_real*math.sin(angle_real)
	# 			F_error=(x_real-i)**2+(y_real-j)**2-R**2
	# 			if F_error>=0:
	# 				break
	# 	else:
	# 		break
	# 	if angle_angle>=angle_delta_steps:
	# 		current_units.x=x_real
	# 		current_units.y=y_real
	# 		break


def move_arc_origin(micro_delay):
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
	direction.y=int(P_current>=P_target)
	gpio.digitalWrite(Y_DIR_PIN,direction.y)

	global delta_length_old,delta_length_new,r_old,r_now,r_r,r_start

	r_r=0
	angle_angle=0
	angle_delta_steps=long((abs(P_target-P_current)*math.pi/180)*Y_STEPS_PER_ARC)
	##记录结束时坐标位置，给下一次插补用
	current_units.x=r_end*round(math.cos((P_target+P_current)*math.pi/180),15)
	current_units.y=r_end*round(math.sin((P_target+P_current)*math.pi/180),15)


	if 	abs(r_start)!=abs(r_end):
		print 'EEEEEEEEEEEE'
		r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
		direction.x=int(r_end>r_start)
		k=gpio.digitalRead(X_STEP_PIN1)
		gpio.digitalWrite(X_DIR_PIN1,direction.x)
		gpio.delayMicroseconds(50)
		while True:
			do_step(X_STEP_PIN1)
			if r_end>=r_start:
				r_r=r_r+1
			elif r_end<r_start:
				r_r=r_r-1
			gpio.delayMicroseconds(2*micro_delay)
			r_real=float(r_start+r_r*X_MM_PER_STEP)
			dda_move_ME1(r_real)
			if abs(r_r)>=r_delta_steps:
				r_start=r_end
				break

	while True:
		do_step(Y_STEP_PIN)
		gpio.delayMicroseconds(2*micro_delay)
		angle_angle=angle_angle+1
		if angle_angle>=angle_delta_steps:
			r_start=r_end
			break

def move_line_origin(micro_delay):
	gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW)
	gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW)
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW)

	global angle_end,angle_vertical,r_r,r_end,r_start,angle_start
	print 'CCCCCCCCCCCCCCC'

	r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
	angle_delta_steps=long((abs(angle_end-angle_start))*Y_STEPS_PER_ARC)

	r_r=0##t用来做累计
	angle_angle=0
	angle_delta=abs(round(angle_end-angle_end,5))
	if (angle_start==angle_end):
		direction.x=int(r_end>=r_start)
		gpio.digitalWrite(X_DIR_PIN1,direction.x)
		while True:
			do_step(X_STEP_PIN1)
			if r_end>=r_start:
				r_r=r_r+1
			elif r_end<r_start:
				r_r=r_r-1
			gpio.delayMicroseconds(micro_delay)
			r_real=float(r_start+r_r*X_MM_PER_STEP)
			dda_move_ME(r_real)
			if abs(r_r)>=r_delta_steps:
				r_start=r_end
				current_units.x=r_end*round(math.cos(angle_end),15)
				current_units.y=r_end*round(math.sin(angle_end),15)
				break

	else:
		if (fpx==0):
			direction.y=int(angle_end<=angle_start)
			gpio.digitalWrite(Y_DIR_PIN,direction.y)
			direction.x=int(r_end>=r_start)
			gpio.digitalWrite(X_DIR_PIN1,direction.x)
			while True:
				do_step(X_STEP_PIN1)
				if r_end>=r_start:
					r_r=r_r+1
				elif r_end<r_start:
					r_r=r_r-1
				gpio.delayMicroseconds(micro_delay)
				r_real=float(r_start+r_r*X_MM_PER_STEP)
				dda_move_ME(r_real)
				if abs(r_r)>=r_delta_steps:
					r_start=r_end
					current_units.x=r_end*round(math.cos(angle_end),15)
					current_units.y=r_end*round(math.sin(angle_end),15)
					break
			# while True:
			# 	do_step(Y_STEP_PIN)
			# 	gpio.delayMicroseconds(2*micro_delay)
			# 	angle_angle=angle_angle+1
			# 	if angle_angle>=angle_delta_steps:
			# 		angle_start=float(angle_start+angle_angle*Y_ARC_PER_STEP)
			# 		current_units.x=r_end*round(math.cos(angle_end),15)
			# 		current_units.y=r_end*round(math.sin(angle_end),15)
			# 		break


		elif (current_units.x==0):
			direction.y=int(angle_end<=angle_start)
			gpio.digitalWrite(Y_DIR_PIN,direction.y)
			direction.x=int(r_end>=r_start)
			gpio.digitalWrite(X_DIR_PIN1,direction.x)
			while True:
				do_step(Y_STEP_PIN)
				gpio.delayMicroseconds(2*micro_delay)
				angle_angle=angle_angle+1
				if angle_angle>=angle_delta_steps:
					angle_start=float(angle_start+angle_angle*Y_ARC_PER_STEP)
					break
			while True:
				do_step(X_STEP_PIN1)
				if r_end>=r_start:
					r_r=r_r+1
				elif r_end<r_start:
					r_r=r_r-1
				gpio.delayMicroseconds(micro_delay)
				r_real=float(r_start+r_r*X_MM_PER_STEP)
				dda_move_ME(r_real)
				if abs(r_r)>=r_delta_steps:
					r_start=r_end
					current_units.x=r_end*round(math.cos(angle_end),15)
					current_units.y=r_end*round(math.sin(angle_end),15)
					break


def dda_move(micro_delay,step_direction):###micro_delay脉冲之间的间隔时间
	gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW)
	gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW)
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW)

	##当有一个点在（0，0）时，C为真
	# a=(current_units.x or current_units.y)
	# b=(fpx or fpy)
	# c=not(a and b)
	# d=(a and b)

	global angle_end,angle_vertical,r_r,r_end,r_start,angle_start
	print 'DDDDDDDD'
	r_real=r_start
	angle_real=angle_start
	#print 'r_start',r_start,'r_end',r_end,'angle_start',angle_start,'angle_end',angle_end
	r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
	angle_delta_steps=long((abs(angle_end-angle_start))*Y_STEPS_PER_ARC)
	#print 'r_delta_steps',r_delta_steps,'angle_delta_steps',angle_delta_steps

	# if abs(angle_end-angle_start)>math.pi:
	# 	if angle_vertical>math.pi:
	# 		angle_vertical=angle_vertical-math.pi
	# 	if angle_start>angle_end:
	# 		angle_real=angle_start-2*math.pi
	# 	else:
	# 		angle_end=angle_end-2*math.pi


	r_r=0##t用来做累计
	angle_angle=0

	F_error=0##判断条件

########插补，angle_end是最终目标，angle_real是实际值，angle_vertical是垂直坐标值
########插补，r_end是目标值，r_real是实际值，r_vertical是垂直坐标值
	if (angle_real==angle_end):
		direction.x=int(r_end>=r_start)
		gpio.digitalWrite(X_DIR_PIN1,direction.x)

		while True:
			do_step(X_STEP_PIN1)
			if r_end>=r_start:
				r_r=r_r+1
			elif r_end<r_start:
				r_r=r_r-1
			gpio.delayMicroseconds(micro_delay)
			r_real=float(r_start+r_r*X_MM_PER_STEP)
			dda_move_ME(r_real)
			if abs(r_r)>=r_delta_steps:
				current_units.x=r_end*round(math.cos(angle_real),15)
				current_units.y=r_end*round(math.sin(angle_real),15)
				#print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
				break

	###C真已经被定义为过原点move_origin_line
	# elif bool(c):#####################33
	# 	if (fpx==0 and fpy==0):
	# 		print '############################################'
	# 		direction.x=int(r_end>=r_start)
	# 		gpio.digitalWrite(X_DIR_PIN1,direction.x)

	# 		while True:
	# 			do_step(X_STEP_PIN1)
	# 			if r_end>=r_start:
	# 				r_r=r_r+1
	# 			elif r_end<r_start:
	# 				r_r=r_r-1
	# 			gpio.delayMicroseconds(2*micro_delay)
	# 			r_real=float(r_start+r_r*X_MM_PER_STEP)
	# 			dda_move_ME(r_real)
	# 			if abs(r_r)>=r_delta_steps:
	# 				angle_start=angle_real
	# 				r_start=math.sqrt(current_units.y**2+current_units.x**2)
	# 				print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
	# 				break

	# 	elif (current_units.x==0 and current_units.y==0):
	# 		print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$'
	# 		if angle_real!=angle_end:
	# 			direction.y=int(angle_end<=angle_real)
	# 			gpio.digitalWrite(Y_DIR_PIN,direction.y)	
	# 			while True:
	# 				do_step(Y_STEP_PIN)
	# 				gpio.delayMicroseconds(micro_delay)
	# 				gpio.delayMicroseconds(micro_delay)
	# 				angle_angle=angle_angle+1
	# 				if angle_angle>=angle_delta_steps:
	# 					angle_start=float(angle_start+angle_angle*Y_ARC_PER_STEP)
	# 					break

	# 		direction.x=int(r_end>=r_start)
	# 		gpio.digitalWrite(X_DIR_PIN1,direction.x)

	# 		while True:
	# 			do_step(X_STEP_PIN1)
	# 			if r_end>=r_start:
	# 				r_r=r_r+1
	# 			elif r_end<r_start:
	# 				r_r=r_r-1
	# 			gpio.delayMicroseconds(micro_delay)
	# 			gpio.delayMicroseconds(micro_delay)
	# 			if abs(r_r)>=r_delta_steps:
	# 				r_start=float(r_start+r_r*X_MM_PER_STEP)
	# 				print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
	# 				break

	# 		current_units.x=r_start*round(math.cos(angle_real),15)
	# 		current_units.y=r_start*round(math.sin(angle_real),15)


	elif (angle_real<angle_end):
		gpio.digitalWrite(Y_DIR_PIN,GPIO.LOW)##实际小于终点角度，LOW表示逆时针
		while True:
			#print 'YYYYYYYYYYYYY'
			do_step(Y_STEP_PIN)
			gpio.delayMicroseconds(micro_delay)
			angle_angle=angle_angle+1
			angle_real=float(angle_start+2.0*angle_angle*math.pi/Y_STEPS_PER_Rotation)
			# with open('data1.txt','a') as d:
			# 	d.write(str(angle_real)+'\n')
			# with open('data2.txt','a') as e:
			# 	e.write(str(r_real)+'\n')
			F_error_flag=r_real*math.cos(angle_vertical-angle_real)-r_vertical
			# print 'r_vertical',r_vertical,'r_real',r_real,'angle_real',angle_real,'angle_vertical',angle_vertical
			# print 'math.cos(angle_vertical-angle_real)',math.cos(angle_vertical-angle_real),'F_error_flag',F_error_flag
			if F_error_flag>0:
				gpio.digitalWrite(X_DIR_PIN1,step_direction)##LOW是朝无隔板的电机，r减小的方向
				gpio.delayMicroseconds(10)
				while True:
					do_step(X_STEP_PIN1)
					r_r=r_r-1
					r_real=float(r_start+r_r*X_MM_PER_STEP)
					F_error=abs(r_real*math.cos(angle_vertical-angle_real))-r_vertical
					gpio.delayMicroseconds(micro_delay)
					dda_move_ME(r_real)
					if F_error<=0:
						break
			else:
				gpio.digitalWrite(X_DIR_PIN1,not step_direction)
				gpio.delayMicroseconds(10)
				while True:
					do_step(X_STEP_PIN1)
					r_r=r_r+1
					r_real=float(r_start+r_r*X_MM_PER_STEP)
					F_error=abs(r_real*math.cos(angle_vertical-angle_real))-r_vertical
					gpio.delayMicroseconds(micro_delay)
					dda_move_ME(r_real)
					if F_error>=0:
						break
			# e.close()
			# d.close()

			# r_real=float(r_start+r_r*X_MM_PER_STEP)
			# dda_move_ME(r_real)

			if abs(angle_angle)>angle_delta_steps:
				angle_angle=0
				r_start=r_end
				angle_start=angle_end
				current_units.x=round(r_end*round(math.cos(angle_real),15),2)
				current_units.y=round(r_end*round(math.sin(angle_real),15),2)
				print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
				break
			

	elif (angle_real>angle_end):
		gpio.digitalWrite(Y_DIR_PIN,GPIO.HIGH)###实际大于终点，HIGH是顺时针
		while True:
			do_step(Y_STEP_PIN)
			gpio.delayMicroseconds(micro_delay)
			angle_angle=angle_angle-1##angle_angle用来做角度脉冲累计
			angle_real=float(angle_start+2.0*angle_angle*math.pi/Y_STEPS_PER_Rotation)##angle_real代表此刻实际的角度，用弧度表示
			F_error_flag=r_real*math.cos(angle_vertical-angle_real)-r_vertical##F_error_flag是判断标准
			#print 'r_vertical',r_vertical,'r_real',r_real,'angle_real',angle_real,'angle_vertical',angle_vertical
			if F_error_flag>0:
				gpio.digitalWrite(X_DIR_PIN1,step_direction)##HIGH朝有隔板的电机，r增大的方向
				gpio.delayMicroseconds(10)
				while True:
					do_step(X_STEP_PIN1)
					r_r=r_r-1
					r_real=float(r_start+r_r*X_MM_PER_STEP)
					F_error=abs(r_real*math.cos(angle_vertical-angle_real))-r_vertical
					gpio.delayMicroseconds(micro_delay)
					dda_move_ME(r_real)
					if F_error<0:
						break
			elif F_error_flag<=0:
				gpio.digitalWrite(X_DIR_PIN1,not step_direction)
				gpio.delayMicroseconds(10)
				while True:
					do_step(X_STEP_PIN1)
					r_r=r_r+1
					r_real=float(r_start+r_r*X_MM_PER_STEP)
					F_error=abs(r_real*math.cos(angle_vertical-angle_real))-r_vertical
					gpio.delayMicroseconds(micro_delay)
					dda_move_ME(r_real)
					if F_error>=0:
						break
			if abs(angle_angle)>angle_delta_steps:
				angle_angle=0
				r_start=r_end
				angle_start=angle_end
				current_units.x=round(r_end*round(math.cos(angle_real),15),2)
				current_units.y=round(r_end*round(math.sin(angle_real),15),2)
				print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
				break
	else:
		print'位置不改变'



############主函数###################################################
f=open('gcode.txt','r')
ser=serial.Serial("/dev/ttyUSB0",9600,timeout=0.5)
ser.open
init_steppers()
order_string=ser.write(PRESURE1)

delta_length_old=100.0
r_old=0.0
r_start=0.0
r_real=0.0
angle_start=0.0
angle_end=0.0
P_current=0


while True:
	line=f.readline()  #读取一行数据

	if line=='':
		print '打印完成'
		gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW) #pin25输出为高电平
		gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW)
		gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
		gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW)
		gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)
		break

	if line.find('E')!=-1:
		extrude_number=getcode('E',line)
		if extrude_number==1:
			order_string=ser.write(PRESURE1)
		if extrude_number==2:
			order_string=ser.write(PRESURE2)
			#gpio.delay(1000)
		if extrude_number==3:
			order_string=ser.write(PRESURE3)
			#gpio.delay(1000)
		Extrude_new_state=1
	else:
		Extrude_new_state=0

	if Extrude_new_state!=Extrude_old_state:
		order_string=ser.write(PRESURE_start_end)##挤出头工作
		if extrude_number==1:
			gpio.delay(50)
			order_string=ser.write(PRESURE1)
		elif extrude_number==2:
			gpio.delay(50)
			order_string=ser.write(PRESURE2)
		else:
			gpio.delay(50)
		Extrude_old_state=Extrude_new_state

	if line.find('X')!=-1:		##找到X则执行
		fpx = getcode('X',line) ##取出目标坐标值
		#print 'fpx=',fpx
	else:						##没找到则执行
		fpx = current_units.x

	if line.find('Y')!=-1:
		fpy = getcode('Y',line)
		#print 'fpy=',fpy
	else:
		fpy = current_units.y

	if line.find('Z')!=-1:
		fpz = getcode('Z',line)
	else:
		fpz = current_units.z

	if line.find('C')!=-1:####L表示只动X轴，写法L10，即走10mm
		c_end = getcode('C',line)
		c_c=0
		print 'ccccc'
		##记录结束时坐标位置，给下一次插补用
		c_delta_steps=long((abs(c_end))*C_STEPS_PER_MM)
		direction.x=int(c_end>0)
		print 'direction.x',direction.x
		gpio.digitalWrite(C_DIR_PIN,direction.x)
		gpio.delayMicroseconds(5)
		while True:
			do_step(C_STEP_PIN)
			c_c=c_c+1
			gpio.delayMicroseconds(100)#feedrate_micros#
			if c_c>=abs(c_delta_steps):
				break

	if line.find('i')!=-1:##圆心位置
		i = getcode('i',line)
	
	if line.find('j')!=-1:
		j = getcode('j',line)

	if line.find('F')!=-1:
		feedrate_micros = int(getcode('F',line))

	if line.find('R')!=-1:#半径大小
		R = getcode('R',line)

	if line.find('L')!=-1:####L表示只动X轴，写法L10，即走10mm
		L = getcode('L',line)
		r_r=0
		r_end=L
		##记录结束时坐标位置，给下一次插补用
		if 	r_start!=r_end:
			r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
			direction.x=int(r_end>r_start)
			k=gpio.digitalRead(X_STEP_PIN1)
			gpio.digitalWrite(X_DIR_PIN1,direction.x)
			gpio.delayMicroseconds(5)
			while True:
				do_step(X_STEP_PIN1)
				if r_end>=r_start:
					r_r=r_r+1
				elif r_end<r_start:
					r_r=r_r-1
				gpio.delayMicroseconds(100)#feedrate_micros#
				r_real=float(r_start+r_r*X_MM_PER_STEP)
				dda_move_ME1(r_real)
				if abs(r_r)>=r_delta_steps:
					r_start=r_end
					break
	if line.find('M')!=-1:####L表示只动X轴，写法L10，即走10mm
		M = getcode('M',line)
		r_r=0
		r_end=M
		##记录结束时坐标位置，给下一次插补用
		if 	r_start!=r_end:
			r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
			direction.x=int(r_end>r_start)
			k=gpio.digitalRead(X_STEP_PIN1)
			gpio.digitalWrite(X_DIR_PIN1,direction.x)
			gpio.delayMicroseconds(5)
			while True:
				do_step(X_STEP_PIN1)
				if r_end>=r_start:
					r_r=r_r+1
				elif r_end<r_start:
					r_r=r_r-1
				gpio.delayMicroseconds(100)#feedrate_micros#
				r_real=float(r_start+r_r*X_MM_PER_STEP)
				dda_move_ME2(r_real)
				if abs(r_r)>=r_delta_steps:
					r_start=r_end
					break
	if line.find('N')!=-1:####L表示只动X轴，写法L10，即走10mm
		N = getcode('N',line)
		print 'CCCCCCCCCCCC'
		r_r=0
		r_end=N
		##记录结束时坐标位置，给下一次插补用
		if 	r_start!=r_end:
			r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
			direction.x=int(r_end>r_start)
			k=gpio.digitalRead(X_STEP_PIN1)
			gpio.digitalWrite(X_DIR_PIN1,direction.x)
			gpio.delayMicroseconds(5)
			while True:
				do_step(X_STEP_PIN1)
				if r_end>=r_start:
					r_r=r_r+1
				elif r_end<r_start:
					r_r=r_r-1
				gpio.delayMicroseconds(100)#feedrate_micros#
				r_real=float(r_start+r_r*X_MM_PER_STEP)
				dda_move_ME3(r_real)
				if abs(r_r)>=r_delta_steps:
					r_start=r_end
					break
	# if line.find('M')!=-1:#####仅用来走细胞支架，弓字形
	# 	M = getcode('M',line)
	# 	r_r=0
	# 	r_end=M
	# 	##记录结束时坐标位置，给下一次插补用
	# 	if 	r_start!=r_end:
	# 		print 'EEEEEEEEEEEE'
	# 		r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
	# 		direction.x=int(r_end>r_start)
	# 		k=gpio.digitalRead(X_STEP_PIN1)
	# 		gpio.digitalWrite(X_DIR_PIN1,direction.x)
	# 		gpio.delayMicroseconds(5)
	# 		while True:
	# 			do_step(X_STEP_PIN1)
	# 			if r_end>=r_start:
	# 				r_r=r_r+1
	# 			elif r_end<r_start:
	# 				r_r=r_r-1
	# 			gpio.delayMicroseconds(feedrate_micros)
	# 			r_real=float(r_start+r_r*X_MM_PER_STEP)
	# 			dda_move_ME(r_real)
	# 			if abs(r_r)>=r_delta_steps:
	# 				r_start=r_end
	# 				current_units.x=r_end
	# 				print 'current_units.x',current_units.x
	# 				current_units.y=0
	# 				break

	# if line.find('N')!=-1:#####仅用来走细胞支架，弓字形
	# 	N = getcode('N',line)
	# 	r_r=0
	# 	r_end=N
	# 	##记录结束时坐标位置，给下一次插补用
	# 	if 	r_start!=r_end:
	# 		print 'EEEEEEEEEEEE'
	# 		r_delta_steps=long((abs(r_end-r_start))*X_STEPS_PER_MM)
	# 		direction.x=int(r_end>r_start)
	# 		k=gpio.digitalRead(X_STEP_PIN1)
	# 		gpio.digitalWrite(X_DIR_PIN1,not direction.x)
	# 		gpio.delayMicroseconds(5)
	# 		while True:
	# 			do_step(X_STEP_PIN1)
	# 			if r_end>=r_start:
	# 				r_r=r_r+1
	# 			elif r_end<r_start:
	# 				r_r=r_r-1
	# 			gpio.delayMicroseconds(feedrate_micros)
	# 			r_real=float(r_start+r_r*X_MM_PER_STEP)
	# 			dda_move_ME(r_real)
	# 			if abs(r_r)>=r_delta_steps:
	# 				r_start=r_end
	# 				current_units.x=r_end
	# 				print 'current_units.x',current_units.x
	# 				current_units.y=0
	# 				break
		

	if line.find('P')!=-1:
		P_target = getcode('P',line)##P是增量角度

	if line.find('S')!=-1:
		pause = long(getcode('S',line))
		gpio.delay(pause)

	if line.find('G')==0:##是否是判断?????不知道
		code=getcode('G',line)

		#if code==90:
		#	if line.find('X'):
		#		fpx = getcode('X',line)
		#	else:
		#		fpx = current_unit.x
		#	if line.find('Y'):
		#		fpy = getcode('Y',line)
		#	else:
		#		fpy = current_unit.y
		#	if line.find('Z'):
		#		fpz = getcode('Z',line)
		#	else:
		#		fpz = current_unit.z
		#elif code==91:
		#	fpx = getcode('X',line) + current_units.x
		#	fpy = getcode('Y',line) + current_units.y
		#	fpz = getcode('Z',line) + current_units.z
		
		#print 'start_direction.x=',direction.x,'start_direction.y=',direction.y

		if (code==1 or code==0 or code==6 or code==7 or code==8):   ##how fast do we move?
			print 'move'
			set_target(fpx,fpy,fpz)
			# print 'current_units.x=',current_units.x,'target_units.x=',target_units.x
			# print 'current_units.y=',current_units.y,'target_units.y=',target_units.y
			# print 'target_direction.x=',direction.x,'targrt_direction.y=',direction.y

			##与极点到直线垂直距离决定r方向，计算垂直点坐标x_vertical，y_vertical

			#print 'current_units.x',current_units.x,'current_units.y',current_units.y
			current_units.x=round(current_units.x,1)
			current_units.y=round(current_units.y,1)
			#print 'current_units.x',current_units.x,'current_units.y',current_units.y
			#print 'fpx',fpx,'fpy',fpy
			x_s=current_units.x
			y_s=current_units.y
			x_e=fpx
			y_e=fpy


			if current_units.y!=fpy or current_units.x!=fpx:
				x_vertical=float((y_e-y_s)*(x_s*y_e-x_e*y_s)/((y_e-y_s)**2+(x_e-x_s)**2))
				y_vertical=float((x_s-x_e)*(x_s*y_e-x_e*y_s)/((y_e-y_s)**2+(x_e-x_s)**2))
				#print 'x_vertical=',x_vertical,'y_vertical',y_vertical

				##计算角度
				if (current_units.x!=0 and current_units.y!=0):
					angle_start=calculate_angle(current_units.x,current_units.y)
				elif (current_units.x>0 and current_units.y==0):
					angle_start=0
				elif (current_units.x<0 and current_units.y==0):
					angle_start=math.pi
				elif (current_units.x==0 and current_units.y>0):
					angle_start=math.pi/2
				elif (current_units.x==0 and current_units.y<0):
					angle_start=3*math.pi/2
				elif(current_units.x==0 and current_units.y==0):
					angle_start=0
	 			if (fpx!=0 and fpy!=0):
	 				angle_end=calculate_angle(fpx,fpy)
				elif (fpx>0 and fpy==0):
					angle_end=0
				elif (fpx<0 and fpy==0):
					angle_end=math.pi
				elif (fpx==0 and fpy>0):
					angle_end=math.pi/2
				elif (fpx==0 and fpy<0):
					angle_end=3*math.pi/2
				elif(fpx==0 and fpy==0):
					angle_end=0
	 			angle_vertical=calculate_angle(x_vertical,y_vertical)
	 			#print 'angle_end',angle_end,'angle_start',angle_start,'angle_vertical',angle_vertical
	 
				##计算半径
				r_vertical=math.sqrt(x_vertical**2+y_vertical**2)
				r_start=calculate_radius(current_units.x,current_units.y)
				r_end=calculate_radius(target_units.x,target_units.y)


				if (current_units.x==0 and current_units.y==0) or (fpx==0 and fpy==0):
					move_line_origin(feedrate_micros)
				else:
					if (code==1):
						#print 'current_units.x',current_units.x,'current_units.y',current_units.y,'r_start',r_start,'angle_start',angle_start
						print 'angle_vertical',angle_vertical,'x_vertical',x_vertical,'y_vertical',y_vertical
						dda_move(feedrate_micros,0)##0代表低电平，方向朝无隔板的电机
					elif(code==6):
						dda_move(feedrate_micros,1)##1代表高电平，方向朝有隔板的电机
					elif(code==0 or code==7):######code==0表示转动方向变换
						angle_vertical=angle_vertical-2*math.pi
						if (angle_start!=0):
							angle_start=angle_start-2*math.pi
						if (angle_end!=0):
							angle_end=angle_end-2*math.pi
						if (code==0):
							dda_move(feedrate_micros,0)
						elif(code==7):
							dda_move(feedrate_micros,1)
						angle_start=angle_start+2*math.pi
					elif(code==8):###x轴角度设置为360度
						angle_end=2.0*math.pi
						angle_vertical==7.0*math.pi/4.0
						dda_move(feedrate_micros,0)
						angle_start=0
						
				current_units.x=fpx
				current_units.y=fpy
				r_start=r_end

			elif (current_units.z!=fpz):
				print '*******************'
				direction.z=int(current_units.z>=fpz)
				direction.c=int(current_units.z<=fpz)
				gpio.digitalWrite(Z_DIR_PIN,direction.z) # 把pin25设置为输出模式dir
				gpio.digitalWrite(C_DIR_PIN,direction.c)
				#Z_delta_steps=long((abs(fpz-current_units.z))*Z_STEPS_PER_MM)
				C_delta_steps=long((abs(fpz-current_units.z))*C_STEPS_PER_MM)
				z_z=0
				c_c=0
				while True:
					gpio.digitalWrite(C_STEP_PIN,GPIO.HIGH)
					gpio.delayMicroseconds(200)
					gpio.digitalWrite(C_STEP_PIN,GPIO.LOW)
					gpio.delayMicroseconds(200)
					c_c=c_c+1
					while True:
						gpio.digitalWrite(Z_STEP_PIN,GPIO.HIGH)
						gpio.delayMicroseconds(40)
						gpio.digitalWrite(Z_STEP_PIN,GPIO.LOW)
						gpio.delayMicroseconds(10)
						z_z=z_z+1
						if z_z==8:
							current_units.z=fpz
							z_z=0
							break
					if c_c>C_delta_steps:
						c_c=0
						break
				
				# print 'current_units.x',current_units.x

		elif code==3:
			if (i==0 and j==0):
				print 'move_arc_origin'
				r_end=R
				move_arc_origin(80)
			else:
				print 'move_arc'
				r_start=calculate_radius(current_units.x,current_units.y)
				print 'r_start',r_start
				angle_start=calculate_angle(current_units.x,current_units.y)
				move_arc(feedrate_micros)
		
		######Fermat's spiral
		elif code==4:
			print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
			gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)	
			gpio.digitalWrite(Y_DIR_PIN,GPIO.HIGH)##实际小于终点角度，HIGH表示顺时针
			gpio.delayMicroseconds(20)
			r_start=20.053####当角度为2Pi时的半径
			angle_start=2*math.pi
			r_real=r_start
			angle_angle=0
			r_r=0
			feedrate_micros=200
			while True:
				do_step(Y_STEP_PIN)
				gpio.delayMicroseconds(feedrate_micros)
				angle_angle=angle_angle-1
				angle_real=float(angle_start+angle_angle*Y_ARC_PER_STEP)
				#####  r=8*β^(1/2)
				if angle_real<=math.pi*3/2:
					feedrate_micros=120
				if angle_real<=math.pi:
					feedrate_micros=80
				if angle_real<=0:
					r_start=0
					angle_start=0
					current_units.x=0
					current_units.y=0
					break
				F_error_flag=r_real-8*angle_real**(0.5)
				# print 'r_vertical',r_vertical,'r_real',r_real,'angle_real',angle_real,'angle_vertical',angle_vertical
				# print 'math.cos(angle_vertical-angle_real)',math.cos(angle_vertical-angle_real),'F_error_flag',F_error_flag
				if F_error_flag>=0:
					gpio.digitalWrite(X_DIR_PIN1,GPIO.LOW)##HIGH朝有隔板的电机，r增大的方向
					gpio.delayMicroseconds(50)
					while True:
						do_step(X_STEP_PIN1)
						r_r=r_r-1
						r_real=float(r_start+r_r*X_MM_PER_STEP)
						F_error=r_real-8*angle_real**(0.5)
						gpio.delayMicroseconds(feedrate_micros)
						dda_move_ME(r_real)
						if F_error<=0:
							break
				else:
					gpio.digitalWrite(X_DIR_PIN1,GPIO.HIGH)
					gpio.delayMicroseconds(50)
					while True:
						do_step(X_STEP_PIN1)
						r_r=r_r+1
						r_real=float(r_start+r_r*X_MM_PER_STEP)
						F_error=r_real-8*angle_real**(0.5)
						gpio.delayMicroseconds(feedrate_micros)
						dda_move_ME(r_real)
						if F_error>=0:
							break

				if abs(angle_angle)<=0:
					r_start=0
					angle_start=0
					current_units.x=0
					current_units.y=0
					print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
					break
	
		elif code==5:
			gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
			gpio.digitalWrite(Y_DIR_PIN,GPIO.LOW)##实际小于终点角度，LOW表示逆时针
			gpio.delayMicroseconds(20)
			r_start=0
			r_real=0
			angle_start=0
			angle_angle=0
			r_r=0
			feedrate_micros=60
			while True:
				do_step(Y_STEP_PIN)
				gpio.delayMicroseconds(feedrate_micros)
				angle_angle=angle_angle+1
				angle_real=float(angle_start+angle_angle*Y_ARC_PER_STEP)
				#####  r=8*β^(1/2)
				F_error_flag=r_real-8*angle_real**(0.5)
				# print 'r_vertical',r_vertical,'r_real',r_real,'angle_real',angle_real,'angle_vertical',angle_vertical
				# print 'math.cos(angle_vertical-angle_real)',math.cos(angle_vertical-angle_real),'F_error_flag',F_error_flag
				if F_error_flag>=0:
					gpio.digitalWrite(X_DIR_PIN1,GPIO.HIGH)##HIGH朝有隔板的电机，r增大的方向，但是这里R增大的方向是没有隔板的电机
					gpio.delayMicroseconds(50)
					while True:
						do_step(X_STEP_PIN1)
						r_r=r_r-1
						r_real=float(r_start+r_r*X_MM_PER_STEP)
						F_error=r_real-8*angle_real**(0.5)
						gpio.delayMicroseconds(feedrate_micros)
						dda_move_ME(r_real)
						if F_error<=0:
							break
				else:
					gpio.digitalWrite(X_DIR_PIN1,GPIO.LOW)
					gpio.delayMicroseconds(50)
					while True:
						do_step(X_STEP_PIN1)
						r_r=r_r+1
						r_real=float(r_start+r_r*X_MM_PER_STEP)
						F_error=r_real-8*angle_real**(0.5)
						gpio.delayMicroseconds(feedrate_micros)
						dda_move_ME(r_real)
						if F_error>=0:
							break

				if abs(angle_angle)>=96000:
					feedrate_micros=90
				if abs(angle_angle)>=144000:#######转一圈96000个脉冲
					r_start=r_real
					angle_start=0
					current_units.x=r_real
					current_units.y=0
					print 'r_end',r_end,'current_units.x',current_units.x,'current_units.y',current_units.y
					break






		##暂停移动
		# elif code==04:
		# 	delay(int(getcode('P',line)))

		elif code==11:
			gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW)
			gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW)
			gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW) 
			gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW) 
			gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW) 
			print '结束'

		###Inches for Units设置当前距离单位为英寸(G20)

		##归零
		elif code==28:
			#set_target(0.0, 0.0, 0.0)
			#dda_move(getMaxSpeed())
			return_home()
			print('G28')

		elif code==90:##设置坐标系
			current_units.x=fpx
			current_units.y=fpy
			r_start=calculate_radius(fpx,fpy)
			angle_start=calculate_angle(fpx,fpy)
			# current_units = currentunits(0.0,0.0,0.0)
			# current_steps = currentsteps(0.0,0.0,0.0)
			print 'G90',current_units.x,current_units.y


        ##设置3D打印机内存中XYZE的位置值，不移动对应的步进电机
		elif code==92:
			set_position(0.0, 0.0, 0.0)


		##都没有，则执行某个响应，等待完善
		##else：
init_steppers()


				







