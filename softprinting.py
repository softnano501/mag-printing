#!/usr/bin/python
# -*- coding: utf-8 -*-

import wiringpi as gpio
import datetime
from wiringpi import GPIO
import math
import serial



X_STEPS_PER_INCH=4800####根据指令修改，每INCH需要的step数，每mm需要的step数
X_STEPS_PER_MM=160
X_MOTOR_STEPS=200#####步进电机每转步数，无需用到(固件)

Y_STEPS_PER_INCH=4800
Y_STEPS_PER_MM=200
Y_MOTOR_STEPS=200

Z_STEPS_PER_INCH=4800
Z_STEPS_PER_MM=100
Z_MOTOR_STEPS=200

border_x=180
border_y=180

#our maximum feedrates，默认值，最大进给速度
FAST_XY_FEEDRATE=600
FAST_Z_FEEDRATE=1200

#Units in curve section
CURVE_SECTION_INCHES=0.019685
CURVE_SECTION_MM=0.5

#Set to one if sensor outputs inverting (ie: 1 means open, 0 means closed)
#RepRap opto endstops are *not* inverting.
SENSORS_INVERTING=0 ##传感器初始值


X_STEP_PIN=8
X_DIR_PIN=9
X_ENABLE_PIN=7
X_MIN_PIN=30

Y_STEP_PIN=0
Y_DIR_PIN=2
Y_ENABLE_PIN=3
Y_MIN_PIN=22

Z_STEP_PIN=1
Z_DIR_PIN=4
Z_ENABLE_PIN=5
Z_MIN_PIN=21

Extrude_0=26
Extrude_1=31

x_units = float(X_STEPS_PER_MM)
y_units = float(Y_STEPS_PER_MM)
z_units = float(Z_STEPS_PER_MM)
curve_section = CURVE_SECTION_INCHES

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
PRESURE_14="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x32\x30\x46\x30\x03\x04"
PRESURE_15="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x31\x35\x30\x45\x46\x03\x04"
PRESURE_20="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x32\x30\x30\x46\x33\x03\x04"
PRESURE_30="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x33\x30\x30\x46\x32\x03\x04"
PRESURE_40="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x34\x30\x30\x46\x31\x03\x04"
PRESURE_50="\x05\x02\x30\x38\x50\x53\x20\x20\x30\x35\x30\x30\x46\x30\x03\x04"
PRESURE_steady="\x05\x02\x30\x34\x4D\x54\x20\x20\x42\x42\x03\x04"##设置为连续点胶模式
PRESURE_start_end="\x05\x02\x30\x34\x44\x49\x20\x20\x43\x46\x03\x04"##开启和关闭点胶

PRESURE_Supporting=PRESURE_2_5####T0右边电磁阀
PRESURE_Conductive=PRESURE_3#####T1左边电磁阀
PRESURE_IC=PRESURE_2_5####T2右边电磁阀


#x_direction = 0
#y_direction = 0
#z_direction = 0

feedrate = 0.0      ###就是速度
feedrate_micros = 0 ###脉冲延迟时间
serial_count=0      ###数组

fpx=0
fpy=0
fpz=0

Extrude_old_state=0
Extrude_new_state=0
Extrude_change=0
Extrude_change_x=60
Extrude_change_z=20


#---------定义结构体---------#

class directions:
	def __init__(self, xpart0,ypart0,zpart0):
		self.x = xpart0
		self.y = ypart0
		self.z = zpart0

direction = directions(0,0,0)

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

class deltaunits:
	def __init__(self, xpart3,ypart3,zpart3):
		self.x = xpart3
		self.y = ypart3
		self.z = zpart3

delta_units = deltaunits(0.0,0.0,0.0)

class currentsteps:
	def __init__(self, xpart4,ypart4,zpart4):
		self.x = xpart4
		self.y = ypart4
		self.z = zpart4

current_steps = currentsteps(0.0,0.0,0.0)

class targetsteps:
	def __init__(self, xpart5,ypart5,zpart5):
		self.x = xpart5
		self.y = ypart5
		self.z = zpart5

target_steps = targetsteps(0.0,0.0,0.0)

class deltasteps:
	def __init__(self, xpart6,ypart6,zpart6):
		self.x = xpart6
		self.y = ypart6
		self.z = zpart6

delta_steps = deltasteps(0.0,0.0,0.0)



#class fps:
#	def __init__(self, xpart4,ypart4,zpart4):
#		self.x = xpart4
#		self.y = ypart4
#		self.z = zpart4

#fp = fps(0.0,0.0,0.0)



def  init_steppers():

	#current_units.x = 0.0;
	#current_units.y = 0.0;
	#current_units.z = 0.0;
	#target_units.x = 0.0;
	#target_units.y = 0.0;
	#target_units.z = 0.0;

	#-----------引脚初始化--------#
	gpio.wiringPiSetup()  #初始化

	gpio.pinMode(X_STEP_PIN,GPIO.OUTPUT)
	gpio.pinMode(X_DIR_PIN,GPIO.OUTPUT)
	gpio.pinMode(X_ENABLE_PIN,GPIO.OUTPUT)
	gpio.pinMode(X_MIN_PIN,GPIO.INPUT)

	gpio.pinMode(Y_STEP_PIN,GPIO.OUTPUT)
	gpio.pinMode(Y_DIR_PIN,GPIO.OUTPUT)
	gpio.pinMode(Y_ENABLE_PIN,GPIO.OUTPUT)
	gpio.pinMode(Y_MIN_PIN,GPIO.INPUT)

	gpio.pinMode(Z_STEP_PIN,GPIO.OUTPUT) 
	gpio.pinMode(Z_DIR_PIN,GPIO.OUTPUT) 
	gpio.pinMode(Z_ENABLE_PIN,GPIO.OUTPUT)
	gpio.pinMode(Z_MIN_PIN,GPIO.INPUT) 

	gpio.pinMode(Extrude_0,GPIO.OUTPUT)
	gpio.pinMode(Extrude_1,GPIO.OUTPUT)
	gpio.pinMode(Extrude_2,GPIO.OUTPUT)

	gpio.digitalWrite(Extrude_0,GPIO.HIGH) #使能
	gpio.digitalWrite(Extrude_1,GPIO.LOW) #不使能
	gpio.digitalWrite(Extrude_2,GPIO.LOW) #不使能
	
	order_string=ser.write(PRESURE_steady)
	order_string=ser.write(PRESURE_Supporting)


	gpio.digitalWrite(X_ENABLE_PIN,GPIO.LOW) #不使能
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW) #不使能
	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW) #不使能

	gpio.digitalWrite(X_REMOVEWARN_PIN,GPIO.LOW) 
	gpio.digitalWrite(Y_REMOVEWARN_PIN,GPIO.LOW) 
	gpio.digitalWrite(Z_REMOVEWARN_PIN,GPIO.LOW) 
	
	gpio.delay(1000)

	gpio.digitalWrite(X_REMOVEWARN_PIN,GPIO.HIGH) #给脉冲，消除警报
	gpio.digitalWrite(Y_REMOVEWARN_PIN,GPIO.HIGH) 
	gpio.digitalWrite(Z_REMOVEWARN_PIN,GPIO.HIGH)
	
	gpio.delay(1000)
	
	gpio.digitalWrite(X_REMOVEWARN_PIN,GPIO.LOW) 
	gpio.digitalWrite(Y_REMOVEWARN_PIN,GPIO.LOW) 
	gpio.digitalWrite(Z_REMOVEWARN_PIN,GPIO.LOW)

	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.HIGH) #使能
	gpio.digitalWrite(X_ENABLE_PIN,GPIO.HIGH) #使能
	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.HIGH) #使能

def set_target(x,y,z):
	target_units.x = round(x,2)
	target_units.y = round(y,2)
	target_units.z = round(z,2)
	calculate_deltas()


def set_position(x,y,z):
	current_units.x = round(x,2)
	current_units.y = round(y,2)
	current_units.z = round(z,2)
	calculate_deltas()


def calculate_deltas():

	delta_units.x = float(abs(target_units.x - current_units.x))
	delta_units.y = float(abs(target_units.y - current_units.y))
	delta_units.z = float(abs(target_units.z - current_units.z))
				
	####set our steps current, target, and delta
	current_steps.x = long(to_steps(x_units, current_units.x))
	current_steps.y = long(to_steps(y_units, current_units.y))
	current_steps.z = long(to_steps(z_units, current_units.z))

	target_steps.x = long(to_steps(x_units, target_units.x))
	target_steps.y = long(to_steps(y_units, target_units.y))
	target_steps.z = long(to_steps(z_units, target_units.z))

	delta_steps.x = long(abs(target_steps.x - current_steps.x))
	delta_steps.y = long(abs(target_steps.y - current_steps.y))
	delta_steps.z = long(abs(target_steps.z - current_steps.z))
	
	#what is our direction
	direction.x = int(target_units.x >= current_units.x)
	direction.y = int(target_units.y >= current_units.y)
	direction.z = int(target_units.z >= current_units.z)

	#set our direction pins as well
	gpio.digitalWrite(X_DIR_PIN,direction.x)
	gpio.digitalWrite(Y_DIR_PIN,direction.y)
	gpio.digitalWrite(Z_DIR_PIN,direction.z)


def to_steps(steps_per_unit,units):
	return steps_per_unit * units


def getcode(key,lines):  #search_string  未经测试函数
	length=len(lines)
	i=lines.find(key)
	i=i+1
	a=''
	while (lines[i]!=' '):
		a=a+lines[i]
		i=i+1
		if i==length:
			number=abs(float(a))
			return number
	number=abs(float(a))
	return number


def getMaxSpeed():
	if delta_steps.z > 0:
		return calculate_feedrate_delay(FAST_Z_FEEDRATE)
	else:
		return calculate_feedrate_delay(FAST_XY_FEEDRATE)


def calculate_feedrate_delay(feedrate):#####我们只要做xy的时间就行了，注意函数的return，已修改
	##how long is our line length?
	###distance = math.sqrt(delta_units.x*delta_units.x + delta_units.y*delta_units.y + delta_units.z*delta_units.z)
	###distance和feedrate单位毫米
	distance = math.sqrt(delta_units.x*delta_units.x + delta_units.y*delta_units.y)
	global master_steps
	##find the dominant axis
#	if delta_steps.x > delta_steps.y:##取最大的变化步数
#		if delta_steps.z > delta_steps.x:
#			master_steps = delta_steps.z
#		else:
#			master_steps = delta_steps.x
#	else:
#		if delta_steps.z > delta_steps.y:
#			master_steps = delta_steps.z
#		else:
#			master_steps = delta_steps.y

##取最大的变化步数
	master_steps = delta_steps.x + delta_steps.y

	if delta_steps.z!=0:
		distance = delta_units.z
		master_steps = delta_steps.z
	if master_steps!=0:
		return ((distance * 60000000.0) / float(feedrate)) / master_steps####得到微秒，feedrate单位秒
	else:
		return ((distance * 60000000.0) / float(feedrate)) / 1000  ####XYZ均无移动
	

	##calculate delay between steps in microseconds.  this is sort of tricky, but not too bad.
	##the formula has been condensed to save space.  here it is in english:
	##distance / feedrate * 60000000.0 = move duration in microseconds
	##move duration / master_steps = time between steps for master axis.

def dda_move(micro_delay):###micro_delay脉冲之间的间隔时间

	x_can_step = False#########此处该为布尔型变量
	y_can_step = False
	z_can_step = False

	x_start=0
	y_start=0
	z_start=0

	global Extrude_new_state
	global Extrude_old_state
	
	calculate_deltas()

	gpio.digitalWrite(X_DIR_PIN,direction.x)
	gpio.digitalWrite(Y_DIR_PIN,direction.y)
	gpio.digitalWrite(Z_DIR_PIN,direction.z)

#	max_delta = max(delta_steps.x, delta_steps.y)##max_delta是需要走的最多步数，delta_steps需要走的步数
#	max_delta = max(delta_steps.z, max_delta)

#	x_counter = -max_delta/2
#	y_counter = -max_delta/2
#	z_counter = -max_delta/2


	#if micro_delay >= 16383:##micro_delay就是feedrate_micros，脉冲间隔时间，用来调节速度
	#	milli_delay = long(micro_delay)/ 1000  #单位ms
	#else:
	#	milli_delay = 0

	x_target = abs(target_steps.x-current_steps.x)
	y_target = abs(target_steps.y-current_steps.y)
	z_target = abs(target_steps.z-current_steps.z)

	if (x_start!=x_target) and (y_start!=y_target):
		#print 'XY均有移动'

		##插补算法，k为斜率
		k = float(y_target)/float(x_target)
		if Extrude_new_state!=Extrude_old_state:
			order_string=ser.write(PRESURE_start_end)##挤出头工作
			gpio.delay(200)
		Extrude_old_state=Extrude_new_state

		while True:
			###x_can_step = can_step(X_MIN_PIN, X_MAX_PIN, current_steps.x, target_steps.x, direction.x)
			x_can_step = can_step(0, x_start, x_target, direction.x)
			y_can_step = can_step(0, y_start, y_target, direction.y)
			#print x_can_step
			#print y_can_step
				
			if (y_start >= float(x_start*k)) and x_can_step:
				
				gpio.digitalWrite(X_DIR_PIN,direction.x)
				#print '插补direction.x=',direction.x
				do_step(X_STEP_PIN)
				x_start = x_start + 1
	
				gpio.delayMicroseconds(int(micro_delay))###微秒

				##if abs(x_start-x_target)<1:
				##	x_start = x_target
				##	y_start = y_target

			elif (y_start < float(x_start*k)) and y_can_step:
				gpio.digitalWrite(Y_DIR_PIN,direction.y)
				#print '插补direction.y=',direction.y
				do_step(Y_STEP_PIN)
				y_start = y_start + 1

				gpio.delayMicroseconds(int(micro_delay))###微秒

				##if abs(y_start-y_target)<1:
				##	x_start = x_target
				##	y_start = y_target

			#if milli_delay > 0:
			#	gpio.delay(int(milli_delay))###毫秒
			#else:
			#	gpio.delayMicroseconds(int(micro_delay))###微秒

			if not (x_can_step or y_can_step):
				break


	elif (current_steps.x!=target_steps.x) and (current_steps.y==target_steps.y):
		#print '仅X有移动'
		x_target=abs(target_steps.x-current_steps.x)
		x_start=0

		if Extrude_new_state!=Extrude_old_state:
			order_string=ser.write(PRESURE_start_end)##挤出头工作
			gpio.delay(200)
		Extrude_old_state=Extrude_new_state

		while True:
			gpio.digitalWrite(X_DIR_PIN,direction.x)
			do_step(X_STEP_PIN)
			x_start = x_start + 1
			if abs(x_start-x_target)<1:
					x_start = x_target
			x_can_step = can_step(X_MIN_PIN, x_start, x_target, direction.x)

			#if milli_delay > 0:
			#	gpio.delay(int(milli_delay))###毫秒
			#else:
			gpio.delayMicroseconds(int(micro_delay))###微秒

			if not x_can_step:
				break


	elif (current_steps.x==target_steps.x) and (current_steps.y!=target_steps.y):
		#print '仅Y有移动'
		y_target=abs(target_steps.y-current_steps.y)
		y_start=0

		if Extrude_new_state!=Extrude_old_state:
			order_string=ser.write(PRESURE_start_end)##挤出头工作
			gpio.delay(200)
		Extrude_old_state=Extrude_new_state

		while True:
			gpio.digitalWrite(Y_DIR_PIN,direction.y)
			do_step(Y_STEP_PIN)
			y_start = y_start + 1
			if abs(y_start-y_target)<1:
					y_start = y_target
			y_can_step = can_step(Y_MIN_PIN, y_start, y_target, direction.y)

			#if milli_delay > 0:
			#gpio.delay(int(milli_delay))###毫秒
			#else:
			gpio.delayMicroseconds(int(micro_delay))###微秒
			if not y_can_step:
				break


	elif current_steps.z!=target_steps.z:
		#print '仅Z有移动'
		z_target=abs(target_steps.z-current_steps.z)
		z_start=0

		if Extrude_new_state!=Extrude_old_state:
			order_string=ser.write(PRESURE_start_end)##挤出头工作
			gpio.delay(200)
		Extrude_old_state=Extrude_new_state

		while True:
			gpio.digitalWrite(Z_DIR_PIN,direction.z)
			do_step(Z_STEP_PIN)
			z_start = z_start + 1
			if (z_target-z_start)<1:
					z_start = z_target
			z_can_step = can_step(Z_MIN_PIN, z_start, z_target, direction.z)

			#if milli_delay > 0:
			#	gpio.delay(int(milli_delay))###毫秒
			#else:
			gpio.delayMicroseconds(int(micro_delay))###微秒
			if not z_can_step:
				break

	current_units.x = target_units.x
	current_units.y = target_units.y
	current_units.z = target_units.z
	#print 'current_units.x=',current_units.x
	#print 'current_units.y=',current_units.y
	

#???????????????????????检查注意这里函数的入口参数????????????????????????###########


####def can_step(byte min_pin, byte max_pin, long current, long target, byte direction)

def can_step(min_pin,current,target,direction):###can_step是重要的，要修改符合实际的反馈
	##stop us if we're on target
	if target <= current:
		return False

	# MIN1=gpio.digitalRead(min_pin)
	# MIN2=gpio.digitalRead(min_pin)
	# MIN3=gpio.digitalRead(min_pin)
	# if MIN1 and MIN2 and MIN3 and (not direction):
	# 	return False

	##default to being able to step
	return True

def do_step(step_pin):######步进电机走一步
	gpio.digitalWrite(step_pin,GPIO.HIGH)
	gpio.delayMicroseconds(1)
	gpio.digitalWrite(step_pin,GPIO.LOW)###发一个脉冲

def return_home():
	gpio.digitalWrite(X_DIR_PIN,GPIO.LOW)##方向向右上顶点
	gpio.digitalWrite(Y_DIR_PIN,GPIO.LOW)
	gpio.digitalWrite(Z_DIR_PIN,GPIO.LOW)

	tx=int(30000000/(X_STEPS_PER_MM*FAST_XY_FEEDRATE)/2)
	ty=int(30000000/(Y_STEPS_PER_MM*FAST_XY_FEEDRATE)/2)
	tz=int(30000000/(Z_STEPS_PER_MM*FAST_Z_FEEDRATE)/2)

	# while True:
	# 	gpio.digitalWrite(X_STEP_PIN,GPIO.HIGH)
	# 	gpio.delayMicroseconds(tx)
	# 	gpio.digitalWrite(X_STEP_PIN,GPIO.LOW)###发一个脉冲
	# 	gpio.delayMicroseconds(tx)
	# 	MINX1=gpio.digitalRead(X_MIN_PIN)
	# 	MINX2=gpio.digitalRead(X_MIN_PIN)
	# 	MINX3=gpio.digitalRead(X_MIN_PIN)
	# 	if MINX1==0 and MINX2==0 and MINX3==0:
	# 		break
	while 1:
		gpio.digitalWrite(Y_STEP_PIN,GPIO.HIGH)
		gpio.delayMicroseconds(ty)
		gpio.digitalWrite(Y_STEP_PIN,GPIO.LOW)###发一个脉冲
		gpio.delayMicroseconds(ty)
		MIN1=gpio.digitalRead(Y_MIN_PIN)
		MIN2=gpio.digitalRead(Y_MIN_PIN)
		MIN3=gpio.digitalRead(Y_MIN_PIN)
		if MIN1==0 and MIN2==0 and MIN3==0:
			break
	while 1:
		gpio.digitalWrite(X_STEP_PIN,GPIO.HIGH)
		gpio.delayMicroseconds(tx)
		gpio.digitalWrite(X_STEP_PIN,GPIO.LOW)###发一个脉冲
		gpio.delayMicroseconds(tx)
		MIN1=gpio.digitalRead(X_MIN_PIN)
		MIN2=gpio.digitalRead(X_MIN_PIN)
		MIN3=gpio.digitalRead(X_MIN_PIN)
		if MIN1==0 and MIN2==0 and MIN3==0:
			break	
	while 1:
		gpio.digitalWrite(Z_DIR_PIN,GPIO.LOW)
		gpio.digitalWrite(Z_STEP_PIN,GPIO.HIGH)
		gpio.delayMicroseconds(30)
		gpio.digitalWrite(Z_STEP_PIN,GPIO.LOW)###发一个脉冲
		gpio.delayMicroseconds(30)
		MIN1=gpio.digitalRead(Z_MIN_PIN)
		MIN2=gpio.digitalRead(Z_MIN_PIN)
		MIN3=gpio.digitalRead(Z_MIN_PIN)
		if MIN1==0 and MIN2==0 and MIN3==0:
			break


############主函数###################################################
f=open('gcode.txt','r')
ser=serial.Serial("/dev/ttyUSB0",9600,timeout=0.5)
ser.open
init_steppers()

while True:
	line=f.readline()  #读取一行数据

	if line=='':
		break

	if line.find('T')!=-1:##找到挤出头，默认是0号挤出头为标准位置
		code = getcode('T',line)
		##如果code是1则换1号挤出头

		if line.find('E')!=-1:
			Extrude_new_state=1
		else:
			Extrude_new_state=0

		if Extrude_new_state!=Extrude_old_state:
			order_string=ser.write(PRESURE_start_end)##挤出头工作
			gpio.delay(200)
		Extrude_old_state=Extrude_new_state

		if code==0:
			gpio.digitalWrite(Extrude_0,GPIO.HIGH)
			gpio.digitalWrite(Extrude_1,GPIO.LOW)
			gpio.digitalWrite(Extrude_2,GPIO.LOW)
			order_string=ser.write(PRESURE_Supporting)

		##否则使用1号挤出头
		elif code==1:
			gpio.digitalWrite(Extrude_0,GPIO.LOW)
			gpio.digitalWrite(Extrude_1,GPIO.HIGH)
			gpio.digitalWrite(Extrude_2,GPIO.LOW)
			order_string=ser.write(PRESURE_Conductive)
		
		##否则使用2号挤出头
		elif code==2:
			gpio.digitalWrite(Extrude_0,GPIO.LOW)
			gpio.digitalWrite(Extrude_1,GPIO.LOW)
			gpio.digitalWrite(Extrude_2,GPIO.HIGH)
			order_string=ser.write(PRESURE_IC)

		gpio.delay(1000)

		master_steps_z=long(Extrude_change_z*Z_STEPS_PER_MM)
		master_steps_x=long(Extrude_change_x*X_STEPS_PER_MM)
		feedrate_micros_z=((Extrude_change_z * 60000000.0) / float(FAST_Z_FEEDRATE)) / master_steps_z
		feedrate_micros_x=((Extrude_change_x * 60000000.0) / float(FAST_XY_FEEDRATE)) / master_steps_x

		if code > Extrude_change:
			fpx = current_units.x
			fpy = current_units.y
			fpz = current_units.z + Extrude_change_z
			set_target(fpx,fpy,fpz)
			dda_move(feedrate_micros_z*2)
			fpx = current_units.x - Extrude_change_x
			set_target(fpx,fpy,fpz)
			dda_move(feedrate_micros_x*2)
			fpz = abs(current_units.z - Extrude_change_z)
			set_target(fpx,fpy,fpz)
			dda_move(feedrate_micros_z*2)
			Extrude_change = 1
		elif code < Extrude_change:
			fpx = current_units.x
			fpy = current_units.y
			fpz = current_units.z + Extrude_change_z
			set_target(fpx,fpy,fpz)
			dda_move(feedrate_micros_z*2)
			fpx = current_units.x + Extrude_change_x
			if fpx>=border_x:
			 	gpio.digitalWrite(X_ENABLE_PIN,GPIO.LOW) #pin25输出为低电平
			 	gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW) #pin25输出为低电平
			 	gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW) #pin25输出为低电平
			 	break
			set_target(fpx,fpy,fpz)
			dda_move(feedrate_micros_x*2)
			fpz = abs(current_units.z - Extrude_change_z)
			set_target(fpx,fpy,fpz)
			dda_move(feedrate_micros_z*2)
			Extrude_change = 0


#	if line.find('P')!=-1:
#		PRESURE=getcode


	if line.find('X')!=-1:		##找到X则执行
		fpx = getcode('X',line)
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

	if line.find('F')!=-1:
		feedrate = getcode('F', line)
	elif line.find('F')==-1 and line.find('Z')!=-1:
		feedrate = FAST_Z_FEEDRATE
	else:
		feedrate = FAST_XY_FEEDRATE

	if line.find('E')!=-1:
		Extrude_new_state=1
	else:
		Extrude_new_state=0

	if line.find('G')==0:
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

		# if code==0:
		# 	set_target(fpx,fpy,fpz)
		# 	feedrate_micros = getMaxSpeed()  ##G0默认进给速度
		# 	dda_move(feedrate_micros)

		if code==1:   ##how fast do we move?
			set_target(fpx,fpy,fpz)
			feedrate_micros = calculate_feedrate_delay(feedrate)
			dda_move(feedrate_micros)
			#print 'current_units.z=',current_units.z,'target_units.z=',target_units.z
			#print 'current_units.x=',current_units.x,'target_units.x=',target_units.x
			#print 'current_units.y=',current_units.y,'target_units.y=',target_units.y
			#print 'target_direction.x=',direction.x,'targrt_direction.y=',direction.y

			#if line.find('F')!=-1:#不等于-1，则找到F，否则采用最大默认速度，单位mm/min
			#	feedrate = getcode('F', line)
			#	print 'feedrate=',feedrate
						
			#	feedrate_micros = calculate_feedrate_delay(feedrate)###脉冲之间的间隔时间
			#	print 'feedrate_micros=',feedrate_micros
			#	dda_move(feedrate_micros)
				#print('G1AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
			#else:
			#	feedrate_micros = getMaxSpeed()
			#	dda_move(feedrate_micros)

		##else:
		##	if feedrate > 0:
		##		feedrate_micros = calculate_feedrate_delay(feedrate)
		##		##nope, no feedrate
		##	else:
		##		feedrate_micros = getMaxSpeed()
		##	dda_move(feedrate_micros)

		##暂停移动
		elif code==4:
			delay(int(getcode('P',line)))

		###Inches for Units设置当前距离单位为英寸(G20)
		# elif code==20:
		# 	x_units = X_STEPS_PER_INCH
		# 	y_units = Y_STEPS_PER_INCH
		# 	z_units = Z_STEPS_PER_INCH
		# 	##curve_section = CURVE_SECTION_INCHES
		# 	calculate_deltas()
			
		###设置单位为毫米
		elif code==21:
			x_units = X_STEPS_PER_MM
			y_units = Y_STEPS_PER_MM
			z_units = Z_STEPS_PER_MM
			##curve_section = CURVE_SECTION_MM
			calculate_deltas()

		##归零
		elif code==28:
			#set_target(0.0, 0.0, 0.0)
			#dda_move(getMaxSpeed())
			return_home()


		elif code==90:##设置绝对坐标值
			current_units = currentunits(0.0,0.0,0.0)
			current_steps = currentsteps(0.0,0.0,0.0)


        ##设置3D打印机内存中XYZE的位置值，不移动对应的步进电机
		# elif code==92:
		# 	set_position(0.0, 0.0, 0.0)


		else:## code==11:
			if Extrude_new_state!=Extrude_old_state:
				order_string=ser.write(PRESURE_start_end)##挤出头工作
				gpio.delay(200)
			gpio.digitalWrite(X_ENABLE_PIN,GPIO.LOW) #pin25输出为低电平
			gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW) #pin25输出为低电平
			gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW) #pin25输出为低电平
			gpio.digitalWrite(Extrude_0,GPIO.LOW)
			gpio.digitalWrite(Extrude_1,GPIO.LOW)



