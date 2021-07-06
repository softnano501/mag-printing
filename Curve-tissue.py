#!/usr/bin/python
# -*- coding: utf-8 -*-
import wiringpi as gpio
from wiringpi import GPIO
import numpy as np
import math
import serial
import threading
import xlrd

X_STEPS_PER_MM=1280.0
X_MM_PER_STEP=float(1.0/X_STEPS_PER_MM)

##32个脉冲1.8°，6400脉冲小齿轮一圈，齿轮比15，转盘一圈96000个脉冲
Y_STEPS_PER_Rotation=96000.0
Y_STEPS_PER_ARC=Y_STEPS_PER_Rotation/(2*math.pi)##电机转1弧度需要多少脉冲
Y_ARC_PER_STEP=float(2*math.pi/Y_STEPS_PER_Rotation)

####真实Z_STEPS_PER_MM是6630.4，为了方便计算近似为6400
Z_STEPS_PER_MM=1280

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

#设置两台电机的频率和步数
motor1_delay = 3500
motor1_steps = 180000

motor2_delay = 100
motor2_steps = 96000

motorC_delay=50

steps_x=0
steps_y=0

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

PRESURE1=PRESURE_10
PRESURE2=PRESURE_20
PRESURE3=PRESURE_20

gpio.wiringPiSetup() #初始化

gpio.pinMode(X_STEP_PIN1,GPIO.OUTPUT)
gpio.pinMode(X_DIR_PIN1,GPIO.OUTPUT)
gpio.pinMode(X_ENABLE_PIN1,GPIO.OUTPUT)

gpio.pinMode(X_STEP_PIN2,GPIO.OUTPUT)
gpio.pinMode(X_DIR_PIN2,GPIO.OUTPUT)
gpio.pinMode(X_ENABLE_PIN2,GPIO.OUTPUT)

gpio.pinMode(Y_STEP_PIN,GPIO.OUTPUT)
gpio.pinMode(Y_DIR_PIN,GPIO.OUTPUT)
gpio.pinMode(Y_ENABLE_PIN,GPIO.OUTPUT)

gpio.pinMode(Z_STEP_PIN,GPIO.OUTPUT)
gpio.pinMode(Z_DIR_PIN,GPIO.OUTPUT) 
gpio.pinMode(Z_ENABLE_PIN,GPIO.OUTPUT)

gpio.pinMode(C_STEP_PIN,GPIO.OUTPUT)
gpio.pinMode(C_DIR_PIN,GPIO.OUTPUT) 
gpio.pinMode(C_ENABLE_PIN,GPIO.OUTPUT)

gpio.digitalWrite(X_ENABLE_PIN1,GPIO.LOW) #低电平使能
gpio.digitalWrite(X_ENABLE_PIN2,GPIO.LOW) #低电平使能
gpio.digitalWrite(Y_ENABLE_PIN,GPIO.LOW)
gpio.digitalWrite(Z_ENABLE_PIN,GPIO.LOW)
gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)


#---------定义结构体---------#

class directions:
	def __init__(self,xpart0,ypart0,zpart0,cpart0):
		self.x = xpart0
		self.y = ypart0
		self.z = zpart0
		self.c = cpart0

direction = directions(0,0,0,0)
#定义电机的类
class Motor:
    #def __init__(self,I1,I2,I3,I4):
    def __init__(self,I1,I2,I3):
    #初始化类的参数
        self.I1 = I1
        self.I2 = I2
        self.I3 = I3
        # GPIO.setwarnings(False)
        # GPIO.setmode(GPIO.BOARD)      # 设置寻址模式，按照pin引脚寻址
        gpio.wiringPiSetup()  #初始化
        gpio.pinMode(self.I1, GPIO.OUTPUT) # 把pin设置为输出模式
        gpio.pinMode(self.I2, GPIO.OUTPUT) # 把pin设置为输出模式
        gpio.pinMode(self.I3, GPIO.OUTPUT) # 把pin设置为输出模式

    def loop(self,delay_loop,steps_loop,steps_axis):
        Motor.forward(self,delay_loop,steps_loop,steps_axis)
        Motor.stop(self)
        gpio.delayMicroseconds(1000)

    def setStep(self, w1, w2, w3):
        #GPIO.output（）---指定脚位输出high还是low
        gpio.digitalWrite(self.I1, w1)#.output（in1，1）代表指定IN1引脚输出的是高电平，0就是低电平
        gpio.digitalWrite(self.I2, w2)
        gpio.digitalWrite(self.I3, w3)
            
    def forward(self, delay, steps, axis):
    	global steps_x,steps_y
    	if axis==1:
	        for i in range(0, steps):
	            Motor.setStep(self,1, 1, 0)
	            gpio.delayMicroseconds(delay)
	            Motor.setStep(self,0, 1, 0)
	            gpio.delayMicroseconds(delay)
	            steps_x+=1
    	elif axis==2:
	        for i in range(0, steps):
	            Motor.setStep(self,1, 1, 0)
	            gpio.delayMicroseconds(delay)
	            Motor.setStep(self,0, 1, 0)
	            gpio.delayMicroseconds(delay)
	            steps_y+=1
    	else :
    		for i in range(0, steps):
	            Motor.setStep(self,1, 1, 0)
	            gpio.delayMicroseconds(delay)
	            Motor.setStep(self,0, 1, 0)
	            gpio.delayMicroseconds(delay)

    def stop(self):
        Motor.setStep(self,0, 0, 0)

def do_step(step_pin):######步进电机走一步
	gpio.digitalWrite(step_pin,GPIO.HIGH)
	gpio.delayMicroseconds(10)
	gpio.digitalWrite(step_pin,GPIO.LOW)###发一个脉冲

def destroy():
    gpio.wiringPiSetup()  #初始化

##Z运动函数
def Function(x,y):
	global X_mm,Y_mm,i,j,X_tip,Y_tip
	R_mag=x*X_MM_PER_STEP+5
	###映射转换
	R_tip=R_mag*0.65
	R_tip=10
	theta=float(y)*2.0*math.pi/Y_STEPS_PER_Rotation
	X_tip=R_tip*round(math.cos(theta),15)
	Y_tip=R_tip*round(math.sin(theta),15)
	# X=int(X_tip*20)/20.0
	# Y=int(Y_tip*20)/20.0
	# j=int(X*20+401)-1
	# i=int(401-Y*20)-1
	X=round(X_tip*100)/100.0
	Y=round(Y_tip*100)/100.0
	j=int(X*10+301)-1##X为j
	i=int(301-Y*10)-1##Y为i
	Z_function=matrix[i,j]
	return Z_function

def dda_move_C(C_delay):
	global steps_x,steps_y,C_now,C_old,Z_function_steps,C_function_steps_old,C_function_steps_new
	gpio.digitalWrite(C_ENABLE_PIN,GPIO.LOW)
	delta_length_steps=0
	##Attention C_old初始位置需先确定
	#C_old=0##X初始为10，即6.5
	C_old=matrix[300,400]
	C_now=0
	C_function_steps_old=0
	C_function_steps_new=0
	p=0
	#while True:
		# C_now=Function(steps_x,steps_y)
		# C_function_steps=(C_now-C_old)*C_STEPS_PER_MM
		# r_now=steps_x*X_MM_PER_STEP+10
		# C_function_steps_new=Function(steps_x,steps_y)*C_STEPS_PER_MM
		# C_function_steps=C_function_steps_new-C_function_steps_old
		#delta_length_steps=C_function_steps
		# if r_now == 0:
		# 	delta_length_steps=C_function_steps
		# elif 0<abs(r_now)<=5:
		# 	delta_length_steps=C_function_steps-10*(r_now-r_old)
		# elif 5<r_now<=15:
		#  	delta_length_steps=C_function_steps-((r_now*16+2.1*r_now**2)-(r_old*16+2.1*r_old**2))
		# elif 15<r_now<22:
		#  	delta_length_steps=C_function_steps-((r_now*16+2.5*r_now**2)-(r_old*16+2.5*r_old**2))
		# elif 22<abs(r_now)<30:
		# 	delta_length_steps=C_function_steps-((r_now*16+2.8*r_now**2)-(r_old*16+2.8*r_old**2))
		# elif -5<=r_now<0:
		# 	delta_length_steps=abs(8*(r_now-r_old))
		# elif -15<=r_now<-5:
		# 	delta_length_steps=abs((abs(r_now)*16+1.8*r_now**2)-(abs(r_old)*16+1.8*r_old**2))
		# elif -22<=r_now<-15:
		# 	delta_length_steps=abs((abs(r_now)*16+2.1*r_now**2)-(abs(r_old)*16+2.1*r_old**2))
		# elif -30<=r_now<-22:
		# 	delta_length_steps=abs((abs(r_now)*16+2.7*r_now**2)-(abs(r_old)*16+2.7*r_old**2))
		# C_direction=bool(C_function_steps>=0)
		# gpio.digitalWrite(C_DIR_PIN,C_direction)
		# gpio.delayMicroseconds(60)
		# while True:
		# 	do_step(C_STEP_PIN)
		# 	#gpio.delayMicroseconds(C_delay)
		# 	p+=1
		# 	if p>=abs(C_function_steps):
		# 		p=0
		# 		C_old=C_now
		# 		C_function_steps=0
		# 		break

	while True:
		C_now=Function(steps_x,steps_y)
		t=C_now-C_old
		C_function_steps=(t)*C_STEPS_PER_MM
		# if 0<abs(C_now)<=5:
		# 	C_function_steps=C_function_steps-10*(t)
		# elif 5<C_now<=15:
		#  	C_function_steps=C_function_steps-(t*16+2.1*t**2)
		# elif 15<C_now<22:
		#  	C_function_steps=C_function_steps-(t*16+3*t**2)
		# elif 22<abs(C_now)<30:
		# 	C_function_steps=C_function_steps-(t*16+3.3*t**2)
		# if C_function_steps!=0:
		#   	print 'C_function_steps,matrix[i,j]',C_function_steps,matrix[i,j]
		#   	print 'i,j',i,j
		C_direction=bool(C_function_steps>=0)
		gpio.digitalWrite(C_DIR_PIN,C_direction)
		gpio.delayMicroseconds(10)
		# if abs(Z_function_steps)!=0:
		# 	print 'Z_function_steps',Z_function_steps
	 	while (abs(C_function_steps)>10):
	 		do_step(C_STEP_PIN)
	 		gpio.delayMicroseconds(100)
	 		p+=1
	 		if p>=abs(C_function_steps):
	 			C_old=C_now
	 			C_function_steps=0
	 			p=0
	 			break
# def readFile(path):
#     # 打开文件（注意路径）
#     f = open('data.txt')
#     # 逐行进行处理
#     first_ele = True
#     for data in f.readlines():
#         ## 去掉每行的换行符，"\n"
#         data = data.strip('\n')
#         ## 按照 空格进行分割。
#         nums = data.split(" ")
#         ## 添加到 matrix 中。
#         if first_ele:
#             ### 将字符串转化为整型数据
#             nums = [int(x) for x in nums ]
#             ### 加入到 matrix 中 。
#             matrix = np.array(nums)
#             first_ele = False
#         else:
#             nums = [int(x) for x in nums]
#             matrix = np.c_[matrix,nums]
#     dealMatrix(matrix)
#     f.close()

def excel2m(path):#读excel数据转为矩阵函数
    data = xlrd.open_workbook(path)
    table = data.sheets()[0] # 获取excel中第一个sheet表
    nrows = table.nrows  # 行数
    ncols = table.ncols  # 列数
    datamatrix = np.zeros((nrows, ncols))
    for x in range(ncols):
        cols = table.col_values(x)
        cols1 = np.matrix(cols)  # 把list转换为矩阵进行矩阵操作
        datamatrix[:, x] = cols1 # 把数据进行存储
    datamatrix.transpose()
    return datamatrix

# def dealMatrix(matrix):
#     ## 一些基本的处理。
#     matrix = matrix.transpose()

def main():
	motor1 = Motor(X_STEP_PIN1,X_DIR_PIN1,X_ENABLE_PIN1)#实例化类
	#motor1.setup(IN1,IN2,IN3,IN4)
	motor2 = Motor(Y_STEP_PIN,Y_DIR_PIN,Y_ENABLE_PIN)#实例化类
	#motor2.setup(IN5,IN6,IN7,IN8)
	global steps_x,steps_y,Z_old,Z_now,r_old,r_now,Z_function_steps,motor1_axis,motor2_axis,matrix	

	matrix = excel2m("result_Z2.xlsx")
	if True:
		print '开始'
	##气压
	#order_string=ser.write(PRESURE1)
	#order_string=ser.write(PRESURE_start_end)
    	gpio.delay(6000)

	Z_old=matrix[300,400]
	#Z_old=0
	Z_now=0
	q=0
	#thread1 = threading.Thread(target=motor1.loop,args=(motor1_delay,motor1_steps,1))#创建线程1
	thread2 = threading.Thread(target=motor2.loop,args=(motor2_delay,motor2_steps,2))#创建线程2
	thread3 = threading.Thread(target=dda_move_C,args=(motorC_delay,))#创建线程36
	#thread1.setDaemon(True)
	thread2.setDaemon(True)
	thread3.setDaemon(True)
	#thread1.start()#开始线程1
	thread2.start()#开始线程2
	thread3.start()#开始线程3
	try:
		#filename='write_data.txt'
		while True:
			Z_now=Function(steps_x,steps_y)
			Z_function_steps=(Z_now-Z_old)*Z_STEPS_PER_MM
			# if Z_function_steps!=0:
			#   	#print 'matrix[i,j]',matrix[i,j]
			#   	with open(filename,'a') as f:
			#   		f.writelines(str(X_tip)+"\t")
			#   		f.writelines(str(Y_tip)+"\t")
			#   		f.writelines(str(matrix[i,j])+"\n")
			Z_direction=bool(Z_function_steps<=0)
			gpio.digitalWrite(Z_DIR_PIN,Z_direction)
			gpio.delayMicroseconds(10)
			# if abs(Z_function_steps)!=0:
			# 	print 'Z_function_steps',Z_function_steps
		 	while (abs(Z_function_steps)>20):
		 		gpio.digitalWrite(Z_STEP_PIN,GPIO.HIGH)
				gpio.delayMicroseconds(100)
				gpio.digitalWrite(Z_STEP_PIN,GPIO.LOW)###发一个脉冲
		 		gpio.delayMicroseconds(100)
		 		q+=1
		 		if q>=abs(Z_function_steps):
				   	# print 'Z_function_steps,matrix[i,j]',Z_function_steps,matrix[i,j]
				   	# print 'i,j',i,j
		 			Z_old=Z_now
		 			Z_function_steps=0
		 			q=0
		 			break
	except KeyboardInterrupt :# When 'Ctrl+C' is pressed, the child function destroy() will be  executed.
		destroy()
	    # if (thread1.isAlive()):
	    # 	pass
	    # else:
	    # 	order_string=ser.write(PRESURE_start_end)
	    # 	destroy()
	    # 	break




if __name__ == '__main__':
	main()
#原文链接：https://blog.csdn.net/weixin_44524040/java/article/details/90450315


