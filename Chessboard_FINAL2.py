
import board
import busio
import math
import RPi.GPIO as GPIO
import pigpio
from time import sleep
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_servokit import ServoKit
import tkinter as tk
from datetime import datetime

Pi = pigpio.pi()

if not Pi.connected:
    print("Failed to connect to pigpio daemon")
else:
    print("Connected to pigpio daemon")

#setting up the RPI board configuration
GPIO.setmode(GPIO.BCM)

#Import CHess engine: Stockfish
import chess.engine

# Path to your Stockfish binary
stockfish_path = "/home/lindelize/Stockfish/src/stockfish"

# Start the Stockfish engine
Stockfish = chess.engine.SimpleEngine.popen_uci(stockfish_path)
	
# Configure Stockfish
Stockfish.configure({"Hash": 16})
Stockfish.configure({"Threads": 2})
SkillLevel=20
Stockfish.configure({"Skill Level":SkillLevel})
MoveTime_Stockfish=1.0

# Create a chessboard for the game
Chessboard_board = chess.Board()
  
#setting up the RPI board configuration
GPIO.setmode(GPIO.BCM)

#Set font for the project
Chessboard_font_size = 24
Message_font_size = 18
Heading_font_size = 28
State_font_size = 52
 
Font = "Helvetica"

# Create a display window
window = tk.Tk()
window.title("The unbeaten automated chessboard...")
window.geometry("1200x800")

# Dimensions for the 8x8 matrix
display_rows, display_cols = 8, 8
display_block_size = 40  # Size of each square block

# Calculate the starting points to center each matrix in its respective canvas
display_x_start = (390 - (display_cols * display_block_size)) // 2
display_y_start = (340 - (display_rows * display_block_size)) // 2

#	--------------------------------------------------------------------
#								RPI PIN CONFIGURATION
#	--------------------------------------------------------------------

#Multiplexer pins
MUX_S3_pin=4		#S3
MUX_S2_pin=17		#S2
MUX_S1_pin=27		#S1
MUX_S0_pin=22		#S0
GPIO.setup(MUX_S3_pin, GPIO.OUT)
GPIO.setup(MUX_S2_pin, GPIO.OUT)
GPIO.setup(MUX_S1_pin, GPIO.OUT)
GPIO.setup(MUX_S0_pin, GPIO.OUT)

#setting up the pins that are used for the I2C
i2c=busio.I2C(board.SCL, board.SDA)

#creating an ADC object that uses i2c
ads=ADS.ADS1115(i2c)

#creating PCA9685 driver object

kit = ServoKit(channels=16, i2c=i2c)

kit.frequency = 200

kit.servo[0].actuation_range = 260
kit.servo[1].actuation_range = 260
kit.servo[2].actuation_range = 270

kit.servo[0].set_pulse_width_range(500, 2500)
kit.servo[1].set_pulse_width_range(500, 2500)
kit.servo[2].set_pulse_width_range(500, 2500)


#Large servos pins
Servo1=12
Servo2=13

#Linear Actuator pins
LA_pin1=20
LA_pin2=21

#Linear Actuator pins
ServoMicroPin=16

#Button pin
ButtonPin = 5

#RGB LED pins
LED_R = 13
LED_G = 19
LED_B = 26

#	--------------------------------------------------------------------
#								PARAMETERS
#	--------------------------------------------------------------------

#Robotic arm parameters (units in m)
l=0.35
s_tube = 0.02		#thickness of the Al tubing
t1=0.037		#distance between end of bracket(on the fixed side) and the turn axis
t2=0.027		#distance between end of bracket(on the moving side) and the turn axis
t3=-0.02		#distance from side of Al tubing to center at which piece is gripped

l1 = math.sqrt(math.pow((t1+t2+s_tube),2)+math.pow(l,2))
l2 = math.sqrt(math.pow((t2+t3),2)+math.pow(l,2))

#Chessboard and interface with robotic arm parameters
SquareSize = 0.06				# units in m
'''
The origin is defined between file D and E with a given verticle offset
'''
RobotArm_Offset = 0.15			# units in m

#PWM setup
Servo12PWM_f=330
MinDutyCycle= 42
MaxDutyCycle= 210
DutyCycleRange=MaxDutyCycle-MinDutyCycle

#Radians to PWM measure
Radians_per_DutyCycle= ((3/2)*math.pi)/DutyCycleRange		#Amount of radians the motor turns for a 1% increase in duty cycle.

#Chessboard and interface with robotic arm parameters
SquareSize = 0.06				# units in m
'''
The origin is defined between file D and E with a given verticle offset
'''
RobotArm_Offset = 0.1			# units in m


#Home positions
#Servo1_Degree_Zero=145
#Servo2_Degree_Zero=125

Servo1_Degree_Zero=128
Servo2_Degree_Zero=111

microServo_open = 130
microServo_close = 160

''' ------------------------------------------------------------------------------
								CLASSES
---------------------------------------------------------------------------------'''


class Servo:
	def __init__(self, pin, prevDegree, zero, number):
		self.pin=pin
		#self.prevDC=prevDC
		self.prevDegree=prevDegree
		self.zero=zero
		self.number=number

class ChessSquare:
	def __init__(self, ADC_channel, MUX_pin, Pos):
		
		#Chess square naming
		self.Position = Pos
		
		#MUX pin and ADC channel
		#decimal to binary conversion
		S3=MUX_pin//8
		S2=(MUX_pin-S3*8)//4
		S1=(MUX_pin-S3*8-S2*4)//2
		S0=(MUX_pin-S3*8-S2*4-S1*2)//1
		self.MUX_pin=[S0, S1, S2, S3]
		
		#ADC channel identification
		if ADC_channel==0:
			self.ADC_channel=ADS.P0
		if ADC_channel==1:
			self.ADC_channel=ADS.P1
		if ADC_channel==2:
			self.ADC_channel=ADS.P2
		if ADC_channel==3:
			self.ADC_channel=ADS.P3
						
		#Unspecified fields
		self.calibrated_P_ID=0
		self.measured_P_ID=0
		self.chessPiece="."
		self.pieceColour="."
		self.x_pos=None
		self.y_pos=None
		
	def DeterminePosition(self):
		
		x = self.x_pos
		y = self.y_pos
		
		#print("x",x)
		#print("y",y)
		
		#Solve linear algebra for the chess square
		
		#Determine the angles alpha for the 2 link schematic
		r = math.sqrt(math.pow(x,2)+math.pow(y,2))
		#print("r",r)		
		
		Alpha2 = math.acos((math.pow(l1,2)+math.pow(l2,2)-math.pow(r,2))/(2*l1*l2))
		#print("Alpha2",Alpha2)
		
		#Alpha1 = math.pi - math.atan(y/x) - math.atan((l2*math.sin(Alpha2))/(l1-l2*math.cos(Alpha2)))
		
		Phi = math.atan(y/x)
		
		if Phi < 0:
			Phi = math.pi - abs(Phi)
		#print("Phi",Phi)	
			
		Alpha1 = math.pi - Phi - math.acos((math.pow(l1,2)+math.pow(r,2)-math.pow(l2,2))/(2*l1*r))
		#print("Alpha1",Alpha1)
		
		if (Alpha1 > (math.pi/2)):
			Alpha1 = Alpha1 - math.pi
		
		#Translate alpha to theta
		#print("t1",t1,"t2",t2,"t3",t3,"s",s_tube,"l",l,"Alpha1",Alpha1)
		Theta1=Alpha1-math.atan((t1+s_tube+t2)/l)
		Theta2=Alpha2-math.atan((t2+t3)/l)-math.atan((t1+s_tube+t2)/l)
		
		#Change Theat from radians to degrees
		Theta1 = Theta1*(360/(2*math.pi))
		Theta2 = Theta2*(360/(2*math.pi))
		
		
		#print("Theta1",Theta1)
		#print("Theta2",Theta2)
		
		#print("Alpha1 and Alpha2:", Alpha1, Alpha2)
		
		#Theta to DutyCycle for the square position
		DutyCycle1 = Theta1/Radians_per_DutyCycle
		DutyCycle2 = Theta2/Radians_per_DutyCycle
		
		#print("x and y:", x,y)
		#print("DutyCycle1 and DutyCycle2:", DutyCycle1, DutyCycle2)
				
		self.Theta1=Theta1
		self.Theta2=Theta2
		self.DutyCycle1=DutyCycle1
		self.DutyCycle2=DutyCycle2
			
		
	def DeterminePiece(self):
		diff=self.measured_P_ID-self.calibrated_P_ID
		
		zeroMargin = 0.0150
		PawnVal = 0.06
		RookVal = 0.1
		KnightVal =0.15
		BishopVal = 0.2
		QueenVal = 0.28
		
		if (diff < (-zeroMargin)):
			self.pieceColour="1"
			
		if (diff > zeroMargin):
			self.pieceColour="2"
			
		if (diff < zeroMargin) and (diff > (-zeroMargin)):
			self.pieceColour="."
			self.chessPiece="."
			
		diff=abs(diff)
		
		if (diff > zeroMargin) and (diff < PawnVal):
			if self.pieceColour == "1":
				self.chessPiece="p"
			if self.pieceColour == "2":
				self.chessPiece="P"
			
		if (diff > PawnVal) and (diff < RookVal):
			if self.pieceColour == "1":
				self.chessPiece="r"
			if self.pieceColour == "2":
				self.chessPiece="R"
			
		if (diff > RookVal) and (diff < KnightVal):
			if self.pieceColour == "1":
				self.chessPiece="n"
			if self.pieceColour == "2":
				self.chessPiece="N"
			
		if (diff > KnightVal) and (diff < BishopVal):
			if self.pieceColour == "1":
				self.chessPiece="b"
			if self.pieceColour == "2":
				self.chessPiece="B"
				
		if (diff > BishopVal) and (diff < QueenVal):
			if self.pieceColour == "1":
				self.chessPiece="q"
			if self.pieceColour == "2":
				self.chessPiece="Q"
				
		if (diff > QueenVal):
			if self.pieceColour == "1":
				self.chessPiece="k"
			if self.pieceColour == "2":
				self.chessPiece="K"



def MoveServo(Servo,Degree_new):
	
	Degree=Servo.prevDegree
	t=0.005
	increment=0.3
	q=Servo.number
	
	if(Degree_new-Degree)<0:
		increment=-0.3
	
	while(1):
		if abs(Degree_new - Degree) < abs(increment):
			kit.servo[q].angle = Degree_new
			break
	
		Degree = Degree + increment
		kit.servo[q].angle = Degree
		#Pi.set_PWM_dutycycle(Servo.pin, Degree)
		sleep(t)
		#print(Dutycycle)
		
		
	print("finish move servos:",Degree)
	
	Servo.prevDegree = Degree
		

def HomeLargeServo(Servo):
	
	Degree=Servo.prevDegree
	t=0.005
	increment=0.3
	q=Servo.number
	
	Degree_new = Servo.zero
	
	if(Degree_new-Degree)<0:
		increment=-0.3
	
	while(1):
	
		Degree = Degree + increment
		
		kit.servo[q].angle = Degree
		#Pi.set_PWM_dutycycle(Servo.pin, Degree)
		sleep(t)
		#print(Dutycycle)
		
		if abs(Degree_new - Degree) < abs(increment):
			break

	Servo.prevDegree = Degree	
		

def SetMUX_Address(Address):
	GPIO.output(MUX_S0_pin,Address[0])
	GPIO.output(MUX_S1_pin,Address[1])
	GPIO.output(MUX_S2_pin,Address[2])
	GPIO.output(MUX_S3_pin,Address[3])
	
def LowerLinearActuator():
	GPIO.output(LA_pin1, GPIO.LOW)
	GPIO.output(LA_pin2, GPIO.HIGH)
	sleep(4)


def LiftLinearActuator():
	GPIO.output(LA_pin1, GPIO.HIGH)
	GPIO.output(LA_pin2, GPIO.LOW)
	sleep(4)
	
def MoveMicroServo(Servo_micro, Pos_prev, Pos_new):
	
	Pos = Pos_prev
	t=0.01
	
	if (Pos_new - Pos_prev)>0:
		increment=0.1
	else:
		increment=-0.1
	
	while(1):
		if Pos!=Pos_new:
			Pos = Pos + increment
			Servo_micro.ChangeDutyCycle(Pos)
			sleep(t)
			
		if 	abs(Pos-Pos_new)<0.09:
			break
	
	print("Done",Pos_new)
	#Servo_micro.ChangeDutyCycle(0)


def GripPiece():
	
	kit.servo[2].angle = microServo_close
	
def ReleasePiece():
	
	kit.servo[2].angle = microServo_open
	
	
def DetermineMove(Chessboard, Chessboard_now, Chessboard_prev):
	
	Pos = []
	j=0
	
	# Determine positions involved in the chess move
	for i in range(len(Chessboard_now)):
		if Chessboard_now[i] != Chessboard_prev[i]:
			Pos.append(i)
			j=j+1	
			
	''' Normal move format: e2 e4. Capture format:  '''
	Move=""
	
	if j < 2:			#No move was made
		print("No move was made")
	
	else:			#A move was made
		
		# NORMAL MOVE AND CAPTURE
		if j == 2: 	# normal move or capture
		
				
			if Chessboard_now[Pos[0]] == ".":
				from_Pos = Pos[0]
				to_Pos = Pos[1]
		
			if Chessboard_now[Pos[1]] == ".":
				from_Pos = Pos[1]
				to_Pos = Pos[0]
		
			from_square = Chessboard[from_Pos].Position
			to_square = Chessboard[to_Pos].Position
			Move = Move + from_square + to_square
			print(Move)	
		
		# CASTLING 
		if j == 4: 	
			
			for t in Pos:
				if ((t+4)%8) == 0:
					from_Pos = t
					
			for r in Pos:
				if (((r+2)%8) == 0) or (((r+6)%8) == 0):
					to_Pos = r
			
					
			from_square = Chessboard[from_Pos].Position
			to_square = Chessboard[to_Pos].Position
			Move = Move + from_square + to_square
			print(Move)
		
		# EN PASSANT
		if j == 3: 	
			
			for g in Pos:
				if Chessboard_now[g] != ".":
					to_Pos = g
					
					for h in Pos:
						if (h == (to_Pos+9)) or (h == (to_Pos+7)):
							from_Pos = h
						
			from_square = Chessboard[from_Pos].Position
			to_square = Chessboard[to_Pos].Position
			Move = Move + from_square + to_square
			print(Move)
		
		
		
		# PAWN PROMOTION
			
			
			
		# Return the move as the correct object
		return chess.Move.from_uci(Move)
	
	
	
	
def VisualiseChessboard(Chessboard):
	# This function is used to visualise the chessboard in a way that is comparable with the board of Stockfish
	
	
	# Define the initial positions of the pieces
	i=1
	'''
	Rank8=[]
	Rank7=[]
	Rank6=[]
	Rank5=[]
	Rank4=[]
	Rank3=[]
	Rank2=[]
	Rank1=[]
	
	'''
	Rank8=""
	Rank7=""
	Rank6=""
	Rank5=""
	Rank4=""
	Rank3=""
	Rank2=""
	Rank1=""
	
	
	for square in Chessboard:
		if i <= 8:
			Rank8 = Rank8 + square.chessPiece
			if i%8 != 0:
				Rank8 = Rank8 + " "
		if i > 8 and i <= 16:
			Rank7 = Rank7 + square.chessPiece 
			if i%8 != 0:
				Rank7 = Rank7 + " "
		if i > 16 and i <= 24:
			Rank6 = Rank6 + square.chessPiece 
			if i%8 != 0:
				Rank6 = Rank6 + " "
		if i > 24 and i <= 32:
			Rank5 = Rank5 + square.chessPiece 
			if i%8 != 0:
				Rank5 = Rank5 + " "
		if i > 32 and i <= 40:
			Rank4 = Rank4 + square.chessPiece 
			if i%8 != 0:
				Rank4 = Rank4 + " "
		if i > 40 and i <= 48:
			Rank3 = Rank3 + square.chessPiece 
			if i%8 != 0:
				Rank3 = Rank3 + " "
		if i > 48 and i <= 56:
			Rank2 = Rank2 + square.chessPiece 
			if i%8 != 0:
				Rank2 = Rank2 + " "
		if i > 56 and i <= 64:
			Rank1 = Rank1 + square.chessPiece
			if i%8 != 0:
				Rank1 = Rank1 + " "
		i=i+1
			
	initial_pieces = [Rank8, Rank7, Rank6, Rank5, Rank4, Rank3, Rank2, Rank1] 

	# Initialize an empty list to hold the chessboard rows
	chessboard_rows = []

	# Create the chessboard string using a for loop
	for rank in initial_pieces:
		chessboard_rows.append(rank)

	# Join the rows into a single string
	VisualChessboard = "\n".join(chessboard_rows)
	
	print(VisualChessboard)

	return VisualChessboard
	
def Chessboard_Matrix(Chessboard):
	ChessboardMatrix=""
	
	for t in Chessboard:
		ChessboardMatrix = ChessboardMatrix + t.chessPiece
	
	return ChessboardMatrix
	
def Chessboard_Colour_Matrix(Chessboard):
	ChessboardColourMatrix=""
	
	for t in Chessboard:
		ChessboardColourMatrix = ChessboardColourMatrix + t.pieceColour
	
	return ChessboardColourMatrix

def Compare_Chessboards(my_Chessboard, Stockfish_Chessboard):
		
	if my_Chessboard == Stockfish_Chessboard:
		Chessboard_similar=1
	
	else:
		Chessboard_similar=0
		
	return Chessboard_similar

def TranslateMove(StockfishMove_string,my_Chessboard_prev,my_Chessboard_now):
	
	moveLength = len(StockfishMove_string)
	StockfishMove_Pos1 = StockfishMove_string[:2]
	StockfishMove_Pos2 = StockfishMove_string[2:4]
	
	Squares_1 = []
	Squares_2 = []
	
	r=0
	for t in Chessboard:
		if StockfishMove_Pos1 in t.Position:
			Squares_1.append(t)
			Squares_1.append(r)
		r=r+1
	r=0		
	for t in Chessboard:
		if StockfishMove_Pos2 in t.Position:
			Squares_2.append(t)
			Squares_2.append(r)	
		r=r+1
		
	return (Squares_1, Squares_2)
		
		
def MakeStockfishMove(StockfishMove_string,Squares_1,Squares_2,my_Colour_Chessboard_now,CaptureSquares,Number_of_captures, Stockfish_Chessboard_prev, Stockfish_Chessboard_now):
	j=0
	castle = 0
	
	if len(StockfishMove_string) == 4:		# all moves except pawn promotion
		
		#NORMAL MOVE OR CASTLING
		if my_Colour_Chessboard_now[Squares_2[1]] == ".":
			
			for i in range(len(Stockfish_Chessboard_now)):
				if Stockfish_Chessboard_now[i] != Stockfish_Chessboard_prev[i]:
					j=j+1
			
			# CASTLING
			if (j==4):	
				
				castle = 1
				
				# Move servos to first position
				Servo1_Degree = Servo1_Degree_Zero - Squares_1[0].Theta1
				Servo2_Degree = Servo2_Degree_Zero + Squares_1[0].Theta2
				
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Lift chess piece
				LowerLinearActuator()
				sleep(0.5)
				GripPiece()
				sleep(0.5)
				LiftLinearActuator()

				# MOve servos to second position
				Servo1_Degree = Servo1_Degree_Zero - Squares_2[0].Theta1
				Servo2_Degree = Servo2_Degree_Zero + Squares_2[0].Theta2
					
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Drop chess piece
				LowerLinearActuator()
				sleep(0.5)
				ReleasePiece()
				sleep(0.5)
				LiftLinearActuator()
				
				if (StockfishMove_string == 'e1g1'):
					# Move servos to first position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[63].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[63].Theta2
			
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Lift chess piece
					LowerLinearActuator()
					sleep(0.5)
					GripPiece()
					sleep(0.5)
					LiftLinearActuator()

					# MOve servos to second position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[61].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[61].Theta2
					
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Drop chess piece
					LowerLinearActuator()
					sleep(0.5)
					ReleasePiece()
					sleep(0.5)
					LiftLinearActuator()
						
				if (StockfishMove_string == 'e1c1'):
					# Move servos to first position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[56].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[56].Theta2
				
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Lift chess piece
					LowerLinearActuator()
					sleep(0.5)
					GripPiece()
					sleep(0.5)
					LiftLinearActuator()

					# MOve servos to second position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[59].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[59].Theta2
					
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Drop chess piece
					LowerLinearActuator()
					sleep(0.5)
					ReleasePiece()
					sleep(0.5)
					LiftLinearActuator()
						
						
				if (StockfishMove_string == 'e8g8'):
					# Move servos to first position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[7].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[7].Theta2
				
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Lift chess piece
					LowerLinearActuator()
					sleep(0.5)
					GripPiece()
					sleep(0.5)
					LiftLinearActuator()

					# MOve servos to second position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[5].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[5].Theta2
					
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Drop chess piece
					LowerLinearActuator()
					sleep(0.5)
					ReleasePiece()
					sleep(0.5)
					LiftLinearActuator()
						
						
				if (StockfishMove_string == 'e8c8'):
					# Move servos to first position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[0].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[0].Theta2
				
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Lift chess piece
					LowerLinearActuator()
					sleep(0.5)
					GripPiece()
					sleep(0.5)
					LiftLinearActuator()

					# MOve servos to second position
					Servo1_Degree = Servo1_Degree_Zero - Chessboard[3].Theta1
					Servo2_Degree = Servo2_Degree_Zero + Chessboard[3].Theta2
					
					MoveServo(Servo_One,Servo1_Degree)
					MoveServo(Servo_Two,Servo2_Degree)

					# Drop chess piece
					LowerLinearActuator()
					sleep(0.5)
					ReleasePiece()
					sleep(0.5)
					LiftLinearActuator()
				
				# Home servos
				HomeLargeServo(Servo_One)
				HomeLargeServo(Servo_Two)
			
			# EN PASSANT
			if (j == 3):
				
				Servo1_Degree = Servo1_Degree_Zero - Chessboard(Squares_2[1]-8).Theta1
				Servo2_Degree = Servo2_Degree_Zero + Chessboard(Squares_2[1]-8).Theta2
				
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Lift chess piece
				LowerLinearActuator()
				sleep(0.5)
				GripPiece()
				sleep(0.5)
				LiftLinearActuator()
				
				# Move servos to capture storage position			
				Servo1_Degree = Servo1_Degree_Zero - CaptureSquares[Number_of_captures].Theta1
				Servo2_Degree = Servo2_Degree_Zero + CaptureSquares[Number_of_captures].Theta2	
				Number_of_captures = Number_of_captures+1
					
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Drop chess piece
				LowerLinearActuator()
				sleep(0.5)
				ReleasePiece()
				sleep(0.5)
				LiftLinearActuator()
							
				Servo1_Degree = Servo1_Degree_Zero - Squares_1[0].Theta1
				Servo2_Degree = Servo2_Degree_Zero + Squares_1[0].Theta2
				
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Lift chess piece
				LowerLinearActuator()
				sleep(0.5)
				GripPiece()
				sleep(0.5)
				LiftLinearActuator()

				# MOve servos to second position
				Servo1_Degree = Servo1_Degree_Zero - Squares_2[0].Theta1
				Servo2_Degree = Servo2_Degree_Zero + Squares_2[0].Theta2
				
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Drop chess piece
				LowerLinearActuator()
				sleep(0.5)
				ReleasePiece()
				sleep(0.5)
				LiftLinearActuator()
				
				HomeLargeServo(Servo_One)
				HomeLargeServo(Servo_Two)
			
			# NORMAL MOVE
			if (j < 3) and (castle == 0):
				# Move servos to first position
				Servo1_Degree = Servo1_Degree_Zero - Squares_1[0].Theta1
				Servo2_Degree = Servo2_Degree_Zero + Squares_1[0].Theta2
				
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Lift chess piece
				LowerLinearActuator()
				sleep(0.5)
				GripPiece()
				sleep(0.5)
				LiftLinearActuator()

				# MOve servos to second position
				Servo1_Degree = Servo1_Degree_Zero - Squares_2[0].Theta1
				Servo2_Degree = Servo2_Degree_Zero + Squares_2[0].Theta2
				
				MoveServo(Servo_One,Servo1_Degree)
				MoveServo(Servo_Two,Servo2_Degree)

				# Drop chess piece
				LowerLinearActuator()
				sleep(0.5)
				ReleasePiece()
				sleep(0.5)
				LiftLinearActuator()
				
				# Home servos
				HomeLargeServo(Servo_One)
				HomeLargeServo(Servo_Two)
				
			
		# CAPTURE MOVE	
		if my_Colour_Chessboard_now[Squares_2[1]] != ".":
						
			# MOve servos to second position
			Servo1_Degree = Servo1_Degree_Zero - Squares_2[0].Theta1
			Servo2_Degree = Servo2_Degree_Zero + Squares_2[0].Theta2
			
			MoveServo(Servo_One,Servo1_Degree)
			MoveServo(Servo_Two,Servo2_Degree)
			
			# Lift chess piece
			LowerLinearActuator()
			sleep(0.5)
			GripPiece()
			sleep(0.5)
			LiftLinearActuator()
			
			# Move servos to capture storage position			
			Servo1_Degree = Servo1_Degree_Zero - CaptureSquares[Number_of_captures].Theta1
			Servo2_Degree = Servo2_Degree_Zero + CaptureSquares[Number_of_captures].Theta2	
			Number_of_captures = Number_of_captures+1
			
			MoveServo(Servo_One,Servo1_Degree)
			MoveServo(Servo_Two,Servo2_Degree)	
			
			# Drop chess piece
			LowerLinearActuator()
			sleep(0.5)
			ReleasePiece()
			sleep(0.5)
			LiftLinearActuator()	
			
			# Move servos to first position
			Servo1_Degree = Servo1_Degree_Zero - Squares_1[0].Theta1
			Servo2_Degree = Servo2_Degree_Zero + Squares_1[0].Theta2
				
			MoveServo(Servo_One,Servo1_Degree)
			MoveServo(Servo_Two,Servo2_Degree)

			# Lift chess piece
			LowerLinearActuator()
			sleep(0.5)
			GripPiece()
			sleep(0.5)
			LiftLinearActuator()

			# MOve servos to second position
			Servo1_Degree = Servo1_Degree_Zero - Squares_2[0].Theta1
			Servo2_Degree = Servo2_Degree_Zero + Squares_2[0].Theta2
				
			MoveServo(Servo_One,Servo1_Degree)
			MoveServo(Servo_Two,Servo2_Degree)

			# Drop chess piece
			LowerLinearActuator()
			sleep(0.5)
			ReleasePiece()
			sleep(0.5)
			LiftLinearActuator()

			# Home servos
			HomeLargeServo(Servo_One)
			HomeLargeServo(Servo_Two)
	
	# PAWN PROMOTION (tans nog net n gewone move)
		
	else:
		# Move servos to first position
		Servo1_Degree = Servo1_Degree_Zero - Squares_1[0].Theta1
		Servo2_Degree = Servo2_Degree_Zero + Squares_1[0].Theta2
		
		MoveServo(Servo_One,Servo1_Degree)
		MoveServo(Servo_Two,Servo2_Degree)

		# Lift chess piece
		LowerLinearActuator()
		sleep(0.5)
		GripPiece()
		sleep(0.5)
		LiftLinearActuator()

		# MOve servos to second position
		Servo1_Degree = Servo1_Degree_Zero - Squares_2[0].Theta1
		Servo2_Degree = Servo2_Degree_Zero + Squares_2[0].Theta2
		
		MoveServo(Servo_One,Servo1_Degree)
		MoveServo(Servo_Two,Servo2_Degree)

		# Drop chess piece
		LowerLinearActuator()
		sleep(0.5)
		ReleasePiece()
		sleep(0.5)
		LiftLinearActuator()
				
		# Home servos
		HomeLargeServo(Servo_One)
		HomeLargeServo(Servo_Two)
		
		message_pp = "Please promote the pawn to a " + StockfishMove_string[4]
		
		update_bottom_label(message_pp)
		window.update()
		
		
	return Number_of_captures
	
def update_top_label(Info_top):
	
    display_top_label.config(text=Info_top)
    
def update_bottom_label(Info_bottom):
	
    display_bottom_label.config(text=Info_bottom)
    
       
def update_matrix_display(left_canvas, right_canvas, Stockfish_matrix, my_matrix):
	
	Stockfish_string=""
	
	for r in Stockfish_matrix:
		if (r != " ") and (r != "\n"):
			Stockfish_string = Stockfish_string + r
			
	for row in range(8):
		for col in range(8):
			x1 = display_x_start + col * display_block_size
			y1 = display_y_start + row * display_block_size
			x2 = x1 + display_block_size
			y2 = y1 + display_block_size
            
			label1 = Stockfish_string[row*8+col]
            
			# Draw each block as a rectangle
			left_canvas.create_rectangle(x1, y1, x2, y2, fill="black", outline="white")
            
			# Place a letter (label) in the middle of each block
			left_canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text=label1, fill="white", font=("Helvetica", 18))
            
	for row in range(8):
		for col in range(8):
			x1 = display_x_start + col * display_block_size
			y1 = display_y_start + row * display_block_size
			x2 = x1 + display_block_size
			y2 = y1 + display_block_size
            
			label2 = my_matrix[row*8+col]
            
			# Draw each block as a rectangle
			right_canvas.create_rectangle(x1, y1, x2, y2, fill="black", outline="white")
            
			# Place a letter (label) in the middle of each block
			right_canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text=label2, fill="white", font=("Helvetica", 18))
    
	
#	--------------------------------------------------------------------
#		CHESS SQUARE INITIALISATION - OBJECTS AND POSITIONS ASSIGNING
#	--------------------------------------------------------------------

# DISPLAY: Initialising 

display_top_frame = tk.Frame(window)
display_top_frame.pack(side="top", fill="x", pady=10)

display_top_label = tk.Label(display_top_frame, text="Welcome", font=("Helvetica", 48))
display_top_label.pack()

# Create a frame for the matrices below the top text
display_matrix_frame = tk.Frame(window)
display_matrix_frame.pack(expand=True, fill="both")

# Create sub-frames for the left and right matrices
display_left_matrix_frame = tk.Frame(display_matrix_frame)
display_left_matrix_frame.pack(side="left", expand=True)

display_right_matrix_frame = tk.Frame(display_matrix_frame)
display_right_matrix_frame.pack(side="right", expand=True)

# Add labels for each matrix
display_left_matrix_label = tk.Label(display_left_matrix_frame, text="Stockfish chessboard", font=("Helvetica", 18))
display_left_matrix_label.pack(pady=5)

display_right_matrix_label = tk.Label(display_right_matrix_frame, text="Automated chessboard", font=("Helvetica", 18))
display_right_matrix_label.pack(pady=5)

# Create canvases for the left and right matrices
display_left_canvas = tk.Canvas(display_left_matrix_frame, width=390, height=340, bg="white")
display_left_canvas.pack()

display_right_canvas = tk.Canvas(display_right_matrix_frame, width=390, height=340, bg="white")
display_right_canvas.pack()


display_bottom_frame = tk.Frame(window)
display_bottom_frame.pack(side="bottom", fill="x", pady=10)

# Add a label at the bottom of the screen
display_bottom_label = tk.Label(display_bottom_frame, text="The automated chessboard is initialising...", font=("Helvetica", 24))
display_bottom_label.pack()

window.update() 

# update_display(Info_top, Chessboard_display, Info_bottom, font_size)

# Creating chess square objects
SquareA1 = ChessSquare(3, 3, 'a1')
SquareA2 = ChessSquare(3, 4, 'a2')
SquareA3 = ChessSquare(3, 6, 'a3')
SquareA4 = ChessSquare(3, 7, 'a4')
SquareA5 = ChessSquare(2, 0, 'a5')
SquareA6 = ChessSquare(2, 3, 'a6')
SquareA7 = ChessSquare(2, 4, 'a7')
SquareA8 = ChessSquare(2, 5, 'a8')

SquareB1 = ChessSquare(3, 2, 'b1')
SquareB2 = ChessSquare(3, 5, 'b2')
SquareB3 = ChessSquare(3, 8, 'b3')
SquareB4 = ChessSquare(3, 9, 'b4')
SquareB5 = ChessSquare(2, 1, 'b5')
SquareB6 = ChessSquare(2, 2, 'b6')
SquareB7 = ChessSquare(2, 6, 'b7')
SquareB8 = ChessSquare(2, 7, 'b8')

SquareC1 = ChessSquare(3, 0, 'c1')
SquareC2 = ChessSquare(3, 14, 'c2')
SquareC3 = ChessSquare(3, 11, 'c3')
SquareC4 = ChessSquare(3, 10, 'c4')
SquareC5 = ChessSquare(2, 14, 'c5')
SquareC6 = ChessSquare(2, 13, 'c6')
SquareC7 = ChessSquare(2, 9, 'c7')
SquareC8 = ChessSquare(2, 8, 'c8')

SquareD1 = ChessSquare(3, 1, 'd1')
SquareD2 = ChessSquare(3, 15, 'd2')
SquareD3 = ChessSquare(3, 13, 'd3')
SquareD4 = ChessSquare(3, 12, 'd4')
SquareD5 = ChessSquare(2, 15, 'd5')
SquareD6 = ChessSquare(2, 12, 'd6')
SquareD7 = ChessSquare(2, 11, 'd7')
SquareD8 = ChessSquare(2, 10, 'd8')

SquareE1 = ChessSquare(1, 3, 'e1')
SquareE2 = ChessSquare(1, 4, 'e2')
SquareE3 = ChessSquare(1, 6, 'e3')
SquareE4 = ChessSquare(1, 7, 'e4')
SquareE5 = ChessSquare(0, 0, 'e5')
SquareE6 = ChessSquare(0, 3, 'e6')
SquareE7 = ChessSquare(0, 4, 'e7')
SquareE8 = ChessSquare(0, 5, 'e8')

SquareF1 = ChessSquare(1, 2, 'f1')
SquareF2 = ChessSquare(1, 5, 'f2')
SquareF3 = ChessSquare(1, 8, 'f3')
SquareF4 = ChessSquare(1, 9, 'f4')
SquareF5 = ChessSquare(0, 1, 'f5')
SquareF6 = ChessSquare(0, 2, 'f6')
SquareF7 = ChessSquare(0, 6, 'f7')
SquareF8 = ChessSquare(0, 7, 'f8')

SquareG1 = ChessSquare(1, 0, 'g1')
SquareG2 = ChessSquare(1, 11, 'g2')
SquareG3 = ChessSquare(1, 15, 'g3')
SquareG4 = ChessSquare(1, 14, 'g4')
SquareG5 = ChessSquare(0, 14, 'g5')
SquareG6 = ChessSquare(0, 13, 'g6')
SquareG7 = ChessSquare(0, 9, 'g7')
SquareG8 = ChessSquare(0, 8, 'g8')

SquareH1 = ChessSquare(1, 1, 'h1')
SquareH2 = ChessSquare(1, 10, 'h2')
SquareH3 = ChessSquare(1, 12, 'h3')
SquareH4 = ChessSquare(1, 13, 'h4')
SquareH5 = ChessSquare(0, 15, 'h5')
SquareH6 = ChessSquare(0, 12, 'h6')
SquareH7 = ChessSquare(0, 11, 'h7')
SquareH8 = ChessSquare(0, 10, 'h8')

# Create chessboard matrix and name matrix
Chessboard = [SquareA8, SquareB8, SquareC8, SquareD8, SquareE8, SquareF8, SquareG8, SquareH8,
SquareA7, SquareB7, SquareC7, SquareD7, SquareE7, SquareF7, SquareG7, SquareH7,
SquareA6, SquareB6, SquareC6, SquareD6, SquareE6, SquareF6, SquareG6, SquareH6,
SquareA5, SquareB5, SquareC5, SquareD5, SquareE5, SquareF5, SquareG5, SquareH5,
SquareA4, SquareB4, SquareC4, SquareD4, SquareE4, SquareF4, SquareG4, SquareH4,
SquareA3, SquareB3, SquareC3, SquareD3, SquareE3, SquareF3, SquareG3, SquareH3,
SquareA2, SquareB2, SquareC2, SquareD2, SquareE2, SquareF2, SquareG2, SquareH2,
SquareA1, SquareB1, SquareC1, SquareD1, SquareE1, SquareF1, SquareG1, SquareH1]


Chessboard_Name = [
'A8', 'B8', 'C8', 'D8', 'E8', 'F8', 'G8', 'H8',
'A7', 'B7', 'C7', 'D7', 'E7', 'F7', 'G7', 'H7',
'A6', 'B6', 'C6', 'D6', 'E6', 'F6', 'G6', 'H6',
'A5', 'B5', 'C5', 'D5', 'E5', 'F5', 'G5', 'H5',
'A4', 'B4', 'C4', 'D4', 'E4', 'F4', 'G4', 'H4',
'A3', 'B3', 'C3', 'D3', 'E3', 'F3', 'G3', 'H3',
'A2', 'B2', 'C2', 'D2', 'E2', 'F2', 'G2', 'H2',
'A1', 'B1', 'C1', 'D1', 'E1', 'F1', 'G1', 'H1']


# Cartesian coordinates for each chess square
file_positions = []
rank_positions = []

# Setting Chessboard locations: Starting at a and 1
for r in range(8):
	file_pos = 3.5*SquareSize - (r)*SquareSize
	file_positions.append(file_pos)
	rank_pos = (r)*SquareSize + RobotArm_Offset
	rank_positions.append(rank_pos)
	
print(file_positions,rank_positions)
# Given each chessbpard square object their location and then solve the linear algebra
for q in range(8):
	for s in range(8):
		Chessboard[(q)*8+s].x_pos = file_positions[s]
		Chessboard[(q)*8+s].y_pos = rank_positions[q]
		Chessboard[(q)*8+s].DeterminePosition()
		#print(Chessboard[(q)*8+s].Position,Chessboard[(q)*8+s].x_pos, Chessboard[(q)*8+s].y_pos)


'''Large Servo objects'''
Servo_One = Servo(Servo1, Servo1_Degree_Zero, Servo1_Degree_Zero,0)
Servo_Two = Servo(Servo2, Servo2_Degree_Zero, Servo2_Degree_Zero,1)


'''
#Setup PWM channels for hardware PWM
Pi.set_mode(Servo_One.pin, pigpio.OUTPUT)
Pi.set_mode(Servo_Two.pin, pigpio.OUTPUT)
#Start the PWM channels artt dutycycle of 0
Pi.set_PWM_frequency(Servo_One.pin, Servo12PWM_f)     
Pi.set_PWM_dutycycle(Servo_One.pin, 0)     			
Pi.set_PWM_frequency(Servo_Two.pin, Servo12PWM_f)    
Pi.set_PWM_dutycycle(Servo_Two.pin, 0) 
'''

'''Linear Actuator setup'''
GPIO.setup(LA_pin1,GPIO.OUT)
GPIO.setup(LA_pin2,GPIO.OUT)

GPIO.output(LA_pin1, GPIO.LOW)
GPIO.output(LA_pin2, GPIO.LOW)


'''Button setup'''
GPIO.setup(ButtonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

'''LED setup'''
GPIO.setup(LED_R,GPIO.OUT)
GPIO.setup(LED_G,GPIO.OUT)
GPIO.setup(LED_B,GPIO.OUT)

GPIO.output(LED_R, GPIO.LOW)
GPIO.output(LED_G, GPIO.LOW)
GPIO.output(LED_B, GPIO.LOW)

# Setting all MUX pins low at start
GPIO.output(MUX_S0_pin, GPIO.LOW)
GPIO.output(MUX_S1_pin, GPIO.LOW)
GPIO.output(MUX_S2_pin, GPIO.LOW)
GPIO.output(MUX_S3_pin, GPIO.LOW)

#Capture positions

Number_of_captures = 0

Capture_1 = ChessSquare(0, 0, 'i8')
Capture_2 = ChessSquare(0, 0, 'j8')
Capture_3 = ChessSquare(0, 0, 'k8')
Capture_4 = ChessSquare(0, 0, 'i7')
Capture_5 = ChessSquare(0, 0, 'j7')
Capture_6 = ChessSquare(0, 0, 'k7')
Capture_7 = ChessSquare(0, 0, 'i6')
Capture_8 = ChessSquare(0, 0, 'j6')
Capture_9 = ChessSquare(0, 0, 'k6')
Capture_10 = ChessSquare(0, 0, 'i5')
Capture_11 = ChessSquare(0, 0, 'j5')
Capture_12 = ChessSquare(0, 0, 'k5')
Capture_13 = ChessSquare(0, 0, 'i4')
Capture_14 = ChessSquare(0, 0, 'j4')
Capture_15 = ChessSquare(0, 0, 'k4')
Capture_16 = ChessSquare(0, 0, 'i3')
Capture_17 = ChessSquare(0, 0, 'j3')
Capture_18 = ChessSquare(0, 0, 'k3')

CaptureSquares = [Capture_1,Capture_2,Capture_3,Capture_4,Capture_5,Capture_6,Capture_7,Capture_8,
Capture_9,Capture_10,Capture_11,Capture_12,Capture_13,Capture_14,Capture_15,Capture_16,Capture_17,Capture_18]

# Cartesian coordinates for each chess square
capture_file_positions = [-0.27, -0.33, -0.39]
capture_rank_positions = [RobotArm_Offset, (RobotArm_Offset+0.06), (RobotArm_Offset+0.12), (RobotArm_Offset+0.18), (RobotArm_Offset+0.24),(RobotArm_Offset+0.30)]

# Given each chessbpard square object their location and then solve the linear algebra
for q in range(6):
	for s in range(3):
		CaptureSquares[(q)*3+s].x_pos = capture_file_positions[s]
		#print(CaptureSquares[(q)*3+s].x_pos)
		CaptureSquares[(q)*3+s].y_pos = capture_rank_positions[q]
		#print(CaptureSquares[(q)*3+s].y_pos)
		CaptureSquares[(q)*3+s].DeterminePosition()
		#print((q)*3+s)
		

'''
-----------------------------------------------------------------------------------
								START THE PROGRAM
-----------------------------------------------------------------------------------
'''
# display update
update_top_label("Welcome")
update_bottom_label("... Homing ...")
window.update() 

#Home all the actuators

kit.servo[0].angle = Servo1_Degree_Zero
kit.servo[1].angle = Servo2_Degree_Zero
kit.servo[2].angle = microServo_open

'''Lift the linear actuator'''
LiftLinearActuator()


# Create chessboard matrices to use when determining the chess move. 
# The curretn chessboard configuration and previous chessboard configuration.
'''
The chessboard convension is a string starting at top left(a8) and moving in files and then ranks to the bottom right. 
The first element in square a8, then there is a space between all squares .
'''
Stockfish_Chessboard_now = str(Chessboard_board )
Stockfish_Chessboard_prev = str(Chessboard_board )

my_Chessboard_Stockfish = VisualiseChessboard(Chessboard)
my_Chessboard_now = Chessboard_Matrix(Chessboard)
my_Chessboard_prev = Chessboard_Matrix(Chessboard)

#Start by calibrating all the sensors: This is done before the pieces are placed on the chessboard.

# display update
update_top_label("Calibration")
update_bottom_label("Do not place any chess pieces on the chessbpard yet.")
window.update() 


while(1):
	
	#print("Calibrating the chessbpard now. Do not place any chess pieces on the chessbpard yet.")
	print("Start reading the chess pieces on the board:",datetime.now())
	for x in Chessboard:
		SetMUX_Address(x.MUX_pin)
		sleep(0.000142)
		MeasureCal=AnalogIn(ads,x.ADC_channel)
		#sleep(0.01)
		x.calibrated_P_ID=MeasureCal.voltage
		#print(x.calibrated_P_ID)
	print("Finish reading the chess pieces on the board:",datetime.now())
	# display update
	
	update_top_label("Calibration finished.")
	update_bottom_label("Populate the chessboard with the chess pieces in their starting positions \nand press the user button when finished.")
	window.update() 
	
	while True:
		# Check if the button is pressed (active low)
		if GPIO.input(ButtonPin) == GPIO.LOW:
			#print("Button pressed!")
			break  # Exit the loop when the button is pressed
        
        # Small delay to prevent high CPU usage
		sleep(0.05)
	
	for y in Chessboard:
		SetMUX_Address(y.MUX_pin)
		sleep(0.000142)
		MeasureCal=AnalogIn(ads,y.ADC_channel)
		#sleep(0.001)
		y.measured_P_ID=MeasureCal.voltage
		#print(Chessboard_Name[j])
		#print(y.measured_P_ID)
		y.DeterminePiece()
		
	my_Chessboard_Stockfish = VisualiseChessboard(Chessboard)
	my_Chessboard_now = Chessboard_Matrix(Chessboard)
	my_Chessboard_prev = Chessboard_Matrix(Chessboard)
	
	Chessboard_similar = Compare_Chessboards(my_Chessboard_Stockfish, Stockfish_Chessboard_now)
	#Chessboard_similar = Compare_Chessboards(Stockfish_Chessboard, Stockfish_Chessboard)
	
	#display update
	update_top_label("Checking positions of the chess pieces.")
	update_bottom_label("")
	window.update() 
		
	sleep(2)

	if Chessboard_similar == 1:
		#display update
		
		update_top_label("Ready.")
		update_bottom_label("The starting positions are correct. The game will start now")
		update_matrix_display(display_left_canvas, display_right_canvas, Stockfish_Chessboard_now, my_Chessboard_now)
		window.update() 
		
		#print("The starting positions are correct. The game will start now")
		break
	else:
		#display update
		update_top_label("Not yet ready...")
		update_bottom_label("The starting positions seem incorrect. \nPlease check and press the user button when everything is correct.")
		update_matrix_display(display_left_canvas, display_right_canvas, Stockfish_Chessboard_now, my_Chessboard_now)
		window.update() 
		
		#print("The starting positions seem incorrect. Please check and press the user button when everything is correct.")
		while True:
			# Check if the button is pressed (active low)
			if GPIO.input(ButtonPin) == GPIO.LOW:
				#print("Button pressed!")
				break  # Exit the loop when the button is pressed
        
			# Small delay to prevent high CPU usage
			sleep(0.05)
	break

sleep(2)

for y in Chessboard:
	SetMUX_Address(y.MUX_pin)
	sleep(0.001)
	MeasureCal=AnalogIn(ads,y.ADC_channel)
	sleep(0.001)
	y.measured_P_ID=MeasureCal.voltage
	#print(Chessboard_Name[j])
	#print(y.measured_P_ID)
	y.DeterminePiece()
	
	my_Chessboard_prev = Chessboard_Matrix(Chessboard)
	my_Chessboard_now = Chessboard_Matrix(Chessboard)
		
	my_Colour_Chessboard_prev = Chessboard_Colour_Matrix(Chessboard)
	my_Colour_Chessboard_now = Chessboard_Colour_Matrix(Chessboard)
		
	my_Chessboard_Stockfish=VisualiseChessboard(Chessboard)

Checkmate = 0
UserMove_flag = 1
ChessboardMove_flag = 0



''' --------------------------------------------------------------------------------------
							START WHILE LOOP FOR THE CHESS GAME
---------------------------------------------------------------------------------------'''
	
while(1):

	if (UserMove_flag ==1 ) and (Checkmate == 0):
		
		# Checks all positions
		j=0
		for y in Chessboard:
			SetMUX_Address(y.MUX_pin)
			sleep(0.000142)
			MeasureCal=AnalogIn(ads,y.ADC_channel)
			y.measured_P_ID=MeasureCal.voltage
			#print(Chessboard_Name[j])
			#print(y.measured_P_ID)
			y.DeterminePiece()
			j=j+1
				
		my_Chessboard_prev = my_Chessboard_now
		my_Chessboard_now = Chessboard_Matrix(Chessboard)
		
		my_Colour_Chessboard_prev = my_Colour_Chessboard_now
		my_Colour_Chessboard_now = Chessboard_Colour_Matrix(Chessboard)
		
		my_Chessboard_Stockfish=VisualiseChessboard(Chessboard)
		
		#display update
		update_top_label("User move")
		update_bottom_label("Make your move and press the button when you are finished.")
		update_matrix_display(display_left_canvas, display_right_canvas, Stockfish_Chessboard_now, my_Chessboard_now)
		sleep(0.1)
		window.update() 
		
		GPIO.output(LED_G, GPIO.HIGH)
		GPIO.output(LED_R, GPIO.LOW)
		GPIO.output(LED_B, GPIO.LOW)
			
		while True:
			# Check if the button is pressed (active low)
			if GPIO.input(ButtonPin) == GPIO.LOW:
				#print("Button pressed!")
				break  # Exit the loop when the button is pressed
        
			# Small delay to prevent high CPU usage
			sleep(0.05)
		
		'''
		# Request move from user
		print("Make your move and press the button when you are finished.")
		input()
		'''
		#Check button press from user

		# Checks all positions
		j=0
		for y in Chessboard:
			SetMUX_Address(y.MUX_pin)
			sleep(0.001)
			MeasureCal=AnalogIn(ads,y.ADC_channel)
			sleep(0.001)
			y.measured_P_ID=MeasureCal.voltage
			#print(Chessboard_Name[j])
			#print(y.measured_P_ID)
			y.DeterminePiece()
			j=j+1
			
		
		
		my_Chessboard_prev = my_Chessboard_now
		my_Chessboard_now = Chessboard_Matrix(Chessboard)
		
		my_Colour_Chessboard_prev = my_Colour_Chessboard_now
		my_Colour_Chessboard_now = Chessboard_Colour_Matrix(Chessboard)
		
		my_Chessboard_Stockfish=VisualiseChessboard(Chessboard)
		sleep(0.2)
		
		# Determine chess move

		UserMove = DetermineMove(Chessboard, my_Colour_Chessboard_now, my_Colour_Chessboard_prev)		# And make move a move object
		
	
			
		# Check if the move is legal with Stockfish	and push move to Stockfish if it is legal
	
		if Chessboard_board.is_legal(UserMove):
			Chessboard_board.push(UserMove)
			UserMove_flag = 0
			ChessboardMove_flag = 1
			
			
		else:
			
			#display update
			update_top_label("User move: Illegal move.")
			update_bottom_label("Undo the move and press the button.")
			#update_matrix_display(display_left_canvas, display_right_canvas, Stockfish_Chessboard_now, my_Chessboard_now)
			window.update() 
			
			
			GPIO.output(LED_R, GPIO.HIGH)
			GPIO.output(LED_G, GPIO.LOW)
			GPIO.output(LED_B, GPIO.LOW)
				
			UserMove_flag = 1
			ChessboardMove_flag = 0
			
			while True:
				# Check if the button is pressed (active low)
				if GPIO.input(ButtonPin) == GPIO.LOW:
								
					break  # Exit the loop when the button is pressed
        
				# Small delay to prevent high CPU usage
				sleep(0.05)
			
			
			#print("Follow further instructions as per usual")
		
		Stockfish_Chessboard_prev = Stockfish_Chessboard_now
		Stockfish_Chessboard_now = str(Chessboard_board)
			
		if Chessboard_board.is_checkmate():
			Checkmate = 1
	
		
	if (ChessboardMove_flag == 1) and (Checkmate == 0):
		
		GPIO.output(LED_B, GPIO.HIGH)
		GPIO.output(LED_R, GPIO.LOW)
		GPIO.output(LED_G, GPIO.LOW)
		
		#display update
		update_top_label("Autonomous chessboard move.")
		update_bottom_label("A move is being made by the chessboard.")
		update_matrix_display(display_left_canvas, display_right_canvas, Stockfish_Chessboard_now, my_Chessboard_now)
		window.update() 
				
		# Stockfish generates a move
		result = Stockfish.play(Chessboard_board, chess.engine.Limit(time=1.0))
		Chessboard_board.push(result.move)
		
			
		Stockfish_Chessboard_prev = Stockfish_Chessboard_now
		Stockfish_Chessboard_now = str(Chessboard_board)
			
		# Idenitfy the squares involved in the chess move
		StockfishMove = result.move
		print(StockfishMove)
		StockfishMove_string = StockfishMove.uci()
		StockfishMove_Pos1 = StockfishMove_string[:2]
		StockfishMove_Pos2 = StockfishMove_string[2:4]
		
		Squares_1, Squares_2 = TranslateMove(StockfishMove_string,my_Chessboard_prev,my_Chessboard_now)
		Number_of_captures = MakeStockfishMove(StockfishMove_string,Squares_1,Squares_2,my_Colour_Chessboard_now,CaptureSquares, Number_of_captures, Stockfish_Chessboard_prev, Stockfish_Chessboard_now)
		
		for y in Chessboard:
			SetMUX_Address(y.MUX_pin)
			sleep(0.001)
			MeasureCal=AnalogIn(ads,y.ADC_channel)
			sleep(0.001)
			y.measured_P_ID=MeasureCal.voltage
			#print(Chessboard_Name[j])
			#print(y.measured_P_ID)
			y.DeterminePiece()
		
		
		UserMove_flag = 1
		ChessboardMove_flag = 0
		
		if Chessboard_board.is_checkmate():
			Checkmate = 1
			
	if Checkmate == 1:
		
		#display update
		update_top_label("Game over")
		update_bottom_label("...")
		update_matrix_display(display_left_canvas, display_right_canvas, Stockfish_Chessboard_now, my_Chessboard_now)
		window.update() 
		
		for h in range(10):
	
			GPIO.output(LED_B, GPIO.HIGH)
			GPIO.output(LED_R, GPIO.LOW)
			GPIO.output(LED_G, GPIO.LOW)
	
			sleep(0.1)
	
			GPIO.output(LED_R, GPIO.HIGH)
			GPIO.output(LED_G, GPIO.LOW)
			GPIO.output(LED_B, GPIO.LOW)

			sleep(0.1)
	
			GPIO.output(LED_G, GPIO.HIGH)
			GPIO.output(LED_B, GPIO.LOW)
			GPIO.output(LED_R, GPIO.LOW)
	
			sleep(0.1)
			
		GPIO.output(LED_G, GPIO.HIGH)
		GPIO.output(LED_B, GPIO.HIGH)
		GPIO.output(LED_R, GPIO.HIGH)
		
		'''
		
		update_display("", "Press the button to restart the game", "", Heading_font_size)
			
		#print("Press the button to restart the game")	
		while True:
			# Check if the button is pressed (active low)
			if GPIO.input(ButtonPin) == GPIO.LOW:
				#print("Button pressed!")
				break  # Exit the loop when the button is pressed
        
			# Small delay to prevent high CPU usage
			sleep(0.05)
		
		update_display("", "Place the chess pieces in the correct starting positions and press the button.", "", Heading_font_size)
		#print("Place the chess pieces in the correct starting positions and press the button.")

		while True:
			# Check if the button is pressed (active low)
			if GPIO.input(ButtonPin) == GPIO.LOW:
				#print("Button pressed!")
				break  # Exit the loop when the button is pressed
        
			# Small delay to prevent high CPU usage
			sleep(0.05)

		my_Chessboard_Stockfish = VisualiseChessboard(Chessboard)
		print(Stockfish_Chessboard_now)
	
		Chessboard_similar = Compare_Chessboards(my_Chessboard_Stockfish, Stockfish_Chessboard_now)
	
		sleep(0.2)

		if Chessboard_similar == 1:
			#display update
			update_display("The starting positions are correct. The game will start now", my_Chessboard_Stockfish, "", Chessboard_font_size)

			break
		else:
			#display update
			update_display("The starting positions seem incorrect. Please check and press the user button when everything is correct.", my_Chessboard_Stockfish, "", Chessboard_font_size)

			
			#print("The starting positions seem incorrect. Please check and press the user button when everything is correct.")
			while True:
				# Check if the button is pressed (active low)
				if GPIO.input(ButtonPin) == GPIO.LOW:
					#print("Button pressed!")
					break  # Exit the loop when the button is pressed
        
				# Small delay to prevent high CPU usage
				sleep(0.05)
		
		for y in Chessboard:
			SetMUX_Address(y.MUX_pin)
			sleep(0.001)
			MeasureCal=AnalogIn(ads,y.ADC_channel)
			sleep(0.001)
			y.measured_P_ID=MeasureCal.voltage
			#print(Chessboard_Name[j])
			#print(y.measured_P_ID)
			y.DeterminePiece()

		my_Chessboard_now = Chessboard_Matrix(Chessboard)
		my_Chessboard_prev = Chessboard_Matrix(Chessboard)

		my_Colour_Chessboard_now = Chessboard_Colour_Matrix(Chessboard)
		my_Colour_Chessboard_prev = Chessboard_Colour_Matrix(Chessboard)

		Checkmate = 0
		UserMove_flag = 1
		ChessboardMove_flag = 0

		sleep(2)
		'''
			
	
'''
-----------------------------------------------------------------------------------
								END THE GAME
-----------------------------------------------------------------------------------
'''

# Close the engine
Stockfish.quit()









