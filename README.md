In my final year project for Mechatronics Engineering at Stellenbosch Univeristy, I designed and built an automated chessboard. The title of the project is: The design and implementation of autonomous movement and [iece identification in an automated chessboard. 

The autonomous movement was achieved with a two-link robotic arm above the chessboard. The robotic arm was controlled with two stepper motors. On the end of the arm is a mechanical gripper that also uses a small servo to grip the chess pieces. 

The piece identification system consists of 64 hall effect sensors, one under each square and magnets placed in the bottom of the chess pieces. Magnets of varying strengths are used to create 6 discrete identities (when the polarities of the magnets are flipped, 6 more discrete identities are created). 

In the project the chess engine, Stockfish is downloaded and used to generate the next best moves. Stockfish is an open source chess engine publicly available. Stockfish can be downloaded from the website Stockfish. (https://stockfishchess.org/download/)

The following libraries are required for the project and can be downloaded on the below websites:

adafruit_ads1x15 (https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15)
adafruit_servokit (https://github.com/adafruit/Adafruit_CircuitPython_ServoKit/releases/tag/1.3.18)
pigpio (https://pypi.org/project/pigpio/#files)
RPi.GPIO (https://pypi.org/project/RPi.GPIO/#files)
python chess (https://pypi.org/project/python-chess/#files)
