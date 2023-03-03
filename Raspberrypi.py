'''
Author: Miles Popiela
Contact: popielamc@vcu.edu

Description: Creating LED visualization of 3D Point rotation
matrix with rotation input being rotational and Y-Axis values
of a game object within unity using UDP sockets
'''

from math import *
import board
import time
import neopixel
import socket
import math

from threading import Thread
from multiprocessing import Pipe

class LED_Strip(Thread):
    Running = True
    def __init__(self, Name, Child, pixel_pin, num_pixels):
        super().__init__()
        self.Name = Name
        self.Child = Child
        self.pixel_pin = pixel_pin
        self.num_pixels = num_pixels

        self.pixels = neopixel.NeoPixel(
            self.pixel_pin, self.num_pixels, brightness=100, auto_write=False, pixel_order=neopixel.GRB
        )

        self.pixels.fill((0, 255, 0))
        self.pixels.show()

        # Setting points
        self.cube_points = [n for n in range(self.num_pixels)]

        self.projection_matrix = [[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 0]]
    
    def Setup_Virtual_Points(self, input): #Decide how to deal with range
        y, Frontside, Leftside, Backside, Rightside = input
        # Front side
        for x in range(Frontside[0], Frontside[1]):
            if y == -1:
                y = y + self.num_pixels
            self.cube_points[y] = [[x * ((1 / (self.num_pixels/4)) * 2)], [0], [1]]
            y = y - 1

        # Left Side
        y = 15
        for x in range(Leftside[0], Leftside[1]):
            self.cube_points[y] = [[-1], [0], [x * -((1 / (self.num_pixels/4)) * 2)]]
            y = y + 1

        #Backside
        for x in range(Backside[0], Backside[1]):
            self.cube_points[y] = [[x * ((1 / (self.num_pixels/4)) * 2)], [0], [-1]]
            y = y + 1

        #Rightside
        for x in range(Rightside[0], Rightside[1]):
            self.cube_points[y] = [[1], [0], [x * ((1 / (self.num_pixels/4)) * 2)]]
            y = y + 1
    
    def RotationOfPoints(self, input):
        angle_x, angle_y, angle_z, Uy = input
        rotation_x = [[1, 0, 0],
                      [0, cos(angle_x), -sin(angle_x)],
                      [0, sin(angle_x), cos(angle_x)]]

        rotation_y = [[cos(angle_y), 0, sin(angle_y)],
                      [0, 1, 0],
                      [-sin(angle_y), 0, cos(angle_y)]]

        rotation_z = [[cos(angle_z), -sin(angle_z), 0],
                      [sin(angle_z), cos(angle_z), 0],
                      [0, 0, 1]]
        
        i = 0
        for point in self.cube_points:

            rotate_x = multiply_m(rotation_x, point)
            rotate_y = multiply_m(rotation_y, rotate_x)
            rotate_z = multiply_m(rotation_z, rotate_y)
            point_2d = multiply_m(self.projection_matrix, rotate_z)

            '''
            Takes virtual plot points and combineds them with

            must think of method of simplification ~ ** The robots' Y-axis ** ~ must think of method of simplification

            '''
            #Local_y = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
            #Color = Local_y 

            Color = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
            if Color > 255:
                Color = 255
            if Color < 0:
                Color = 0

            self.pixels[i] = (255 - int(Color), 70, int(Color))
            i += 1
    
    def run(self):
        self.Setup_Virtual_Points(self.Child.recv())
        self.Child.send(True)
        
        while self.Running:
            self.RotationOfPoints(self.Child.recv())
            self.Child.send(True)

        super().run()



# Matrix multiplication function
def multiply_m(a, b):
    a_rows = len(a)
    a_cols = len(a[0])
    b_rows = len(b)
    b_cols = len(b[0])
    # Dot product matrix dimensions = a_rows x b_cols
    product = [[0 for _ in range(b_cols)] for _ in range(a_rows)]

    for i in range(a_rows):
        for j in range(b_cols):
            for k in range(b_rows):
                product[i][j] += a[i][k] * b[k][j]

    return product

if __name__ == "__main__":
    '''
    Setting Up UDP Server
    '''
    IP = "192.168.0.50"
    PORT =  8000

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, PORT))
    print("Binded")


    '''
    Setting up NeoPixels
    '''
    Pipes = [Pipe() for k in range (2)] # in, out = Pipe
    Parent = [k[0] for k in Pipes]
    #Front, Bottom, Inner, Top
    #Name, Pin, Number of LEDs
    LED_Strips = [  LED_Strip('Top',    Pipes[0][1], board.D18, 104), #pin 12 GPIO 18
                    LED_Strip('Front',  Pipes[1][1], board.D21, 120)] #pin 40 GPIO 21
                    #LED_Strip('Inner', Pipes[2][1], board.D24, 120)]#,
                    #LED_Strip('Bottom',Pipes[3][1], board.D27, 120)]
    
    threads= []
    #y, Frontside, Leftside, Backside, Rightside
    #LED_Strips[0].Setup_Virtual_Points(15, [-13,14], [-13,13], [-13,13], [-13,13])
    #LED_Strips[1].Setup_Virtual_Points(15, [-15,16], [-15,15], [-15,15], [-15,15])
    #LED_Strips[2].Setup_Virtual_Points(15, [-15,16], [-15,15], [-15,15], [-15,15])
    #LED_Strips[3].Setup_Virtual_Points(15, [-15,16], [-15,15], [-15,15], [-15,15])
    Points = []
    Points.append([15, [-13,14], [-13,13], [-13,13], [-13,13]])
    Points.append([15, [-15,16], [-15,15], [-15,15], [-15,15]])
    for i in range(len(LED_Strips)):
        LED_Strips[i].start()
        Parent[i].send(Points[i])
    
    for p in Parent:
        if p.recv:
            print('Points Setup Succesful')

    '''
    Setting Up Virtual Points

    TODO!!!
    We need to make two more cube_points for inner,top, and bottom faces of the planchette
    TODO!!!
    '''

    #Scaling for later use
    scale = 250 #Because half the hieght of the square is 250mm
    angle_x = angle_y = angle_z = 0

    '''
    Main Loop
    '''
    while True:

        # Get values from Unity
        pac, address = sock.recvfrom(35)
        data = [float(k) for k in str(pac, 'UTF-8').split(',')]


        # Matching up data by converting degrees to radian
        angle_x = math.radians(data[0])
        angle_y = math.radians(data[1])
        angle_z = math.radians(data[2])


        Uy = data[3]

        # Setup for alg
        #points = [0 for _ in range(len(cube_points))]


        '''
        Rotation of Points

        TODO!!!
        We need to make 2 more for loops that correlate with the other faces. So far we only have the outwards face completed.
        TODO!!!
        '''

        for p in Parent:
            p.send(angle_x, angle_y, angle_z, Uy)
        
        for p in Parent:
            if p.recv():
                print("Points updated")
        
        for LEDStrip in LED_Strips:
            LEDStrip.pixels.show()
        

