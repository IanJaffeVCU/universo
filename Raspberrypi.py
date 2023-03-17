''':
Author: Miles Popiela
Contact: popielamc@vcu.edu

Description: Creating LED visualization of 3D Point rotation
matrix with rotation input being rotational and Y-Axis values
of a game object within unity using UDP sockets
'''

from math import *
import board
import neopixel
import socket
import math
import numpy

from threading import Thread
from multiprocessing import Pipe
#from numba import jit


#dev = False

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
        #self.pixels.show()

        # Setting points
        self.cube_points = [n for n in range(self.num_pixels)]

        self.projection_matrix = [[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 0]]
    
    def Setup_Virtual_Points(self, input): #Decide how to deal with range
        mid, Frontside, Leftside, Backside, Rightside = input
        #if dev:
        #    print("Setting up points for " + self.Name)
        # Front side
        y = mid
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
    
    #@jit(nopython=True)
    def RotationOfPoints(self, input):
        #if dev:
        #    print("Updating points for " + self.Name)
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
            try:
                #rotate_x = multiply_m(rotation_x, point)
                #rotate_y = multiply_m(rotation_y, rotate_x)
                #rotate_z = multiply_m(rotation_z, rotate_y)
                #point_2d = multiply_m(self.projection_matrix, rotate_z)

                rotate_x = numpy.dot(rotation_x, point)
                rotate_y = numpy.dot(rotation_y, rotate_x)
                rotate_z = numpy.dot(rotation_z, rotate_y)
                point_2d = numpy.dot(self.projection_matrix, rotate_z)

                '''
                Takes virtual plot points and combineds them with

                must think of method of simplification ~ ** The robots' Y-axis ** ~ must think of method of simplification

                '''
                #Local_y = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
                #Color = Local_y 

                Color = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
                if Color > 255:
                    Color = 255
                elif Color < 0:
                    Color = 0

                self.pixels[i] = (255 - int(Color), 70, int(Color))
                i += 1
            except Exception as e:
                print(e)
                #if dev:
                #    print(e)
    def run(self):
        self.Setup_Virtual_Points(self.Child.recv())
        self.Child.send(self.Name)
        self.pixels.show()
        
        while self.Running:
            self.RotationOfPoints(self.Child.recv())
            self.Child.recv()
            self.pixels.show()
            self.Child.send(True)

        super().run()

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
    Pipes = [Pipe() for k in range (3)] # in, out = Pipe
    Parent = [k[0] for k in Pipes]
    #Front, Bottom, Inner, Top
    #Name, Pin, Number of LEDs
    LED_Strips = [  LED_Strip('Top/Bottom',    Pipes[0][1], board.D18, 104), #pin 12 GPIO 18
                    LED_Strip('Front',  Pipes[1][1], board.D21, 120), #pin 40 GPIO 21
                    LED_Strip('Inner', Pipes[2][1], board.D10, 108)]#,/
                    #LED_Strip('Bottom',Pipes[3][1], board.D27, 120)]
    
    Points = []
    Points.append([13, [-13,14], [-13,13], [-13,13], [-13,13]])
    Points.append([15, [-15,16], [-15,15], [-15,15], [-15,15]])
    Points.append([14, [-14,15], [-13,13], [-14,14], [-13,13]])
    for i in range(len(LED_Strips)):
        LED_Strips[i].start()
        Parent[i].send(Points[i])
    
    for p in Parent:
        Recieve = p.recv()
        #if dev:
        #    print('Points Setup Succesful for ' + p.recv())

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
                p.send([angle_x, angle_y, angle_z, Uy])

        for p in Parent:
            p.send(True)
            p.recv()
            #if p.recv():
            #    if dev:
            #        print("LED Updated")
            
        

