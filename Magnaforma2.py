''':
Author: Miles Popiela
Contact: popielamc@vcu.edu

Description: Creating LED visualization of 3D Point rotation
matrix with rotation input being rotational and Y-Axis values
of a game object within unity using UDP sockets
'''

import neopixel
import socket

from time import sleep
from itertools import count as itertoolsCount
from board import D10, D18, D21, D12
from numpy import dot as MatrixMultiplication
from math import radians, cos, sin
#from threading import Thread
#from multiprocessing import Pipe
#from numba import jit

from multiprocessing import Pool, Process
from multiprocessing.dummy import Pool as ThreadPool


#dev = False

#class LED_Strip(Thread):

projection_matrix = [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 0]]

class LED_Strip():
    id_iter = itertoolsCount()
    def __init__(self, Name, pixel_pin, num_pixels):
        super().__init__()
        self.Name = Name
        self.id = next(self.id_iter)
        #self.Child = Child
        #self.Update = Update
        self.pixel_pin = pixel_pin
        self.num_pixels = num_pixels
        self.Running = True

        self.pixels = neopixel.NeoPixel(
            self.pixel_pin, self.num_pixels, brightness=100, auto_write=False, pixel_order=neopixel.GRB
        )

        self.pixels.fill((0, 255, 0))
        #self.pixels.show()

        # Setting points
        self.cube_points = [n for n in range(self.num_pixels)]
    
    def Setup_Virtual_Points(self, mid, Frontside, Leftside, Backside, Rightside): #Decide how to deal with range
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
    def RotationOfPoints(self, angle_x, angle_y, angle_z, Uy):
        rotation_x = [[1, 0, 0],
                      [0, cos(angle_x), -sin(angle_x)],
                      [0, sin(angle_x), cos(angle_x)]]

        rotation_y = [[cos(angle_y), 0, sin(angle_y)],
                      [0, 1, 0],
                      [-sin(angle_y), 0, cos(angle_y)]]

        rotation_z = [[cos(angle_z), -sin(angle_z), 0],
                      [sin(angle_z), cos(angle_z), 0],
                      [0, 0, 1]]
        
        for i, point in enumerate(self.cube_points):
            #try:
            rotate_x = MatrixMultiplication(rotation_x, point)
            rotate_y = MatrixMultiplication(rotation_y, rotate_x)
            rotate_z = MatrixMultiplication(rotation_z, rotate_y)
            point_2d = MatrixMultiplication(projection_matrix, rotate_z)

            '''
            Takes virtual plot points and combineds them with

            must think of method of simplification ~ ** The robots' Y-axis ** ~ must think of method of simplification

            '''

            Color = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
            if Color > 255:
                Color = 255
            elif Color < 0:
                Color = 0

            self.pixels[i] = (255 - int(Color), 70, int(Color))
    
    def UpdatePixels(self, Colors):
        for i, Color in enumerate(Colors):
            self.pixels[i] = (255 - int(Color), 70, int(Color))
        self.pixels.show()
'''
    def run(self):
        self.Setup_Virtual_Points(self.Child.recv())
        #self.Child.send(self.Name)
        
        while self.Running:
            self.RotationOfPoints(self.Child.recv())
            #self.Update.recv()
            #self.pixels.show()
            self.Update.send(self.id)
            #self.Child.recv()
            #self.pixels.show()
            #self.Child.send(True)

        super().run()

class Update_LED(Thread):
    def __init__(self, Updates, LED_Strips):
        super().__init__()
        self.Updates = Updates
        self.LED_Strips = LED_Strips
        self.start()
   
    def run(self):
        while(True):
            for Update in self.Updates:
                self.LED_Strips[Update.recv()].pixels.show()
                #Update.send(True)
                #pixels = Update.recv()
                #pixels.show()
        super().run()
'''

def RotationOfPointsStandAlone(input):
    angle_x, angle_y, angle_z, Uy, cube_points = input
    rotation_x = [[1, 0, 0],
                  [0, cos(angle_x), -sin(angle_x)],
                  [0, sin(angle_x), cos(angle_x)]]

    rotation_y = [[cos(angle_y), 0, sin(angle_y)],
                  [0, 1, 0],
                  [-sin(angle_y), 0, cos(angle_y)]]

    rotation_z = [[cos(angle_z), -sin(angle_z), 0],
                  [sin(angle_z), cos(angle_z), 0],
                  [0, 0, 1]]
    Colors = []
    for point in cube_points:
        #try:
        rotate_x = MatrixMultiplication(rotation_x, point)
        rotate_y = MatrixMultiplication(rotation_y, rotate_x)
        rotate_z = MatrixMultiplication(rotation_z, rotate_y)
        point_2d = MatrixMultiplication(projection_matrix, rotate_z)

        '''
        Takes virtual plot points and combineds them with

        must think of method of simplification ~ ** The robots' Y-axis ** ~ must think of method of simplification

        '''

        Color = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
        if Color > 255:
            Color = 255
        elif Color < 0:
            Color = 0
        
        Colors.append(Color)
    return Colors

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
    #LED_Pipes = [Pipe() for k in range (3)] # in, out = Pipe
    #UPDATE_Pipes = [Pipe() for k in range (len(LED_Pipes))] # in, out = Pipe
    #Parent = [k[0] for k in LED_Pipes]

    #Front, Bottom, Inner, Top
    #Name, Pin, Number of LEDs
    #LED_Strips = [  LED_Strip('Front',      LED_Pipes[0][1], UPDATE_Pipes[0][1], D18, 120), #pin 12 GPIO 18
    #                LED_Strip('Inner',      LED_Pipes[1][1], UPDATE_Pipes[1][1], D21, 108), #pin 40 GPIO 21
    #                LED_Strip('Top/Bottom', LED_Pipes[2][1], UPDATE_Pipes[2][1], D12, 104)] #pin 32 GPIO 12
    #Update_LED([k[0] for k in UPDATE_Pipes], LED_Strips)
    
    LED_Strips = [  LED_Strip('Front',       D18, 120), #pin 12 GPIO 18
                    LED_Strip('Inner',       D21, 108), #pin 40 GPIO 21
                    LED_Strip('Top/Bottom',  D12, 104)] #pin 32 GPIO 12
    

    #Points = []
    #Points.append([15, [-15,16], [-15,15], [-15,15], [-15,15]])
    #Points.append([13, [-13,14], [-13,13], [-13,13], [-13,13]])
    #Points.append([14, [-14,15], [-13,13], [-14,14], [-13,13]])
    #for i in range(len(LED_Strips)):
    #    LED_Strips[i].start()
    #    Parent[i].send(Points[i])
    
    LED_Strips[0].Setup_Virtual_Points(15, [-15,16], [-15,15], [-15,15], [-15,15])
    LED_Strips[1].Setup_Virtual_Points(13, [-13,14], [-13,13], [-13,13], [-13,13])
    LED_Strips[2].Setup_Virtual_Points(14, [-14,15], [-13,13], [-14,14], [-13,13])
    

    #for p in Parent:
    #    Recieve = p.recv()

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
        angle_x = radians(data[0])
        angle_y = radians(data[1])
        angle_z = radians(data[2])


        #Uy = data[3]

        # Setup for alg
        #points = [0 for _ in range(len(cube_points))]


        '''
        Rotation of Points

        TODO!!!
        We need to make 2 more for loops that correlate with the other faces. So far we only have the outwards face completed.
        TODO!!!
        '''

        with Pool() as pool:
            Array = [[angle_x, angle_y, angle_z, data[3], LED_Strips[0].cube_points],
                     [angle_x, angle_y, angle_z, data[3], LED_Strips[1].cube_points],
                     [angle_x, angle_y, angle_z, data[3], LED_Strips[2].cube_points],]
            Colorss = pool.map(RotationOfPointsStandAlone, Array)
            for i, Colors in enumerate(Colorss):
                LED_Strips[i].UpdatePixels(Colors)

        #for p in Parent:
        #        p.send([angle_x, angle_y, angle_z, data[3]])

        #for LED_Strip in LED_Strips:
        #    LED_Strip.RotationOfPoints(angle_x, angle_y, angle_z, data[3])

        #for p in Parent:
            #p.send(True)
            #p.recv()
            #if p.recv():
            #    if dev:
            #        print("LED Updated")