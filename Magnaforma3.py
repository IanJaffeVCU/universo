'''
Author: Miles Popiela
Contact: popielamc@vcu.edu
Description: Creating LED visualization of 3D Point rotation
matrix with rotation input being rotational and Y-Axis values
of a game object within unity using UDP sockets
'''

from math import cos, sin, radians
#import RPi.GPIO as GPIO
#import time
import rpi_ws281x as ws
from rpi_ws281x import PixelStrip, Color
import socket
import threading
from numpy import dot as multiply_m
from multiprocessing import Pool

projection_matrix = [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 0]]

class LED_Strip:
    def __init__(self, Name, pixel_pin, num_pixels):
        self.Name = Name
        #self.pixel_pin = pixel_pin
        #self.num_pixels = num_pixels

        self.strip = PixelStrip(
            num_pixels,
            pixel_pin,
            800000,
            10,
            False,
            brightness=100,
            channel=0,
            strip_type=ws.WS2811_STRIP_GRB
        )
        self.strip.begin()
        
        #SET ALL PIXELS TO GREEN
        for i in range(num_pixels):
            self.strip.setPixelColor(i, Color(0,0,255))
        self.strip.show()

        # Setting points
        self.cube_points = [n for n in range(num_pixels)]
        self.lock = threading.Lock()
    
    def Setup_Virtual_Points(self, Frontside, Leftside, Backside, Rightside):
        
        #Time for some Math
        side_length = len(Frontside)
        spacing = 1 / (side_length + 1)

        # Front side
        for i, idx in enumerate(Frontside):
            x = -1 + (i + 1) * spacing
            
            self.cube_points[idx] = [-x, 0, 1]

        # Left side
        for i, idx in enumerate(Leftside):
            z = 1 - (i + 1) * spacing
            self.cube_points[idx] = [-1, 0, z]

        # Back side
        for i, idx in enumerate(Backside):
            x = 1 - (i + 1) * spacing
            self.cube_points[idx] = [-x, 0, -1]

        # Right side
        for i, idx in enumerate(Rightside):
            z = -1 + (i + 1) * spacing
            self.cube_points[idx] = [1, 0, z]

    def update_strip(self, Colors, barrier):
        for i, Color in enumerate(Colors):
            self.pixels[i] = (255 - Color, 70, Color)
        barrier.wait()
        self.strip.show()

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
        rotate_x = multiply_m(rotation_x, point)
        rotate_y = multiply_m(rotation_y, rotate_x)
        rotate_z = multiply_m(rotation_z, rotate_y)
        point_2d = multiply_m(projection_matrix, rotate_z)

        '''
        Takes virtual plot points and combineds them with

        must think of method of simplification ~ ** The robots' Y-axis ** ~ must think of method of simplification

        '''

        Color = ((point_2d[1][0] * scale) + (Uy * 1000) + 150)
        if Color > 255:
            Color = 255
        elif Color < 0:
            Color = 0
        
        Colors.append(int(Color))
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

    #Front, Bottom, Inner, Top
    #Name, Pin, Number of LEDs
    LED_Strips = [  LED_Strip('Top', 12, 105),
                    LED_Strip('Inner', 21, 110),
                    LED_Strip('Front', 18, 120)
                ]
    
    #y, Frontside, Leftside, Backside, Rightside
    LED_Strips[0].Setup_Virtual_Points(list(range(91, 105)) + list(range(13)), list(range(13, 39)), list(range(39, 65)), list(range(65, 91)))
    LED_Strips[1].Setup_Virtual_Points(list(range(96, 110)) + list(range(14)), list(range(14, 41)), list(range(41, 69)), list(range(69, 96)))
    LED_Strips[2].Setup_Virtual_Points(list(range(105, 120)) + list(range(15)), list(range(15, 45)), list(range(45, 75)), list(range(75, 105)))
    

    '''
    Setting Up Virtual Points
    TODO!!!
    We need to make two more cube_points for inner,top, and bottom faces of the planchette
    TODO!!!
    '''

    #Scaling for later use
    scale = 250 #Because half the height of the square is 250mm


    angle_x = angle_y = angle_z = 0

    
    #Main Loop
    
    while True:

        # Get values from Unity
        pac, address = sock.recvfrom(35)
        data = [float(k) for k in str(pac, 'UTF-8').split(',')]
        

        # Matching up data by converting degrees to radian
        angle_x = radians(data[0])
        angle_y = radians(data[1])
        angle_z = radians(data[2])
        Uy = data[3]

        barrier = threading.Barrier(len(LED_Strips))

        with Pool() as pool:
            Array = [[angle_x, angle_y, angle_z, data[3], LED_Strips[0].cube_points],
                     [angle_x, angle_y, angle_z, data[3], LED_Strips[1].cube_points],
                     [angle_x, angle_y, angle_z, data[3], LED_Strips[2].cube_points],]
            Colorss = pool.map(RotationOfPointsStandAlone, Array)
            
            threads = []
            for i, Colors in enumerate(Colorss):
                #THREADS
                thread = threading.Thread(target=LED_Strips[i].update_strip, args=(Colors, barrier))
                threads.append(thread)
                thread.start()
            for thread in threads:
                thread.join()