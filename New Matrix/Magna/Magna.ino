#include <MatrixMath.h>

#include<Uduino.h>
#include <Adafruit_NeoPixel.h>

Uduino uduino("myArduinoName");

#define PIN        6
#define NUMPIXELS 60
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int x,y,z;

//Default Variables made by Ian
float ROTATE_SPEED = 0.001;
int projection_matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 0}};
int cube_points_X = 240;
int cube_points_Y = 3;
int cube_points[240][3];

void setup()
{
  pixels.begin();
  Serial.begin(115200);
  uduino.addCommand("myCommand", Command);
}

void multiply_m(float *MatrixA, float *MatrixB, int rowA, int colA, int colB, float *Matrix){
  
}

void Setup_cube_points(){
  int y = 30;
  for (int x = -30; x < 31; x++){
    if (y == -1){
        y = y + 240;
    }
    cube_points[y][0] = x * ((1/60) *2);
    cube_points[y][1] = 0;
    cube_points[y][2] = 1;
    y = y - 1;
  }
  //Left Side
  y = 30;
  for (int x = -30; x < 30; x++){
    //cube_points[y] = [[-1], [0], [x * -((1 / 60) * 2)]];
    cube_points[y][0] = -1;
    cube_points[y][1] = 0;
    cube_points[y][2] = x * ((1/60) *2);
    y = y + 1;
  }
  for (int x = -30; x < 30; x++){
    //cube_points[y] = [[x * ((1 / 60) * 2)], [0], [-1]];
    cube_points[y][0] = x * ((1/60) *2);
    cube_points[y][1] = 0;
    cube_points[y][2] = -1;
    y = y + 1;
  }
  for (int x = -30; x < 30; x++){
    //cube_points[y] = [[1], [0], [x * ((1 / 60) * 2)]];
    cube_points[y][0] = 1;
    cube_points[y][1] = 0;
    cube_points[y][2] = x * ((1/60) *2);
    y = y + 1;
  }    
}

void Command() {
  int parameters = uduino.getNumberOfParameters(); // returns 2
  if(parameters > 0) {
    int x = uduino.charToInt(uduino.getParameter(0));
    int y = uduino.charToInt(uduino.getParameter(1));
    int z = uduino.charToInt(uduino.getParameter(2));
    int rx = uduino.charToInt(uduino.getParameter(3));
    int ry = uduino.charToInt(uduino.getParameter(4));
    int rz = uduino.charToInt(uduino.getParameter(5));
    //Matrix(x,y,z,rx,ry,rz);
  }
  
}
int scale = 100;
int angle_x = 0;
int angle_y = 0;
int angle_z = 0;

void loop()
{
  pixels.clear();
  float rotation_x[3][3] = {{1, 0, 0}, {0, cos(angle_x), -sin(angle_x)}, {0, sin(angle_x), cos(angle_x)}};
  float rotation_y[3][3] = {{cos(angle_y), 0, sin(angle_y)}, {0, 1, 0}, {-sin(angle_y), 0, cos(angle_y)}};
  float rotation_z[3][3] = {{cos(angle_z), -sin(angle_z), 0}, {sin(angle_z), cos(angle_z), 0}, {0, 0, 1}};

  int points[240];
  int j = 0;
  for (int i = 0; i < cube_points_X; i++){
    float rotate_x[3][3];
    multiply_m(rotation_x, cube_points[i], 3, 3, 3, rotate_x);
  }
      
  uduino.update();
}
