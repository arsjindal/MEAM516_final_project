#include <iq_module_communicaiton.hpp>


// MAKE SURE THAT MOTOR 1 (SERIAL1) CHANGES THE X OF THE EYE
// AND THAT MOTOR 2 (SERIAL2) CHANGES THE Y OF THE EYE
#define TRAJECTORY_TIME 0.5
#define M1Kp 2
#define M1Kd 0
#define M1Ki 0

#define M2Kp 1
#define M2Kd 0
#define M2Ki 0

IqSerial iq2(Serial2);
IqSerial iq1(Serial1);

MultiTurnAngleControlClient angle(0);

// size of image in pizel
const int image_pixel_height = 920;
const int image_pixel_width = 1280;

// calibrated angles to border of image
const float m1_upper_limit = 0.70;
const float m1_lower_limit = -0.49;
const float m2_upper_limit = 0.93;
const float m2_lower_limit = -0.35;

// array of linspaced angles to pixels of image
float height_linspace_array[image_pixel_height] = {0};
float width_linspace_array[image_pixel_width] = {0};

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  delay(100);
  
  iq1.begin(115200);
  iq2.begin(115200);

  //set the current angle to 0
  iq1.set(angle.obs_angular_displacement_, 0.0f);
  iq2.set(angle.obs_angular_displacement_, 0.0f);

  // PID values
  iq1.set(angle.angle_Kp_,(float)M1Kp);
  iq1.set(angle.angle_Ki_,(float)M1Ki);
  iq1.set(angle.angle_Kd_,(float)M1Kd);

  iq2.set(angle.angle_Kp_,(float)M2Kp);
  iq2.set(angle.angle_Ki_,(float)M2Ki);
  iq2.set(angle.angle_Kd_,(float)M2Kd);

  //linspace the pixel values to motor angles
  linspaceAngle2Pixel();
}

void loop() {

  // get the pixel values from serial
  static int x = 0;
  static int y = 0;
  while(!readPixelValues(x,y));
  
  aimAtPixel(x,y); // aims the motor

  // display angle of motor
  static float my_angle1;
  static float my_angle2;
  iq1.get(angle.obs_angular_displacement_, my_angle1);
  iq2.get(angle.obs_angular_displacement_, my_angle2);
  Serial.print("motor 1 = ");
  Serial.print(my_angle1);
  Serial.print(" angle it should be = ");
  Serial.println(height_linspace_array[y]);
  Serial.print("motor 2 = ");
  Serial.print(my_angle2);
  Serial.print(" angle it should be = ");
  Serial.println(width_linspace_array[x]);
}

bool readPixelValues(int &x, int &y)
{
  if(Serial.available())
  {
    String str = Serial.readStringUntil('\n');
    x = str.substring(0,3).toInt();
    y = str.substring(3,6).toInt();
  }
  else
  {
    return 0;
  }
  return 1;
}

void aimAtPixel(int x, int y)
{
  float m1_desired_angle = height_linspace_array[y];
  float m2_desired_angle = width_linspace_array[x];

  setMotorAngles(m1_desired_angle, m2_desired_angle);
}

void linspaceAngle2Pixel()
{
  linspace(m1_lower_limit, m1_upper_limit, image_pixel_height, &height_linspace_array[0]);
  linspace(m2_lower_limit, m2_upper_limit, image_pixel_width, &width_linspace_array[0]);
  return;
}

void linspace(float lower_lim, float upper_lim, int n, float* linspace_Array)
{
    /* step size */
    double step_size = (upper_lim - lower_lim)/(n - 1);
    
    /* fill vector */
    for(int ii = 0; ii < n - 1; ++ii)
    {
      *linspace_Array = lower_lim + ii*step_size;
      linspace_Array++;
    }
 
    /* fix last entry */
    *linspace_Array = upper_lim;
  
    return;
}
void setMotorAngles(float m1_desire_angle, float m2_desire_angle)
{
  if(m1_desire_angle >= m1_upper_limit)
  {
    m1_desire_angle = m1_upper_limit;
  }
  if(m1_desire_angle <= m1_lower_limit)
  {
    m1_desire_angle = m1_lower_limit;
  }


  if(m2_desire_angle >= m2_upper_limit)
  {
    m2_desire_angle = m2_upper_limit;
  }
  if(m2_desire_angle <= m2_lower_limit)
  {
    m2_desire_angle = m2_lower_limit;
  }

  sendTrajectory1(TRAJECTORY_TIME, m1_desire_angle);
  sendTrajectory2(TRAJECTORY_TIME, m2_desire_angle);
  delay(1000);

  return;
}

void sendTrajectory1(float time_cmd, float angle_cmd)
{
  // Generate the set messages
  iq1.set(angle.trajectory_angular_displacement_,angle_cmd);
  iq1.set(angle.trajectory_duration_,time_cmd);
}

void sendTrajectory2(float time_cmd, float angle_cmd)
{
  // Generate the set messages
  iq2.set(angle.trajectory_angular_displacement_,angle_cmd);
  iq2.set(angle.trajectory_duration_,time_cmd);
}
