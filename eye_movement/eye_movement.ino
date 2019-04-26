#include <iq_module_communicaiton.hpp>


// MAKE SURE THAT MOTOR 1 (SERIAL1) CHANGES THE X OF THE EYE
// AND THAT MOTOR 2 (SERIAL2) CHANGES THE Y OF THE EYE
#define TRAJECTORY_TIME 0.05
#define M1Kp 3
#define M1Kd 0
#define M1Ki 0

#define M2Kp 3
#define M2Kd 0
#define M2Ki 0

// Define PID gains
#define Kp 0.04 //0.04
#define Kd 0.02 //0.02
#define Ki 0.01 //0.01


IqSerial iq2(Serial3);
IqSerial iq1(Serial2);

MultiTurnAngleControlClient angle(0);
BrushlessDriveClient mot(0);

// size of image in pizel
//const int image_pixel_height = 920;
//const int image_pixel_width = 1280;

const int image_pixel_height = 256;
const int image_pixel_width = 256;

// calibrated angles to border of image
const float m1_upper_limit = 0.66;
const float m1_lower_limit = -0.33;
const float m2_upper_limit = 0.24;
const float m2_lower_limit = -0.34;

// array of linspaced angles to pixels of image
float height_linspace_array[image_pixel_height] = {0};
float width_linspace_array[image_pixel_width] = {0};

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  delay(1000);
  
  iq1.begin(115200);
  iq2.begin(115200);

  //set the current angle to 0
  iq1.set(angle.obs_angular_displacement_, 0.0f);
  iq2.set(angle.obs_angular_displacement_, 0.0f);
//
//  iq1.set(angle.ctrl_angle_, 0.0f);
//  iq2.set(angle.ctrl_angle_, 0.0f);

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
  Serial.print("x = ");
  Serial.print(x);
  Serial.print(" | y = ");
  Serial.println(y);
  
  
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

//  setMotorAnglesPID(m1_desired_angle, m2_desired_angle);
}

void linspaceAngle2Pixel()
{
  linspace(m1_lower_limit, m1_upper_limit, image_pixel_height, &height_linspace_array[0]);
  linspace(m2_lower_limit, m2_upper_limit, image_pixel_width, &width_linspace_array[0]);

//  Serial.print(height_linspace_array[0]);
//  Serial.print(" | ");
//  Serial.println(height_linspace_array[image_pixel_height-1]);
//  Serial.print(width_linspace_array[0]);
//  Serial.print(" | ");
//  Serial.println(width_linspace_array[image_pixel_width-1]);
//  for(int ii = 0; ii < image_pixel_height; ++ii)
//  {
//    Serial.println(height_linspace_array[ii]);
//  }
//  Serial.println("----------------------------------");
//  for(int ii = 0; ii < image_pixel_width; ++ii)
//  {
//    Serial.println(width_linspace_array[ii]);
//  }
//  Serial.println("----------------------------------");
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



void setMotorAngles(float m1_desired_angle, float m2_desired_angle)
{
  if(m1_desired_angle >= m1_upper_limit)
  {
    m1_desired_angle = m1_upper_limit;
  }
  if(m1_desired_angle <= m1_lower_limit)
  {
    m1_desired_angle = m1_lower_limit;
  }


  if(m2_desired_angle >= m2_upper_limit)
  {
    m2_desired_angle = m2_upper_limit;
  }
  if(m2_desired_angle <= m2_lower_limit)
  {
    m2_desired_angle = m2_lower_limit;
  }

  sendTrajectory(TRAJECTORY_TIME, m1_desired_angle, m2_desired_angle);

  uint8_t mode1 = 0;
  uint8_t mode2 = 0;
 
  do
  {
    iq1.get(angle.ctrl_mode_, mode1);
    iq2.get(angle.ctrl_mode_, mode2);
  }while(mode1 == 4 && mode2 ==4); // Check if the motor is still executing the last trajectory


  return;
}

void sendTrajectory(float time_cmd, float angle_cmd1, float angle_cmd2)
{
  // Generate the set messages
  iq1.set(angle.trajectory_angular_displacement_,angle_cmd1);
  iq1.set(angle.trajectory_duration_,time_cmd);
  iq2.set(angle.trajectory_angular_displacement_,angle_cmd2);
  iq2.set(angle.trajectory_duration_,time_cmd);
}



void setMotorAnglesPID(float m1_desired_angle, float m2_desired_angle)
{ 
  float my_angle1 = m1_desired_angle; // in case packet drops
  float my_angle2 = m2_desired_angle;
  iq1.get(angle.obs_angular_displacement_, my_angle1);
  iq2.get(angle.obs_angular_displacement_, my_angle2);

  float error1 = m1_desired_angle - my_angle1;
  float error2 = m2_desired_angle - my_angle2;
  
  float pid_start_time1 = millis();
  float voltage1 = PID(pid_start_time1, error1);
  
  float pid_start_time2 = millis();
  float voltage2 = PID(pid_start_time2, error2);

  iq1.set(mot.drive_spin_volts_,voltage1);
  iq2.set(mot.drive_spin_volts_,voltage2);
}



float PID(float start_time, float error)
{
  static float error_old = 0.0f;
  static float Ui_old = 0.0f;
  static float delta_t = 1;
  if (error <= 0.1 && error >= -0.1)
  {
    Ui_old = 0;
  }
  float Ui = Ui_old + Ki*error*delta_t;
  if (Ui > 0.07)
  {
    Ui = 0.07;
  }
  if (Ui < -0.07)
  {
    Ui = -0.07;
  }
  float Ud =  Kd*((error - error_old)/ delta_t);
  float U = Kp * error + Ud + Ui;
  Ui_old = Ui;
  error_old = error;

  float run_time = millis()-start_time;
  if(run_time < delta_t)
  {
    delay(delta_t - run_time);
  }

  //  Serial.print("U = ");
//  Serial.println(U);
//  Serial.print("Ud = ");
//  Serial.println(Ud);
//  Serial.print("Ui = ");
//  Serial.println(Ui);
//  Serial.print("error = ");
//  Serial.println(error);
//  Serial.print("START POS ");
//  Serial.println(starting_pos);

  return U;
}
