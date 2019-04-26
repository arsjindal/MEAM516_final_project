#include <iq_module_communicaiton.hpp>


#define TRAJECTORY_TIME 0.5
#define M1Kp 0
#define M1Kd 0
#define M1Ki 0

#define M2Kp 0
#define M2Kd 0
#define M2Ki 0

IqSerial iq2(Serial2);
IqSerial iq1(Serial3);


MultiTurnAngleControlClient angle(0);

//const float m1_upper_limit = 0.70;
//const float m1_lower_limit = -0.49;
//
//const float m2_upper_limit = 0.93;
//const float m2_lower_limit = -0.35;


void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("HHEEELLOOO WORRRLLDDD");
  
  iq1.begin(115200);
  iq2.begin(115200);

  iq1.set(angle.angle_Kp_,(float)M1Kp);
  iq1.set(angle.angle_Ki_,(float)M1Ki);
  iq1.set(angle.angle_Kd_,(float)M1Kd);

  iq2.set(angle.angle_Kp_,(float)M2Kp);
  iq2.set(angle.angle_Ki_,(float)M2Ki);
  iq2.set(angle.angle_Kd_,(float)M2Kd);

  iq1.set(angle.obs_angular_displacement_, 0.0);
  iq2.set(angle.obs_angular_displacement_, 0.0);
  iq1.save(angle.obs_angular_displacement_);
  iq2.save(angle.obs_angular_displacement_);

  
  delay(100);

}

void loop() {
  static float my_angle1;
  static float my_angle2;
  static float m1_desire_angle;
  static float m2_desire_angle;
  
  iq1.get(angle.obs_angular_displacement_, my_angle1);
  iq2.get(angle.obs_angular_displacement_, my_angle2);
  Serial.print("motor 1 = ");
  Serial.print(my_angle1);
  Serial.print(" | motor 2 = ");
  Serial.println(my_angle2);
}
