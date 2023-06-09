
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
// Might need to include, maybe not ??
// #include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// Might need to include, maybe not??
// #include <nav_msgs/Odometry.h>
#include <std_msgs/UInt32.h>
#include <cstring>
// Various bitfields
byte ENCODER_FLAGS = 0x00 // X X DIR_L DIR_R _ AL BL AR BR basically just stores last digitalRead value in LL, and the direction in XH
//  Explanation of ENC_CNT
//    Using unsigned integers we can benefit greatly from
//    overflow. This means on each encoder we can track
//    up to 65535 ticks per cycle, which seems like a safe
//    upper bound. When it rolls over, we could find delta_count
//    easily by calculating (65535 - prev) + curr, and we would
//    know to do this in the event that prev > curr.
//    
//    IDEA ==========================================================
//        A subroutine should now handle the encoder counting.
//        Inside of this subroutine, we could use locally scoped
//    Each unsigned integer is simply the total tick count of 
//    the corresponding encoder, mapped as such
//                         0       1       2       3
//                         ENC_L_A ENC_L_B ENC_R_A ENC_R_B
//unsigned int ENC_CNT[4] = {0x0000, 0x0000, 0x0000, 0x0000};


//    The following fields we make volatile, due to their inclusion
//    in interrupt service routines.
//                                    0       1
//                                    PULSE_L PULSE_R
volatile unsigned int PULSE_COUNTER[2] = {0x0000, 0x0000};
volatile byte PULSE_VALUE_BYTE_OLD = 0b00000000;
// Keep the old values on high byte, new on low byte
volatile byte PULSE_VALUE_BYTE = 0b00000000;
volatile byte PULSE_DIRECTION_BYTE = 0b00000000;

//  Constants for calculations
// Loop period in milliseconds
unsigned int LOOP_PERIOD = 50;
// Divide loop period by 1000 to get Hz
float FREQUENCY = 1.0 / (LOOP_PERIOD / 1000.0);
float WHEEL_RADIUS = 3.3;
unsigned int CYCLE_PULSE_COUNTER[]  = {0, 0};
byte CYCLE_PULSE_DIRECTION_BYTE = 0x00;

float VELOCITY_CALCULATED_LEFT = 0.0;
float VELOCITY_CALCULATED_RIGHT = 0.0;

// A_8 -- PK0 -- PCINT16 -- PCMSK2 |= 0b00000001
// A_9 -- PK1 -- PCINT17 -- PCMSK2 |= 0b00000010
// A10 -- PK2 -- PCINT18 -- PCMSK2 |= 0b00000100
// A11 -- PK3 -- PCINT19 -- PCMSK2 |= 0b00001000

byte PWM_VALUE_LEFT = 255;
byte PWM_VALUE_RIGHT = 255;

//  Explanation of idea behind M_DTY --
//    We convert the duty cycle percentage to the [0, 255] range,
//    set the DIR_L/R flags equal to,
//    for L, !bitRead(LEFT_MOTOR_DUTY_CYCLE, 31),
//    so negative LEFT_MOTOR_DUTY_CYCLE (should) imply DIR_L = 0 thus CCW rotation, 
//    and for R, bitRead(RIGHT_MOTOR_DUTY_CYCLE, 31),
//    so negative RIGHT_MOTOR_DUTY_CYCLE (should) imply DIR_R = 1 thus CW rotation,
//    with DIR_L/R = 0 implying relative forward motor drive.
//    we can then easily store and retrieve these anywhere,
//    and only make the ones involved in the ISR volatile (as is recommended)
unsigned int M_DTY = 0x0000;

//  ALSO BIG IDEA; USE PULSEIN ARDUINO FUNCTION
//  SEEMS LIKE IT MEASURES THE LENGTH OF HIGH / LOW PULSES
//  KNOWING THE DURATION SPECIFICALLY SEEMS TO SIMPLIFY SPEED
//  CALCULATIONS

// Pin variables for motors.
uint8_t MOTOR_OUTPUT_LEFT_PWM = 4;//PG5
uint8_t MOTOR_OUTPUT_LEFT_A = 26;//PA4
uint8_t MOTOR_OUTPUT_LEFT_B = 27;//PA5
uint8_t MOTOR_OUTPUT_RIGHT_PWM = 9;
uint8_t MOTOR_OUTPUT_RIGHT_A = 28;//PA6
uint8_t MOTOR_OUTPUT_RIGHT_B = 29;//PA7

// Pin variables for encoders
// changed pins back to original, and implemented more robust ISR
uint8_t ENCODER_INPUT_LEFT_A = A8;
uint8_t ENCODER_INPUT_LEFT_B = A9;
uint8_t ENCODER_INPUT_RIGHT_A = A10;
uint8_t ENCODER_INPUT_RIGHT_B = A11;
// For looping in ISR
uint8_t ENCODER_INPUT_ARRAY = {ENCODER_INPUT_LEFT_A, ENCODER_INPUT_LEFT_B, ENCODER_INPUT_RIGHT_A, ENCODER_INPUT_RIGHT_B};


//  Turns a "signed" percentage [-1.00, ..., 1.00] to an "unsigned"
//  byte, as this allows us to simplify velocity commands, as we
//  can provide them as a duty cycle, and calculate velocity in ODOM
byte sPERCENT_TO_uBYTE(float p) {
  byte p_b = (byte) (100.00 * abs(p));
  byte b = (byte) (map(p_b, 0, 100, 85, 255));
  return b;
}

byte GET_MOTOR_PWM_LEFT() {
  return (byte) highByte(M_DTY);
}

void SET_MOTOR_PWM_LEFT(byte NEW_VALUE) {
  M_DTY &= 0x00FF;
  M_DTY |= (((unsigned int) NEW_VALUE) << 8);
}

void INC_MOTOR_PWM_LEFT() {
  byte OLD_VALUE = GET_MOTOR_PWM_LEFT();
  byte NEW_VALUE = OLD_VALUE + 1;
  SET_MOTOR_PWM_LEFT(NEW_VALUE);
}

void DEC_MOTOR_PWM_LEFT() {
  byte OLD_VALUE = GET_MOTOR_PWM_LEFT();
  byte NEW_VALUE = OLD_VALUE - 1;
  SET_MOTOR_PWM_LEFT(NEW_VALUE);
}

void DEBUG_MOTOR_LEFT() {
  Serial.println("--------\nLEFT WHEEL DATA");
  Serial.println(bitRead(CYCLE_PULSE_DIRECTION_BYTE, 0));
  Serial.println(CYCLE_PULSE_COUNTER[0]);
  Serial.print(VELOCITY_CALCULATED_LEFT);
  Serial.println(" cm/s");
  Serial.println(GET_MOTOR_PWM_LEFT());
}

byte GET_MOTOR_PWM_RIGHT() {
  return (byte) lowByte(M_DTY);
}

void SET_MOTOR_PWM_RIGHT(byte NEW_VALUE) {
  M_DTY &= 0xFF00;
  M_DTY |= ((unsigned int) NEW_VALUE);
}

void INC_MOTOR_PWM_RIGHT() {
  byte OLD_VALUE = GET_MOTOR_PWM_RIGHT();
  byte NEW_VALUE = OLD_VALUE + 1;
  SET_MOTOR_PWM_RIGHT(NEW_VALUE);
}

void DEC_MOTOR_PWM_RIGHT() {
  byte OLD_VALUE = GET_MOTOR_PWM_RIGHT();
  byte NEW_VALUE = OLD_VALUE - 1;
  SET_MOTOR_PWM_RIGHT(NEW_VALUE);
}

void DEBUG_MOTOR_RIGHT() {
  Serial.println("--------\nRIGHT WHEEL DATA");
  Serial.println(bitRead(CYCLE_PULSE_DIRECTION_BYTE, 1));
  Serial.println(CYCLE_PULSE_COUNTER[1]);
  Serial.print(VELOCITY_CALCULATED_RIGHT);
  Serial.println(" cm/s");
  Serial.println(GET_MOTOR_PWM_RIGHT());
}

void LOAD_DUTY_CYCLES(float LEFT_MOTOR_DUTY_CYCLE, float RIGHT_MOTOR_DUTY_CYCLE) {
  //  Gets the direction from the top bit of float
  //  Unsure if correctly assigned with regards to
  //  physical space
  //  Stores it in the encoder flags
  M_DTY &= 0x0000;
  bitWrite(ENCODER_FLAGS, 5, !bitRead(LEFT_MOTOR_DUTY_CYCLE, 31));
  bitWrite(ENCODER_FLAGS, 4, bitRead(RIGHT_MOTOR_DUTY_CYCLE, 31));
  M_DTY = (unsigned int) ( (unsigned int) (sPERCENT_TO_uBYTE(LEFT_MOTOR_DUTY_CYCLE) << 8) | sPERCENT_TO_uBYTE(RIGHT_MOTOR_DUTY_CYCLE) );
  analogWrite(MOTOR_OUTPUT_LEFT_PWM, sPERCENT_TO_uBYTE(LEFT_MOTOR_DUTY_CYCLE));
  bool LEFT_DIR = bitRead(ENCODER_FLAGS, 5);
  digitalWrite(MOTOR_OUTPUT_LEFT_A, !LEFT_DIR);
  digitalWrite(MOTOR_OUTPUT_LEFT_B, LEFT_DIR);
  analogWrite(MOTOR_OUTPUT_RIGHT_PWM, sPERCENT_TO_uBYTE(RIGHT_MOTOR_DUTY_CYCLE));
  bool RIGHT_DIR = bitRead(ENCODER_FLAGS, 4);
  digitalWrite(MOTOR_OUTPUT_RIGHT_A, !RIGHT_DIR);
  digitalWrite(MOTOR_OUTPUT_RIGHT_B, RIGHT_DIR);
}
void MOTOR_REFRESH() {

  /*
    Seems what I discovered finally in the lab based on gh changes was how to actually drive the engine

    The PWM values are written to a specific place in memory

    Nothing needs changing for the moment in earlier implementation, but this is a place for many things to go wrong

    KEEP THAT IN MIND###########################

    Reads the inverse of sign bit of LEFT_MOTOR_DUTY_CYCLE set in encoder flags
  digitalWrite(MOTOR_OUTPUT_LEFT_B, (bool) bitRead(ENCODER_FLAGS, 5));

  
    Reads the high byte of the M_DTY variable
  analogWrite(MOTOR_OUTPUT_LEFT_A, (int) highByte(M_DTY));
    
    This is where the implementation needs changing, the same is true for the opposite side
  

    Below follows a naive approach
  
    The idea here is based on lines 74-79 in measurevelocity, it seems that the way it is wired, I should just read in one value, and the opposite into the other

    Also, might be smart to clear the engines before changing direction
  */
  analogWrite(MOTOR_OUTPUT_LEFT_PWM, 0);
  digitalWrite(MOTOR_OUTPUT_LEFT_B, (bool) bitRead(ENCODER_FLAGS, 5));
  digitalWrite(MOTOR_OUTPUT_LEFT_A, !((bool) bitRead(ENCODER_FLAGS, 5)));
  analogWrite(MOTOR_OUTPUT_LEFT_PWM, (int) highByte(M_DTY));


  analogWrite(MOTOR_OUTPUT_LEFT_PWM, 0);
  digitalWrite(MOTOR_OUTPUT_RIGHT_B, (bool) bitRead(ENCODER_FLAGS, 4));
  digitalWrite(MOTOR_OUTPUT_RIGHT_A, !((bool) bitRead(ENCODER_FLAGS, 4)));
  analogWrite(MOTOR_OUTPUT_RIGHT_PWM, (int) lowByte(M_DTY));
}

void MOTOR_CLEAR() {
  /*
  digitalWrite(MOTOR_OUTPUT_LEFT_B, HIGH); // Initial state, points backwards
  analogWrite(MOTOR_OUTPUT_LEFT_A, 0); // Initial state, 0 duty cycle
  digitalWrite(MOTOR_OUTPUT_RIGHT_B, LOW); // Initial state, points forwards
  analogWrite(MOTOR_OUTPUT_RIGHT_A, 0); // Initial state, 0 duty cycle
  */
  analogWrite(MOTOR_OUTPUT_LEFT_PWM, 0);
  digitalWrite(MOTOR_OUTPUT_LEFT_B, LOW);
  digitalWrite(MOTOR_OUTPUT_LEFT_A, HIGH);


  analogWrite(MOTOR_OUTPUT_LEFT_PWM, 0);
  digitalWrite(MOTOR_OUTPUT_RIGHT_B, LOW);
  digitalWrite(MOTOR_OUTPUT_RIGHT_A, HIGH);
}

void DEBUG_LOG(int CYCLE_COUNT_DELTA) {
  Serial.println("----------");
  Serial.println("INFO");
  Serial.print("COUNT DELTA ");
  Serial.println(CYCLE_COUNT_DELTA);
  Serial.print("FREQUENCY ");
  Serial.println(FREQUENCY);
  Serial.println(" Hz");
  DEBUG_MOTOR_LEFT();
  DEBUG_MOTOR_RIGHT();  
}

// ATTEMPTED TO IMPLEMENT
void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  
  //  MESSAGE_COMPONENT_X
  int MESSAGE_COMPONENT_X = (int) (100.00 * min(1.00, abs(msg.linear.x)));
  bitSet(MESSAGE_COMPONENT_X, 15, bitRead(msg.linear.x, 63));

  //  MESSAGE_COMPONENT_Z
  int MESSAGE_COMPONENT_Z = (int) (100.00 * min(1.00, abs(msg.angular.z)));
  bitSet(MESSAGE_COMPONENT_Z, 15, bitRead(msg.angular.z, 63));

  float LEFT_MOTOR_DUTY_CYCLE = ((float) ((MESSAGE_COMPONENT_X - MESSAGE_COMPONENT_Z) / 2)) / 100.00; // In case of negative Z, LEFT_MOTOR_DUTY_CYCLE should increase relative to its forward, then negated in all cases (should be negated later on)
  float RIGHT_MOTOR_DUTY_CYCLE = ((float) ((MESSAGE_COMPONENT_X + MESSAGE_COMPONENT_Z) / 2)) / 100.00; // In case of negative Z, RIGHT_MOTOR_DUTY_CYCLE should decrease relative to its forward

  LOAD_DUTY_CYCLES(LEFT_MOTOR_DUTY_CYCLE, RIGHT_MOTOR_DUTY_CYCLE); // Final step of callback should be to simply load the new values
}

//SOMMER ALGORITHM (didel.com/mot/RomEnco.pdf) can be used to find direction
// Pulse val
// High byte : B Right old - A Right old - B Left old - A Left old
// Low byte : B Right new - A Right new - B Left new - A Left new
// Pulse dir (1 means backwards)
// Low byte : x - x - Right dir - Left dir
// Pulse cnt
// Index 1 : Right pulse count
// Index 0 : Left pulse count
ISR(PCINT2_vect) {
  // Has to handle all pins with one interrupt
  for(int i = 0; i < 4; i++) {
    // Shift new value to old (4 bits over)
    bitWrite(PULSE_VALUE_BYTE, i + 4, bitRead(PULSE_VALUE_BYTE, i));
    // Read current value
    bitWrite(PULSE_VALUE_BYTE, i, digitalRead(ENCODER_INPUT_ARRAY[i]));
    // If there was a change on this pin, increment (integer division for index)
    if(bitRead(PULSE_VALUE_BYTE, i + 4) != bitRead(PULSE_VALUE_BYTE, i)) PULSE_COUNTER[(int) i / 2]++;
  }
  // If falling edge on A Right,
  if(bitRead(PULSE_VALUE_BYTE, 6) && !bitRead(PULSE_VALUE_BYTE, 2)) {
    // Copy the opposite of B new
    bitWrite(PULSE_DIRECTION_BYTE, 1, !bitRead(PULSE_VALUE_BYTE, 3));
  }
  // If falling edge on A Left,
  if(bitRead(PULSE_VALUE_BYTE, 4) && !bitRead(PULSE_VALUE_BYTE, 0)) {
    // Copy the same as B new
    bitWrite(PULSE_DIRECTION_BYTE, 0, bitRead(PULSE_VALUE_BYTE, 1));
  }
}
/*-------------------------------------------------------------------
|   IDEA FOR BETTER INTERRUPT HANDLING
| Just an idea, maybe I should just toggle some flags to indicate to
| the main loop something happened.
| 
| But I think if I use very simple commands, the interrupts, even if
| there are 4 of them, should not affect the program too much.
| 
| There are 32 slits on the encoder, and each slit represents 64
| transitions, each of which occur on 2 phases, so 128 signal
| transitions for the encoder outputs of one engine in total,
| which means all in all 256 ISRs will be called per rotation, and
| since the engine reportedly runs at 152 RPM, that will equate to
| around 648.533 interrupts per second, which the 16MHz clock should
| handle just fine, especially assuming an increment and bitwrite
| take up around 1-2 cycles each.
|
| If we then take into account that fact, estimating 1-2 cycles per 
| interrupt statement, that leaves us with around 2594.133 clock
| cycles used up for the ISRs, or around 0.01621333% of the processor
| cycles.
|  
| Considering the reduction ratio of the motor, 1:120, we should
| rather get an output rotational
| speed of around 1.26667 RPM, so more like 324.26667 interrupts per
| second.
| 
| That means instead our ISRs, take up closer to 1297.06667 clock
| cycles, or 0.00810667% of the processor time.
| 
-------------------------------------------------------------------*/

// ROS globals
ros::NodeHandle nodehandle;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
ros::Publisher odom_pub = nodehandle.advertise<nav_msgs::Odometry>("odom", 50);
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped t;

// Odometry globals
nav_msgs::Odometry o;
double x = 0.0;
double y = 0.0;
double theta = 0.0;

char base_link[] = "/base_link";
char odom[] = "/odom";


void setup() {
  nodehandle.initNode();
  nodehandle.subscribe(sub);

  nodehandle.advertise(odom_pub);
  odom_broadcaster.init(nodehandle);

  //  Set PWM pins as outputs
  pinMode(MOTOR_OUTPUT_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_OUTPUT_LEFT_A, OUTPUT);
  pinMode(MOTOR_OUTPUT_LEFT_B, OUTPUT);
  pinMode(MOTOR_OUTPUT_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_OUTPUT_RIGHT_A, OUTPUT);
  pinMode(MOTOR_OUTPUT_RIGHT_B, OUTPUT);
  
  //  Set encoder pins as inputs, and attach the corresponding ISRs
  pinMode(ENCODER_INPUT_LEFT_A, INPUT);
  pinMode(ENCODER_INPUT_LEFT_B, INPUT);
  pinMode(ENCODER_INPUT_RIGHT_A, INPUT);
  pinMode(ENCODER_INPUT_RIGHT_B, INPUT);
  // Here, I enable pins PCINT16-19, or PK0-3, or A8-11 to trigger ISR
  PCMSK2 |= 0b00001111;
  // Here, I enable the PCMSK2 defined pins to trigger ISR
  PCICR |= 0b00000100;
}


void loop() {
  // Make two 16 bit into one 32 bit
  encoder_count.data = ((unsigned long)(PULSE_COUNTER[0] << 16) | ((unsigned long) PULSE_COUNTER[1]));
  nodehandle.spinOnce();

  // Clear counters
  PULSE_COUNTER[0] = 0;
  PULSE_COUNTER[1] = 0;

  // Let data aggregate
  delay(LOOP_PERIOD);

  // START OF CRITICAL SECTION
  noInterrupts();
  //  Copy values (idx 1 : left wheel)
  CYCLE_PULSE_COUNTER[0] = PULSE_COUNTER[0];
  CYCLE_PULSE_COUNTER[1] = PULSE_COUNTER[1];
  //  Copy direction (HH-X_X_RDIR_LDIR)
  CYCLE_PULSE_DIRECTION_BYTE = PULSE_DIRECTION_BYTE;
  interrupts();
  // END OF CRITICAL SECTION

  // float VELOCITY_LEFT = WHEEL_RADIUS * (CYCLE_PULSE_COUNTER[0] / 128.00) * (2 * PI * FREQUENCY) * (1 / 120.00);
  bool BACK_LEFT = bitRead(CYCLE_PULSE_DIRECTION_BYTE, 0);
  double DIR_LEFT = BACK_LEFT ? 1.0 : -1.0;
  double DISTANCE_LEFT = DIR_LEFT * WHEEL_RADIUS * (CYCLE_PULSE_COUNTER[0] / 128.00) * (2 * PI) * (1 / 120.00);  

  bool BACK_RIGHT = bitRead(CYCLE_PULSE_DIRECTION_BYTE, 1);
  double DIR_RIGHT = BACK_RIGHT ? 1.0 : -1.0;
  double DISTANCE_RIGHT = DIR_RIGHT * WHEEL_RADIUS * (CYCLE_PULSE_COUNTER[1] / 128.00) * (2 * PI) * (1 / 120.00);  

  double AVERAGE_DISTANCE = 0.5 * (DISTANCE_RIGHT + DISTANCE_LEFT);

  ros::Time current_time = nodehandle.now();

  double delta_x = AVERAGE_DISTANCE * cos(theta);
  double delta_y = AVERAGE_DISTANCE * sin(theta);
  double delta_theta = 0.5 * (DISTANCE_RIGHT-DISTANCE_LEFT) / 0.15;

  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  geometry_msgs::Quaternion yaw = tf::createQuaternionFromYaw(theta);

  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.translation.x = x;
  t.transform.translation.y = y;

  t.transform.rotation = yaw;

  t.header.stamp = current_time;

  // SEND THE TRANSFORM
  odom_broadcaster.sendTransform(t);

  // NEXT THE ODOMETRY MESSAGE
  o.header.frame_id = odom;

  // SET THE POSITION
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = 0.0;
  o.pose.pose.orientation = yaw;

  o.child.frame_id = base_link;
  // SET THE VELOCITY
  //Here we multiply by frequency to get 1 / delta t, but this might be wildly inaccurate, it might be wiser to use actual time measurement rather than hoping everything will run perfectly on time.
  o.twist.twist.linear.x = delta_x * FREQUENCY;
  o.twist.twist.linear.y = delta_y * FREQUENCY;
  o.twist.twist.angular.z = delta_theta * FREQUENCY;

  o.header.stamp = current_time;
  //PUBLISH THE MESSAGE
  odom_pub.publish(o);
  
  int CYCLE_PULSE_COUNT_DELTA = CYCLE_PULSE_COUNTER[0] - CYCLE_PULSE_COUNTER[1];

  //  Attempt at speed adjusting so the wheels have about the same speed
  //  Logic here is that if the aboslute value of cycle pulse count delta is greater than 3 (meaning there is a significant deviation, should be calculated what an expectable pulse count delta should be), then adjust the speed accordingly
  if(abs(CYCLE_PULSE_COUNT_DELTA) > 3) {
    //  If delta is negative, right must be slowed or left must be sped up
    if(CYCLE_PULSE_COUNT_DELTA < 0) {
      if(GET_MOTOR_PWM_RIGHT() > 240) DEC_MOTOR_PWM_RIGHT();
      else if(GET_MOTOR_PWM_LEFT() < 255) INC_MOTOR_PWM_LEFT();
    }
    else if(CYCLE_PULSE_COUNT_DELTA > 0) {
      if(GET_MOTOR_PWM_LEFT() > 240) DEC_MOTOR_PWM_LEFT();
      else if(GET_MOTOR_PWM_RIGHT() < 255) DEC_MOTOR_PWM_RIGHT();
    }
  }
  
  DEBUG_LOG();
}