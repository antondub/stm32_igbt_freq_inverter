#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);

float target_angle = 0;

void Update_IT_callback(void)
{
   target_angle += 0.0175;
   motor.target = target_angle;
   digitalWrite (PC13, !digitalRead (PC13));
}

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  //SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  
  motor.foc_modulation = FOCModulationType::SinePWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle_openloop;
  //motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  //  maximal velocity of the position control
  motor.velocity_limit = 4;
  
  motor.current_limit = 1;   // [Amps]

  motor.P_angle.P = 20;
  motor.P_angle.output_ramp = 10000;
  
  motor.target = target_angle;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  _delay(1000);

  pinMode(PC13, OUTPUT);

  HardwareTimer *MyTim = new HardwareTimer(TIM2);
  MyTim->setOverflow(50, HERTZ_FORMAT); // 10 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();

}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();

}