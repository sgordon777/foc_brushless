#define MOT_2208

#include <Arduino.h>
#include <SimpleFOC.h>
#include "STM32HWEncoder.h"
#include <SimpleFOC.h>
#include "DRV8301.h"
#include "Wire.h"
#include "AS5600.h"
#include "core_cm4.h"
#include "pinouts.h"
#include "motors.h"
//#include "MagneticSensorMT6701SSI.h"

#define USE_DRV8301
//#define HIGHPERF_MEAS
#define MOD_FREQ (25000)
#define COMMANDER
//#define CURSENS
#define CLOSED_LOOP
#define MONITOR
//#define HALL
//#define ENCODER
//#define USE_SMOOTHSENSOR


#ifdef CLOSED_LOOP
STM32HWEncoder sensor = STM32HWEncoder(1024, ENC1_A, ENC1_B);  // nucleo32-g431
#endif // CLOSED_LOOP

#ifdef USE_DRV8301
DRV8301 gate_driver = DRV8301(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS, MOT1_EN, MOT1_FAULT); // MOSI, MISO, SCLK, CS, EN_GATE, FAULT
#endif //USE_DRV8301

// motor
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_RES, MOTOR_K, MOTOR_IND); // uni motor

// driver
//BLDCDriver6PWM driver = BLDCDriver6PWM(MOT1_AH, MOT1_BH, MOT1_CH, MOT1_AL, MOT1_BL, MOT1_CL, MOT1_EN);   // disco-STM32G431CB
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT1_AH, MOT1_BH, MOT1_CH, MOT1_EN);   // disco-STM32G431CB

#ifdef CURSENS
LowsideCurrentSense current_sense = LowsideCurrentSense(0.0005, 10, MOT1_SO1, MOT1_SO2, MOT1_SO3);
#endif

#ifdef HALL
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
#endif // HALL

#ifdef COMMANDER
// commander communication instance
Commander command = Commander(Serial);
void doTarget(char* cmd) {command.scalar(&motor.target, cmd);}
void doLimit(char* cmd) {command.scalar(&motor.voltage_limit, cmd);}
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doInduct(char* cmd) { command.scalar(&motor.phase_inductance, cmd); }
#endif // COMMANDER

#ifdef USE_SMOOTHSENSOR
SmoothingSensor smooth = SmoothingSensor(sensor, motor);
#endif // USE_SMOOTHSENSOR

#ifdef AS5600_CONFIG_REG
AS5600 as5600(&Wire);   //  use default Wire
#endif

char msgbuf[256];


void setup() {
  Serial.begin(921600); // WARNING: low value like 115200 cause distorted FOC


  // for timer analysis
  SimpleFOCDebug::enable(&Serial);
  //delay(5000);
  Serial.printf("enter setup...\n");

#ifdef CLOSED_LOOP
#ifdef HALL
  sensor.pullup = Pullup::USE_INTERN;
  sensor.enableInterrupts(doA, doB, doC);
#endif // HALL

  sensor.init();

#ifdef AS5600_CONFIG_REG
  Wire.setClock(1000000);
#endif

  // link the motor to the sensor

#ifdef USE_SMOOTHSENSOR
#ifdef HALL
  smooth.phase_correction = -_PI_6; // FOR HALL SENSOR
#endif  
  motor.linkSensor(&smooth);
#else // USE_SMOOTHSENSOR
  motor.linkSensor(&sensor);
#endif

#endif // CLOSED_LOOP

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = SUPPLY_V;
  driver.voltage_limit = DRIVER_V_LIMIT;
#if defined (MOD_FREQ)
  driver.pwm_frequency = MOD_FREQ;
#endif // MOD_FREQ
  driver.init();

#ifdef USE_DRV8301
  // configure the DRV8301
  gate_driver.begin(PWM_INPUT_MODE_3PWM, SHUNT_GAIN_10); 
  _delay(100);
  int reg1, reg2, reg3, reg4, fault;
  gate_driver.get_regs(&reg1, &reg2, &reg3, &reg4);
  fault = gate_driver.is_fault();
  sprintf(msgbuf, "DRV8301: fault=%x, STATREG1=0x%.4x, STATREG2=0x%.4x, CTRLREG1=0x%.4x, CTRLREG2=0x%.4x", fault, reg1, reg2, reg3, reg4);
  Serial.println(msgbuf);
#endif // USES_DRV8301

  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
#ifdef CURSENS
  current_sense.linkDriver(&driver);
  // current sense init and linking
  if (current_sense.init())  
    Serial.println("Current sense init success!");
  else{
    Serial.println("Current sense init failed!");
    return;
  }
  motor.linkCurrentSense(&current_sense);
#endif // CURSENS

#ifdef CLOSED_LOOP
  motor.PID_velocity.P = VEL_P; motor.PID_velocity.I = VEL_I; motor.PID_velocity.D = VEL_D; motor.PID_velocity.output_ramp = VEL_R; motor.PID_velocity.limit = VEL_L; motor.LPF_velocity.Tf = VEL_F;
  motor.P_angle.P = ANG_P;      motor.P_angle.I = ANG_I;      motor.P_angle.D = ANG_D;      motor.P_angle.output_ramp = ANG_R;      motor.P_angle.limit = ANG_L;      motor.LPF_angle.Tf = ANG_F;
#ifdef CURSENS
  motor.torque_controller = TorqueControlType::foc_current;
  // current q loop PID  1/40, 1/40
  motor.PID_current_q.P = CUR_PQ; motor.PID_current_q.I = CUR_IQ; motor.PID_current_q.D = CUR_DQ; motor.PID_current_q.output_ramp = CUR_RQ; motor.PID_current_q.limit = CUR_LQ; motor.LPF_current_q.Tf = CUR_FQ;
  // current d loop PID
  motor.PID_current_d.P = CUR_PD; motor.PID_current_d.I = CUR_ID; motor.PID_current_d.D = CUR_DD; motor.PID_current_d.output_ramp = CUR_RD; motor.PID_current_d.limit = CUR_LD; motor.LPF_current_d.Tf = CUR_FD;
#endif // CURSENS

#endif // CLOSED_LOOP

  // limts
  motor.voltage_sensor_align = SENSOR_ALIGN_V;
  //motor.controller = MotionControlType::torque;
  motor.controller = CONTROL_TYPE;
  // default voltage_power_supply
  motor.velocity_limit = VEL_L;
  motor.voltage_limit = MOTOR_V_LIMIT;
  motor.current_limit = MOTOR_I_LIMIT;

//  motor.motion_downsample = 10;
  // set the inital target value
  motor.target = 0;
#ifdef MONITOR
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable intially
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE;  // monitor target velocity and angle
#endif

  // initialise motor
  // moduleation mode
  //motor.foc_modulation = SpaceVectorPWM;
  motor.init();
#ifdef CLOSED_LOOP
  // align encoder and start FOC
  // for absolute encoder, set zero angle
  //motor.zero_electric_angle = 3.01;
  //motor.sensor_direction = CCW;
  motor.initFOC();
#endif


#ifdef COMMANDER
  // subscribe motor to the commander
  command.add('L', doLimit, "voltage limit");
  command.add('M',doMotor,"motor");
#endif

  Serial.printf("setup complete...\n");
  _delay(1000);

#ifdef HB_IO
  pinMode(HB_IO, OUTPUT);
#endif

#ifdef BUT_IO
  pinMode(BUT_IO, INPUT_PULLUP);
#endif
}

void loop() 
{
#ifdef HB_IO
digitalToggle(HB_IO);
#endif

#ifdef CLOSED_LOOP
  // iterative setting FOC phase voltage
  motor.loopFOC();
#endif // CLOSED_LOOP

  // iterative function setting the outter loop target
  motor.move();

#ifdef MONITOR
  // motor monitoring
  motor.monitor();
#endif // MONITOR

#ifdef COMMANDER
  // user communication
  command.run();
#endif // COMMANDER
}