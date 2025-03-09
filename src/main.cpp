#include <Arduino.h>
#include <SimpleFOC.h>
#include "STM32HWEncoder.h"
#include <SimpleFOC.h>
#include "Wire.h"
#include "AS5600.h"
#include "spi.h"
//#include "MagneticSensorMT6701SSI.h"

#define CONTROL_TYPE (MotionControlType::velocity)
#define SUPPLY_V (18)
#define DRIVER_V_LIMIT (18)
#define MOTOR_V_LIMIT (1)
#define MOTOR_VEL_LIMIT (25000)
#define SENSOR_ALIGN_V (0.2)

#define MOTOR_PP (7)
#define MOTOR_RES (0.33)
#define MOTOR_K (2000)
#define MOTOR_IND (0.00001)
//#define MOD_FREQ (64000)
#define COMMANDER
//#define CURSENS
#define CLOSED_LOOP
#define MONITOR
#define DEB_RETRIG_THRUS_INIT_US (500000)
#define DEB_RETRIG_THRS_SUS_US (100000)

#define ADC_THROT_INST ADC2
#define ADC_THROT_CHAN ADC_CHANNEL_4
#define THROT_CENT (1720.0)
#define THROT_MAX (2048.0)
#define THROT_DEADZONE (0.25)

//#define HALL
//#define ENCODER
//#define USE_SMOOTHSENSOR

#ifdef USE_SMOOTHSENSOR
#include "smoothsensor.h"
#endif


#define FOC_MAX(a, b) ( (a) > (b) ? (a) : (b) )
#define FOC_MIN(a, b) ( (a) < (b) ? (a) : (b) )

#define HB_IO (PA5)
//#define BUT_IO (PA5)
#ifdef CLOSED_LOOP
//Encoder sensor = Encoder(PA0, PA1, 100);
//STM32HWEncoder sensor = STM32HWEncoder(1024, PA_0, PA_1);  // nucleo64-g474, zero overhead encoder: nucleo64
STM32HWEncoder sensor = STM32HWEncoder(1024, PA_11_ALT2, PA_12_ALT1);  // nucleo32-g431
//STM32HWEncoder sensor = STM32HWEncoder(1024, PB_6, PB_7_ALT1);  // disco-STM32G431CB
//MagneticSensorSPI sensor = MagneticSensorSPI(my_AS5147_SPI, 10); // MOSI=11, MISO=12, CSK=13
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10); // MOSI=11, MISO=12, CSK=13. For some reason, AS5048_SPI doesnt work
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C); 
//SPISettings MT6701SSISettings_fast(15625000, MT6701_BITORDER, SPI_MODE2); // @suppress("Invalid arguments")
//MagneticSensorMT6701SSI sensor(10, MT6701SSISettings_fast); // MOSI=11, MISO=12, CSK=13
#endif // CLOSED_LOOP

#ifdef AS5600_CONFIG_REG
AS5600 as5600(&Wire);   //  use default Wire
#endif


// motor
BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_RES, MOTOR_K, MOTOR_IND); // uni motor
//BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_RES, MOTOR_K); // uni motor
//BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_RES); // uni motor
//BLDCMotor motor = BLDCMotor(7, 11, 3000, 0.00001); // 2204-2300KV
//BLDCMotor motor = BLDCMotor(7); // 2204-2300KV
//BLDCMotor motor = BLDCMotor(7, 7.5, 90, 0.0018); // 2204-260KV
//BLDCMotor motor = BLDCMotor(7, 7.5, 90, 0.0018); // 2208
//BLDCMotor motor = BLDCMotor(7, 15, 22, 0.0018); // 2208
//BLDCMotor motor = BLDCMotor(4, 0.34, 250, 0.0005); // 42bls02
//BLDCMotor motor = BLDCMotor(3, 0.1, 300); // big motor
//BLDCMotor motor = BLDCMotor(7, 0.1, 3000, 0.00001); // rs2205
//BLDCMotor motor = BLDCMotor(7, 0.1, 1400, 0.00001); // D3536
//BLDCMotor motor = BLDCMotor(6, 0.25); // tiny motor
//BLDCMotor motor = BLDCMotor(6, 2, 833); // XBOX DVD motor
//StepperMotor motor = StepperMotor(50, 2.6, 25, 0.001);
//BLDCMotor motor = BLDCMotor(15, 0.4, 28, 0.0004); // hub
//BLDCMotor motor = BLDCMotor(15, 0.4, 28); // hub

// driver
//BLDCDriver3PWM driver = BLDCDriver3PWM(PC0, PC1, PC2, PC3); // 8301/2 with simplified wiring
BLDCDriver3PWM driver = BLDCDriver3PWM(PA6, PA4, PB0, PB5); // nucleo32-g431
//BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);   // disco-STM32G431CB
//StepperDriver4PWM driver = StepperDriver4PWM(PC0, PC1, PC2, PC3);
#ifdef CURSENS
// inline current sensor instance
//InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A0, A2, _NC);
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 12.2, PA0, PA1, PB14);
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 12.2, PA0, PA1, PB14);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 12.2, PA0, PA1, _NC);

#endif
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 12.2, PA0, PA1, PB14);

#ifdef HALL
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
#endif // HALL

#ifdef COMMANDER
float g_myval1 = 6.283;
float g_myval2 = -6.283;
float g_scl = 0;
// commander communication instance
Commander command = Commander(Serial);
// void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doTarget(char* cmd) {command.scalar(&motor.target, cmd);}
void doLimit(char* cmd) {command.scalar(&motor.voltage_limit, cmd);}
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doInduct(char* cmd) { command.scalar(&motor.phase_inductance, cmd); }
void doMyval1(char* cmd) { command.scalar(&g_myval1, cmd); }
void doMyval2(char* cmd) { command.scalar(&g_myval2, cmd); }
void doscl(char* cmd) { command.scalar(&g_scl, cmd); }
#endif 

#ifdef USE_SMOOTHSENSOR
SmoothingSensor smooth = SmoothingSensor(sensor, motor);
#endif

int key_debounce(int butval, int retrig_thresh_init, int retrig_thresh_sus);
LowPassFilter lpf_throt(0.1);
ADC_HandleTypeDef hadc1;
#ifdef ADC_THROT_INST
static void MX_ADC_Init(void);
#endif 


void setup() {
  Serial.begin(921600); // WARNING: low value like 115200 cause distorted FOC
  // for timer analysis
  SimpleFOCDebug::enable(&Serial);
  //delay(5000);
  Serial.printf("enter setup...\n");

#ifdef AS5600_CONFIG_REG
  Wire.begin();
  uint8_t as5600_addr = 0;
  int control_reg_0, control_reg_1;
  as5600.begin();
  int b = as5600.isConnected();
  control_reg_0 = as5600.getConfigure();
  as5600.setConfigure(0x1f00);  // fast filter
  control_reg_1 = as5600.getConfigure();
  as5600_addr = as5600.getAddress();
  Serial.printf("AS5600 Connect: %d, addr=0x%x, confb4=0x%x, confaf=0x%x\n", b, as5600_addr, control_reg_0, control_reg_1);
#endif
 
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
  // velocity loop PID
  motor.PID_velocity.P = 0.033;
  motor.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 0.0;
  motor.PID_velocity.limit = 1000.0;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.01;
  // angle loop PID
  motor.P_angle.P = 40.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = MOTOR_VEL_LIMIT;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;

#ifdef CURSENS
  motor.torque_controller = TorqueControlType::foc_current;

  // current q loop PID  1/40, 1/40
  motor.PID_current_q.P = 0.2;
  motor.PID_current_q.I = 5;
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.output_ramp = 0;
  motor.PID_current_q.limit = 30;
  // Low pass filtering time constant 
  motor.LPF_current_q.Tf = 0.01;
  // current d loop PID
  motor.PID_current_d.P = 0.2;
  motor.PID_current_d.I = 5;
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.output_ramp = 0;
  motor.PID_current_d.limit = 30;
  // Low pass filtering time constant 
  motor.LPF_current_d.Tf = 0.01;
#endif // CURSENS


#endif // CLOSED_LOOP
  //motor.motion_downsample = 16;  //sssss try

  // limts
  motor.voltage_sensor_align = SENSOR_ALIGN_V;
  //motor.controller = MotionControlType::torque;
  motor.controller = CONTROL_TYPE;
  // default voltage_power_supply
  motor.velocity_limit = MOTOR_VEL_LIMIT;
  motor.voltage_limit = MOTOR_V_LIMIT;
  motor.current_limit = 1000.0;
  
//  motor.motion_downsample = 10;

  // set the inital target value
  motor.target = 0;
//  Serial.begin(115200); // WARNING: low value like 115200 cause distorted FOC
  // comment out if not needed
#ifdef MONITOR
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable intially
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE;  // monitor target velocity and angle
#endif

  //motor.foc_modulation = SpaceVectorPWM;
  // initialise motor
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
  command.add('T', doTarget, "target");  // ssss space
  command.add('L', doLimit, "voltage limit");
  command.add('I', doInduct, "phase_inductance");
  command.add('X', doMyval1, "delta_val1");
  command.add('Y', doMyval2, "delta_val2");
  command.add('Z', doscl, "sclval");
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

#ifdef ADC_THROT_INST
MX_ADC_Init();
#endif

}

void loop() 
{
  static int loopct = 0;

#ifdef HB_IO
  digitalWrite(HB_IO, 0);
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

#ifdef BUT_IO
static int flip = 0;
int rawval = digitalRead(BUT_IO);
  int butval = key_debounce(1-rawval, DEB_RETRIG_THRUS_INIT_US, DEB_RETRIG_THRS_SUS_US );  
  //Serial.printf("raw=%d, proc=%d\n", rawval, butval);
  if ( butval   )
  {
    if (flip == 0) motor.target = motor.target + g_myval1;
    else           motor.target = motor.target + g_myval2;
    flip = 1-flip;
  }
#endif // BUT_IO

#ifdef ADC_THROT_INST
 int adc0;
 HAL_ADC_Start(&hadc1);  // Start conversion
 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // Wait for conversion ~20us
 adc0 = HAL_ADC_GetValue(&hadc1);  // Get ADC value
 float adcf = lpf_throt(adc0);
 float throt_x = (adcf - THROT_CENT) / THROT_MAX;



 //Serial.printf("adc=%d, adcf=%d, cmdvel=%d\n", adc0, (int)adcf, (int)cmdvel*2048.0);
 if (g_scl != 0.0)
 {
    if (throt_x > 0)
      throt_x = FOC_MAX(throt_x - THROT_DEADZONE, 0);
    else
      throt_x = FOC_MIN(throt_x + THROT_DEADZONE, 0);
    motor.target = throt_x * g_scl;
 }
 #endif // ADC_THROT_INST

 ++loopct;


//  if (loopct % 1000)
//    float read_pot = _readADCVoltageInline(A_POTENTIOMETER, current_sense.params)*0.305810398;
#ifdef HB_IO
digitalWrite(HB_IO, 1);
#endif

}



#define RETRIG
int key_debounce(int butval, int retrig_thresh_init, int retrig_thresh_sus)
{
  static int wait_last_release = 0;
  int tu = micros();
  int output = 0;
  static int t_retrig = 1<<30;
  static int retrig_thresh = retrig_thresh_init;

  // signal button was relesed (no output will be generated until a button-release is detected)
  if (wait_last_release == 1 && butval == 0)
  {
    wait_last_release = 0;
    retrig_thresh = retrig_thresh_init;
  }
  // initial trigger    
  if ( butval && !wait_last_release )  
  {
    t_retrig = tu;
    wait_last_release = 1; // we cannot re-trigger until key is released
    output = 1;
  }

#ifdef RETRIG
  // retrigger
  int retrig_del = tu - t_retrig;
  if (wait_last_release == 1 && retrig_del > retrig_thresh)
  {
    t_retrig = tu;
    retrig_thresh = retrig_thresh_sus;
    output = 1;
  }
#endif
  return output;

}



#ifdef ADC_THROT_INST
static void MX_ADC_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC_THROT_INST;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
             Serial.printf("ERROR:HAL_ADC_Init\n");

    Error_Handler();
  }
  sConfig.Channel = ADC_THROT_CHAN;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
                     Serial.printf("ERROR:HAL_ADC_ConfigChannel\n");

    Error_Handler();
  }

}
#endif // ADC_THROT_INST