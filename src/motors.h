
#if defined (MOT_GM5208)
// GM5208
#define CONTROL_TYPE (MotionControlType::velocity)
#define SUPPLY_V (20)
#define DRIVER_V_LIMIT (20)
#define MOTOR_V_LIMIT (12)
#define MOTOR_I_LIMIT (100.0)
#define SENSOR_ALIGN_V (5)
#define MOTOR_PP (7)
#define MOTOR_RES (7.5)
#define MOTOR_K (90)
#define MOTOR_IND (0.002700)
#define VEL_P (0.05)
#define VEL_I (0.1)
#define VEL_D (0.0)
#define VEL_R (0.0)
#define VEL_L (100.0)
#define VEL_F (0.01)
#define ANG_P (40.0)
#define ANG_I (0.0)
#define ANG_D (0.0)
#define ANG_R (0.0)
#define ANG_L (0.0)
#define ANG_F (0.00)

#elif defined(MOT_2208)
#define CONTROL_TYPE (MotionControlType::velocity)
#define SUPPLY_V (20)
#define DRIVER_V_LIMIT (20)
#define MOTOR_V_LIMIT (12)
#define MOTOR_I_LIMIT (100.0)
#define SENSOR_ALIGN_V (4)
#define MOTOR_PP (7)
#define MOTOR_RES (7.5)
#define MOTOR_K (90)
#define MOTOR_IND (0.002700)
#define VEL_P (0.05)
#define VEL_I (0.1)
#define VEL_D (0.0)
#define VEL_R (0.0)
#define VEL_L (100.0)
#define VEL_F (0.01)
#define ANG_P (40.0)
#define ANG_I (0.0)
#define ANG_D (0.0)
#define ANG_R (0.0)
#define ANG_L (0.0)
#define ANG_F (0.00)

#elif defined(MOT_42BLS02)
// 42bls02
#define CONTROL_TYPE (MotionControlType::velocity)
#define SUPPLY_V (12)
#define DRIVER_V_LIMIT (12)
#define MOTOR_V_LIMIT (6)
#define MOTOR_I_LIMIT (100.0)
#define SENSOR_ALIGN_V (1)
#define MOTOR_PP (4)
#define MOTOR_RES (0.34)
#define MOTOR_K (250)
#define MOTOR_IND (0.000520)
#define VEL_P (0.05)
#define VEL_I (1.0)
#define VEL_D (0.0)
#define VEL_R (0.0)
#define VEL_L (10.0)
#define VEL_F (0.01)
#define ANG_P (40.0)
#define ANG_I (0.0)
#define ANG_D (0.0)
#define ANG_R (0.0)
#define ANG_L (0.0)
#define ANG_F (0.00)

#elif defined(MOT_D3536)
// D3536
#define CONTROL_TYPE (MotionControlType::velocity)
#define SUPPLY_V (12)
#define DRIVER_V_LIMIT (12)
#define MOTOR_V_LIMIT (3)
#define MOTOR_I_LIMIT (100.0)
#define SENSOR_ALIGN_V (0.5)
#define MOTOR_PP (7)
#define MOTOR_RES (0.1)
#define MOTOR_K (1450)
#define MOTOR_IND (0.000010)
#define VEL_P (0.05)
#define VEL_I (0.1)
#define VEL_D (0.0)
#define VEL_R (0.0)
#define VEL_L (100.0)
#define VEL_F (0.01)
#define ANG_P (40.0)
#define ANG_I (0.0)
#define ANG_D (0.0)
#define ANG_R (0.0)
#define ANG_L (0.0)
#define ANG_F (0.00)

#elif defined(MOT_RS2205)
// 2205
#define CONTROL_TYPE (MotionControlType::velocity)
#define SUPPLY_V (12)
#define DRIVER_V_LIMIT (12)
#define MOTOR_V_LIMIT (2)
#define MOTOR_I_LIMIT (100.0)
#define SENSOR_ALIGN_V (0.5)
#define MOTOR_PP (7)
#define MOTOR_RES (0.1)
#define MOTOR_K (2500)
#define MOTOR_IND (0.000017);
#define VEL_P (0.05)
#define VEL_I (0.1)
#define VEL_D (0.0)
#define VEL_R (0.0)
#define VEL_L (100.0)
#define VEL_F (0.01)
#define ANG_P (40.0)
#define ANG_I (0.0)
#define ANG_D (0.0)
#define ANG_R (0.0)
#define ANG_L (0.0)
#define ANG_F (0.00)

#endif

#define CUR_PQ (0.2)
#define CUR_IQ (0.5)
#define CUR_DQ (0.0)
#define CUR_RQ (0.0)
#define CUR_LQ (0.0)
#define CUR_FQ (0.01)
#define CUR_PD (0.2)
#define CUR_ID (0.5)
#define CUR_DD (0.0)
#define CUR_RD (0.0)
#define CUR_LD (0.0)
#define CUR_FD (0.01)


//BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_RES, MOTOR_K); // uni motor
//BLDCMotor motor = BLDCMotor(MOTOR_PP, MOTOR_RES); // uni motor
//BLDCMotor motor = BLDCMotor(2, 1, 300, 0.0033); // tool motor
//BLDCMotor motor = BLDCMotor(7, 11, 3000, 0.00001); // 2204-2300KV
//BLDCMotor motor = BLDCMotor(7); // 2204-2300KV
//BLDCMotor motor = BLDCMotor(7, 7.5, 90, 0.0018); // 2204-260KV
//BLDCMotor motor = BLDCMotor(7, 7.5, 90, 0.0018); // 2208
//BLDCMotor motor = BLDCMotor(7, 15, 22, 0.0018); // 2208
//BLDCMotor motor = BLDCMotor(4, 0.34, 250, 0.005); // 42bls02
//BLDCMotor motor = BLDCMotor(3, 0.1, 300); // big motor
//BLDCMotor motor = BLDCMotor(7, 0.1, 3000, 0.00001); // rs2205
//BLDCMotor motor = BLDCMotor(7, 0.1, 1400, 0.00001); // D3536
//BLDCMotor motor = BLDCMotor(6, 0.25); // tiny motor
//BLDCMotor motor = BLDCMotor(6, 2, 833); // XBOX DVD motor
//StepperMotor motor = StepperMotor(50, 2.6, 25, 0.001);
//BLDCMotor motor = BLDCMotor(15, 0.4, 28, 0.0004); // hub
//BLDCMotor motor = BLDCMotor(15, 0.4, 28); // hub

//BLDCDriver3PWM driver = BLDCDriver3PWM(PC0, PC1, PC2, PC3); // nucleo64-g474
//BLDCDriver3PWM driver = BLDCDriver3PWM(PA6, PA4, PB0, PB5); // nucleo32-g431
//BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);   // disco-STM32G431CB
//StepperDriver4PWM driver = StepperDriver4PWM(PC0, PC1, PC2, PC3); 

// inline current sensor instance
//InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A0, A2, _NC);
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 12.2, PA0, PA1, PB14);
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 12.2, PA0, PA1, PB14);
