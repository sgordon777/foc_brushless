#include "Arduino.h"

#if defined(ARDUINO_NUCLEO_G431KB)

// driver pionouts
  #define MOT1_A  PA6
  #define MOT1_B  PA4
  #define MOT1_C  PB0
  #define MOT1_EN PB5
  #define MOT2_A  PA8
  #define MOT2_B  PA9
  #define MOT2_C  PA10
  #define MOT2_EN PB6
  // encoder pinouts
  #define ENC1_A PA_11_ALT2
  #define ENC1_B PA_12_ALT1
  #define ENC2_A PA_0
  #define ENC2_B PA_1
  // I2C
  #define I2C1_SDA PB7
  #define I2C1_SCL PA15
  // UART
  #define UART_RX PB4
  #define UART_TX PB3
  // misc
  #define LED_IO PB8
  #define VBATTERY PA7
  #define BUT1 PA5
  // free
//  #define FREE3 PF0 ???
//  #define FREE4 PF1 ???
  #define UART_RX_DO_NOT_USE PA2
  #define UART_TX_DO_NOT_USE PA3

#elif defined (ARDUINO_NUCLEO_G474RE)
// driver pionouts
  #define MOT1_AH  PC0
  #define MOT1_BH  PC1
  #define MOT1_CH  PC2
  #define MOT1_AL _NC
  #define MOT1_BL _NC
  #define MOT1_CL _NC
  #define MOT1_EN PC3
  #define MOT1_FAULT _NC
  // conflict with ENC1
  #define MOT1_SO1 PA0
  #define MOT1_SO2 PA1
  #define MOT1_SO3 _NC

  #define MOT2_EN PB6
  // encoder pinouts
  #define ENC1_A PA_0
  #define ENC1_B PA_1
  #define ENC1_Z _NC

  // SPI
  #define SPI_SCK _NC
  #define SPI_MISO _NC
  #define SPI_MOSI _NC
  #define SPI_CS _NC

#elif defined (ARDUINO_DISCO_B_G431B_ESC1)
#elif defined (ARDUINO_NUCLEO_G431RB)

#elif defined (STM32F405xx)
// driver pionouts
  #define MOT1_AH  PA8
  #define MOT1_BH  PA9
  #define MOT1_CH  PA10
  #define MOT1_AL PB13
  #define MOT1_BL PB14
  #define MOT1_CL PB15
  #define MOT1_EN PB12
  #define MOT1_FAULT PD2
  #define MOT1_SO1 PC0
  #define MOT1_SO2 PC1
  #define MOT1_SO3 _NC
  #define MOT2_AH  PC6
  #define MOT2_BH  PC7
  #define MOT2_CH  PC8
  #define MOT2_AL  PA7
  #define MOT2_BL  PB0
  #define MOT2_CL  PB1
  
  #define MOT2_EN PB6
  // encoder pinouts
  #define ENC1_A PB_4
  #define ENC1_B PB_5
  #define ENC1_Z PC_9
  #define ENC2_A PB_6
  #define ENC2_B PB_7
  #define ENC2_Z PC_15
  // SPI
  #define SPI_SCK PC10
  #define SPI_MISO PC11
  #define SPI_MOSI PC12
  #define SPI_CS PC13


  // free
#endif
