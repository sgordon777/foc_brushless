#ifndef _DRV8301_H_
#define _DRV8301_H_

#include <Arduino.h>

#define DRV8301_STATUS_REG1 0x00
#define DRV8301_STATUS_REG2 0x01
#define DRV8301_CONTROL_REG1 0x02
#define DRV8301_CONTROL_REG2 0x03

#define GATE_CURRENT_MASK 0x0003
#define GATE_CURRENT_1P7A 0x0000
#define GATE_CURRENT_0P7A 0x0001
#define GATE_CURRENT_0P25A 0x0002
#define GATE_CURRENT_RESERVED GATE_CURRENT_MASK

#define GATE_RESET_MASK 0x0004
#define GATE_RESET_NORMAL_MODE 0x0000
#define GATE_RESET_RESET_MODE GATE_RESET_MASK

#define PWM_MODE_MASK 0x0008
#define PWM_MODE_6_PWM_INPUTS 0x0000
#define PWM_MODE_3_PWM_INPUTS PWM_MODE_MASK

#define OCP_MODE_MASK 0x0030
#define OCP_MODE_OC_LATCH_SHUTDOWN 0x0010
#define OCP_MODE_REPORT_ONLY 0x0020
#define OCP_MODE_DISABLE OCP_MODE_MASK

#define OC_ADJ_SET_MASK 0x07C0
#define OC_ADJ_SET(_oc_adj_) (((uint16_t)_oc_adj_ & 0x001F) << 6)

#define SHUNT_GAIN_MASK (0x000C)
#define SHUNT_GAIN_SHIFT (2)
#define SHUNT_GAIN_10 (0)
#define SHUNT_GAIN_20 (1)
#define SHUNT_GAIN_40 (2)
#define SHUNT_GAIN_80 (3)

enum DRV8301_PWM_INPUT_MODE
{
    PWM_INPUT_MODE_3PWM,
    PWM_INPUT_MODE_6PWM
};

/***********************************************************************************
* Gate driver DRV8301 class
***********************************************************************************/
class DRV8301
{
public:
    /**
     * DRV8301 class constructor
     * @param mosi  DRV8301 SPI data in pin
     * @param miso  DRV8301 SPI data out pin
     * @param sclk  DRV8301 SPI clock pin
     * @param cs  DRV8301 SPI chip select pin
     * @param en_gate   DRV8301 enable(reset) pin
     * @param fault  DRV8301 fault pin (pull_up)
     */
    DRV8301(int mosi, int miso, int sclk, int cs, int en_gate, int fault);

    /**
    * Initialize pin and initialize DRV8301
    * @param pwm_mode DRV8301 PWM signal input mode
    */
    void begin(DRV8301_PWM_INPUT_MODE pwm_mode, int shunt_gain);

    /** Reset DRV8301 */
    void reset(void);

    /**
    * Detect if DRV8301 has fault occurred
    * @retval   0:no faults 1:has faults
    */
    int is_fault(void);

    /**
    * Read DRV8301's fault value
    * @retval DRV8301's fault value
    */
    int read_fault(void);

    /**
    * Get DRV8301's chip id
    * @retval   chip id
    */
    int get_id(void);


    int get_regs(int *reg1, int *reg2, int*reg3, int*reg4);
    int get_reg1(int *reg1);

private:
    // SPI pins defineder
    int drv8301_mosi_pin; //!< SPI data output pin
    int drv8301_miso_pin; //!< SPI data input pin
    int drv8301_sclk_pin; //!< SPI bit clock pin
    // SPI functions
    void spi_delay(void);
    /** SPI exchange 16bit value */
    uint16_t spi_transfer(uint16_t txdata);

    // DRV8301 pins defineder
    int drv8301_cs_pin;
    int drv8301_en_gate_pin;
    int drv8301_fault_pin;

    uint16_t drv8301_ctrl_reg1_val;
    uint16_t drv8301_ctrl_reg2_val;
    // DRV8301 functions
    int drv8301_read_reg(uint16_t reg);
    void drv8301_write_reg(uint16_t reg, uint16_t data);
};


/***********************************************************************************
* Gate driver DRV8302 class
***********************************************************************************/

class DRV8302
{
public:
    /**
     * DRV8302 class constructor
     * @param fault  DRV8302 fault pin (pull_up)
     * @param m_pwm PWM mode: 0=6, 1=3
     * @param m_oc overcurrent mode: 0=CycleByCycle, 1=shutdown
     * @param gain 0=10V/V, 1=40V/V
     * @param oc_adj trip point for overcurrent, set high
     * @param en_gate DRV8302 enable(reset) pin
     */
    DRV8302(int fault, int m_pwm, int m_oc, int gain, int oc_adj, int en_gate);

    /**
    * Initialize pin and initialize DRV8302
    * @param pwm_mode DRV8302 PWM signal input mode
    * @param shunt_gain DRV8302 PWM signal input mode
    */
    void begin(DRV8301_PWM_INPUT_MODE pwm_mode, int shunt_gain);

    /** Reset DRV8302 */
    void reset(void);

    /**
    * Detect if DRV8302 has fault occurred
    * @retval   0:no faults 1:has faults
    */
    int is_fault(void);

private:
    int drv8302_fault_pin; //!< SPI data output pin
    int drv8302_m_pwm_pin; //!< SPI data output pin
    int drv8302_m_oc_pin; //!< SPI data output pin
    int drv8302_gain_pin; //!< SPI data output pin
    int drv8302_oc_adj_pin; //!< SPI data output pin
    int drv8302_en_gate_pin; //!< SPI data output pin


};

#endif // _DRV8302_H_
