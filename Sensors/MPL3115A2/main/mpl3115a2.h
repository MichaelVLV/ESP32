#ifndef __H_MPL3115A2_H__
#define __H_MPL3115A2_H__

/********************************** MPL3115A2 **********************************
 * @dataseet:  http://www.nxp.com/docs/en/data-sheet/MPL3115A2.pdf
 * @appnote1:  http://cache.freescale.com/files/sensors/doc/app_note/AN4519.pdf
 * @appnote2:  https://cache.nxp.com/docs/en/application-note/AN4481.pdf
 ******************************************************************************/
//--------------------- MPL3115A2 registers
#define MPL3115A2_ADDR                 0xC0    //0xC0     //(0xC0 >>1)
#define MPL3115A2_R_STATUS             0x00
#define MPL3115A2_STATUS_DATA_READY    0x08
#define MPL3115A2_STATUS_PRESS_READY   0x04
#define MPL3115A2_STATUS_TEMP_READY    0x02
#define MPL3115A2_R_PRESS_MSB          0x01
#define MPL3115A2_R_PRESS_CSB          0x02
#define MPL3115A2_R_PRESS_LSB          0x03
#define MPL3115A2_R_TEMP_MSB           0x04
#define MPL3115A2_R_TEMP_LSB           0x05
#define MPL3115A2_R_WHO_AM_I           0x0C
#define MPL3115A2_WHO_AM_I_ID          0xC4
#define MPL3115A2_R_SYSMOD             0x11
#define MPL3115A2_SYSMOD_SYSMOD        0x01
#define MPL3115A2_R_PT_DATA_CFG        0x13
#define MPL3115A2_CFG_DATA_EVENT_EN    0x04
#define MPL3115A2_CFG_PRESS_EVENT_EN   0x02
#define MPL3115A2_CFG_TEMP_EVENT_EN    0x01
#define MPL3115A2_R_CTRL1              0x26
#define MPL3115A2_CTRL1_OS2            0x20
#define MPL3115A2_CTRL1_OS1            0x10
#define MPL3115A2_CTRL1_OS0            0x08
#define MPL3115A2_CTRL1_START          0x02
#define MPL3115A2_CTRL1_ACTIVATE       0x01
//-----------------------------------------
#define MPL_ERR                        0x00
#define MPL_OK                         0x01
//-----------------------------------------

//---------------------- FUNCTION PROTOTYPES

void i2c_master_initialize(void);
uint8_t mpl3115a2_test(void);
static esp_err_t MPL3115A2_RegWrite(uint8_t reg_wr, uint8_t val_wr);
static esp_err_t MPL3115A2_RegRead(uint8_t reg_rd, uint8_t* data_rd);

#endif // __H_MPL3115A2_H__
