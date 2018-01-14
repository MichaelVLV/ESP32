/********************************** MPL3115A2 **********************************
 * @datashet:  http://www.nxp.com/docs/en/data-sheet/MPL3115A2.pdf
 * @appnote1:  http://cache.freescale.com/files/sensors/doc/app_note/AN4519.pdf
 * @appnote2:  https://cache.nxp.com/docs/en/application-note/AN4481.pdf
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#include "mpl3115a2.h"

//---------------------------I2C DEFINES
#define I2C_MASTER_NUM                I2C_NUM_1
#define I2C_MASTER_SCL_PIN            23
#define I2C_MASTER_SDA_PIN            4
#define I2C_MASTER_FREQ_HZ            100000
#define WRITE_BIT                     I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                      I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                  0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                 0x0            /*!< I2C master will not check ack from slave */
#define ACK_VAL                       0x0            /*!< I2C ack value */
#define NACK_VAL                      0x1          	 /*!< I2C nack value */
//-------------------------------------------
#define _MPL3115A2_ADDR               0x60
#define SLAVE_ADDR                    _MPL3115A2_ADDR        // this should be 7-bit value
//-------------------------------------------
uint8_t MPL3115_BUF[10];
//-------------------------------------------

// initialize & configure i2c peripheral
void i2c_master_initialize(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

uint8_t mpl3115a2_test(void)
{
//  uint8_t* whoiam;
//  MPL3115A2_RegRead(MPL3115A2_R_WHO_AM_I, whoiam);
//  printf("who_im:%x\n", whoiam);

//-------------------------------------------- read is done!
    memset(MPL3115_BUF, 0, sizeof(MPL3115_BUF));

    MPL3115A2_RegRead(MPL3115A2_R_WHO_AM_I, MPL3115_BUF);

    printf("MPL3115A2_R_WHO_AM_I:%02x\n", MPL3115_BUF[0]);

    disp_buf(MPL3115_BUF, 10);
//--------------------------------------------^read is done!

    if (MPL3115_BUF[0] != MPL3115A2_WHO_AM_I_ID)
    {
        return 0;
    }
    return 1;
}


void mpl3115a2_initialize(void)
{
  /*Select sensor, test responce*/
    if (mpl3115a2_test() == 0)
    {
        printf("init failed\n");
    }
    memset(MPL3115_BUF, 0, sizeof(MPL3115_BUF));

  /* check system mode*/
  MPL3115A2_RegRead(MPL3115A2_R_SYSMOD, MPL3115_BUF);
  printf("MPL3115A2_R_SYSMOD:%02x\n", MPL3115_BUF[0]); // 0 - standby, 1 - active

  if (MPL3115_BUF[0] != 0)
  {
      printf("mode error: still in active mode\n");
  }

  MPL3115A2_RegWrite(MPL3115A2_R_SYSMOD,0x38);

  /*set to barometer OSR= 128*/ // 0x26 (register)-> 0x38 (value)
  MPL3115A2_RegWrite(MPL3115A2_R_CTRL1,0x38); //0xB8 - altimeter, 0x38 - barometer

  /*Enable data flags*/
  MPL3115A2_RegWrite(MPL3115A2_R_PT_DATA_CFG,0x07);// 0x13 (register) -> 0x07 (value)

  /*Set active*/
  MPL3115A2_RegWrite(MPL3115A2_R_CTRL1,0x39);//0xB9 - for altimeter, 0x39 - for barometer

  /* check system mode*/
  MPL3115A2_RegRead(MPL3115A2_R_SYSMOD, MPL3115_BUF);
  printf("MPL3115A2_R_SYSMOD:%02x\n", MPL3115_BUF[0]); // 0 - standby, 1 - active

//  /*Set active*/
//  MPL3115Tx[0]  = MPL3115A2_R_CTRL1;
//  MPL3115Tx[1]  = 0x39;        //0xB9 - for altimeter, 0x39 - for barometer
//  ConditionHalI2CWrite(2, MPL3115Tx, WithStop);     // 0x26 (register) -> 0xB9 (value)
//
//  /* check system mode*/
//  MPL3115Tx[0] = MPL3115A2_R_SYSMOD;
//  ConditionHalI2CWrite(1, &MPL3115Tx[0], WithoutStop);
//  MPL_HalI2CRead(1, MPL3115Rx);
//
//  //MPL3115A2_AlternativeRead(MPL3115A2_R_SYSMOD, MPL3115Rx, 1);
//  if( (MPL3115Rx[0] & MPL3115A2_SYSMOD_SYSMOD) == 0 ) return MPL_ERR;
//
//  return MPL_OK;
}

uint8_t MPL3115A2_GetValues(float * pPress, int * pTemp)
{
  printf("get values\n");
  uint8_t data_status;
  memset(MPL3115_BUF, 0, sizeof(MPL3115_BUF));
  unsigned long Pressure = 0;
  int Temperature = 0;

  /*Read STATUS register*/
  for(char i = 0; i <= 100; i++)
  {
    MPL3115A2_RegRead(MPL3115A2_R_STATUS, MPL3115_BUF);
    //printf("MPL3115A2_R_STATUS:%02x\n", MPL3115_BUF[0]);

    data_status = MPL3115_BUF[0];
    if( (data_status & MPL3115A2_STATUS_DATA_READY) != 0)
    {
      printf("Data ready\n");
      break;   // when data is ready, break cycle
    }

    if( i == 100 )
    {
        printf("Error: i==100\n");
        return MPL_ERR;
    }
  }

  //-------------------------------------------------
  // Complex 6 byte read from sensor registers to buffer
  //-------------------------------------------------

//  for (uint8_t  i=0 ; i<=6; i++)
//  {
//  MPL3115A2_RegRead(0x00, MPL3115_BUF);
//  printf("0x%d:%02x\n", i, MPL3115_BUF[0]);
//  };

  //-------------------------------------------------
  // step by step reading (testing)
  //-------------------------------------------------
  MPL3115A2_RegRead(0x00, &MPL3115_BUF[0]);     // 0x00
  printf("MPL3115_BUF[0]:%x\n", MPL3115_BUF[0]);

  MPL3115A2_RegRead(MPL3115A2_R_PRESS_MSB,&MPL3115_BUF[1]);
  printf("MPL3115_BUF[1]:%x\n", MPL3115_BUF[1]);

  MPL3115A2_RegRead(MPL3115A2_R_PRESS_CSB,&MPL3115_BUF[2]);
  printf("MPL3115_BUF[2]:%x\n", MPL3115_BUF[2]);

  MPL3115A2_RegRead(MPL3115A2_R_PRESS_LSB,&MPL3115_BUF[3]);
  printf("MPL3115_BUF[3]:%x\n", MPL3115_BUF[3]);

  MPL3115A2_RegRead(MPL3115A2_R_TEMP_MSB, &MPL3115_BUF[4]);     // 0x04
  printf("MPL3115_BUF[4]:%x\n", MPL3115_BUF[4]);

  MPL3115A2_RegRead(MPL3115A2_R_TEMP_LSB, &MPL3115_BUF[5]);     // 0x05
  printf("MPL3115_BUF[5]:%x\n", MPL3115_BUF[5]);

  MPL3115A2_RegRead(0x06, &MPL3115_BUF[6]);     // test
  printf("MPL3115_BUF[6]:%x\n", MPL3115_BUF[6]);
  disp_buf(MPL3115_BUF, 10);
  //*********************************  PRESSURE ********************************

  Pressure |=  (MPL3115_BUF[2] << 8);
  printf("Pressure1:%lu\n", Pressure);
  Pressure &= ~0xFFFF0000;   // clear mask
  printf("Pressure2:%lu\n", Pressure);

  Pressure |= (MPL3115_BUF[1] * 65536);
  printf("Pressure3:%lu\n", Pressure);

  Pressure |= MPL3115_BUF[3];
  printf("Pressure4:%lu\n", Pressure);

//  Pressure = (MPL_data[2] << 8) | (MPL_data[1] * 65536) | MPL_data[3];
  Pressure >>= 6;   // pressure value in Pa
  printf("Pressure5:%lu\n", Pressure);

  unsigned long MPL_pressure = Pressure;
  printf("MPL_pressure:%lu\n", MPL_pressure);

  //******************************   ALTITUDE   ********************************
  //************************  (require spesial init)  **************************
  float tempcsb;
  float MPL_Altitude = 0;

  tempcsb = (MPL3115_BUF[3]>>4)/16.0;

  MPL_Altitude = (float)( (MPL3115_BUF[1] << 8) | MPL3115_BUF[2]) + tempcsb;

  //******************************  TEMPERATURE ********************************

  Temperature =( (MPL3115_BUF[4] << 8) |
                 (MPL3115_BUF[5] >> 4) );
  printf("Temperature:%i\n", Temperature);
  int MPL_temperature = Temperature / 256.0;  // value in C
  printf("MPL_temperature:%i\n", MPL_temperature);

  //******************************  TO EXERNAL  ********************************
  *pPress = MPL_pressure / 1000.0;  // value in kPa
  *pTemp = MPL_temperature;         // value in C

return MPL_OK;
}

void mpl3115a2_task(void *pvParameter)
{
    float press;
    int temper;

    while(1)
    {
      MPL3115A2_GetValues(&press , &temper);
      printf("pressure:%f , temperature: %i", press, temper);
      vTaskDelay(100);
    }
}

//main
void app_main()
{
    i2c_master_initialize(); // i2c init
//  mpl3115a2_test();
    mpl3115a2_initialize();	 // mpl init

    xTaskCreate(&mpl3115a2_task, "mpl3115a2_task", configMINIMAL_STACK_SIZE*5, NULL, 5, NULL);

}

/**
 * @brief Single byte WRITE to MPL3115A2
 *
 *   _______________________________________________________________________________________
 * M:| start | mpl_7bit_addr + wr_bit + ack(from_mpl) | register + ack  |data + ack  | stop |
 *   --------|----------------------------------------|-----------------|------------|------|
 * S:|-------|------------------------------|send ACK |-------|send ACK |---|send ACK|------|
 */
static esp_err_t MPL3115A2_RegWrite(uint8_t reg_wr, uint8_t val_wr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_wr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, val_wr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Single byte READ from MPL3115A2
 *  M:   - master device (i.e. esp32)
 *  S:   - slave device (i.e mpl3115)
 *  ST   - start
 *  r_ST - restart
 *  SP   - stop
 *
 *   _____________________________________________________________________________________________________
 * M:|ST| mpl_addr + WR_bit + ack |---|reg_addr + ack |---|r_ST| mpl_addr + RD_bit + ack |---|----|NAK|SP|
 *   |--|-------------------------|---|---------------|---|------------------------------|---|----|------|
 * S:|--|-------------------------|ACK|---------------|ACK|------------------------------|ACK|DATA|------|
 */
static esp_err_t MPL3115A2_RegRead(uint8_t reg_rd, uint8_t* data_rd)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_rd, ACK_CHECK_EN);
    i2c_master_start(cmd); // RESTART
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, 1, NACK_VAL); // 1 if byte, 7 or 8 in bits?
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000);
    i2c_cmd_link_delete(cmd);
    return ret;
}



/**
 * @brief todo:Multiple bytes WRITE to MPL3115A2
 *
 *   _______________________________________________________________________________________
 * M:| start | mpl_7bit_addr + wr_bit + ack(from_mpl) | register + ack  |data + ack  | stop |
 *   --------|----------------------------------------|-----------------|------------|------|
 * S:|-------|------------------------------|send ACK |-------|send ACK |---|send ACK|------|
 */
//static esp_err_t MPL3115A2_MultiRegWrite(uint8_t* reg_rw, uint8_t* val_rw)
//{
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, reg_rw, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, val_rw, ACK_CHECK_EN);
//    i2c_master_stop(cmd);
//    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}


