#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "audio_example_file.h"
#include "esp_adc_cal.h"

/*
 * BLACK  - GND
 * RED    - VCC
 * BLUE   - SELECT // ws_io_num (?)
 * YELLOW - DATA   // data_in_num
 * WHITE  - CLOCK  // bck_io_num (?)
 * */



#define READ_LEN (32 * 1024)
#define ICS41350_PIN_DATA 2
#define ICS41350_PIN_CLK  4
#define ICS41350_PIN_SEL  15

struct WAVHEADER
{
    char chunkId[4];             // 'RIFF'
    unsigned long chunkSize;     // file size - 8
    char format[4];              // 'WAVE'
    char subchunk1Id[4];         // 'fmt'
    uint16_t audioFormat;        //  0x10 linear PCM
    uint16_t numChannels;
    unsigned long byteRate;      // Byte/sec = 44100x2x1 = 88200
    uint16_t blockAlign;         // 0x0200 = mono 16 bit
    uint16_t bitsPerSample;      // 0x1000 = 16 bit
    char subchunk2Id[4];         // 'data'
    unsigned long subchunk2Size; // data size
};

uint8_t BUFFER[READ_LEN] = {0};

void ICS41350_disp_buf(uint8_t* buf, int length)
{
    printf("======\n");
    for (int i = 0; i < length; i++) {
        printf("%02X", buf[i]);
//        if ((i + 1) % 8 == 0) {
//            printf("\n");
//        }
    }
    printf("======\n");
}

void ICS41350_i2s_init()
{
	 int i2s_num = 0;
	 i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX| I2S_MODE_PDM ,
        .sample_rate =  44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .communication_format = I2S_COMM_FORMAT_I2S,//I2S_COMM_FORMAT_PCM | I2S_COMM_FORMAT_PCM_SHORT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,//I2S_CHANNEL_FMT_RIGHT_LEFT,//I2S_CHANNEL_FMT_ONLY_RIGHT,
		//.use_apll = true,
		//.fixed_mclk = 48000,
	    //.intr_alloc_flags = 0,
	    .dma_buf_count = 2,
	    .dma_buf_len = 128
	 };

	 i2s_pin_config_t pin_config = {
	    .ws_io_num   = ICS41350_PIN_CLK,
	    .data_in_num = ICS41350_PIN_DATA,
		//.bck_io_num  = ICS41350_PIN_CLK,
	 };

	 //install and start i2s driver
	 i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
	 i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
	 i2s_set_pin(I2S_NUM_0, &pin_config);
}

void ICS41350_record_task (void* arg)
{
	//int i2s_read_len = READ_LEN;
    //char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    //i2s_read_bytes(I2S_NUM_0, (char*) BUFFER, READ_LEN, portMAX_DELAY);
    int len = 0;
	while (1)
	{
		len = i2s_read_bytes(I2S_NUM_0, (char*) BUFFER, READ_LEN, (1000 / portTICK_RATE_MS));
	    //i2s_pop_sample
		ESP_LOGI("I2S", "READ len:%d", len);
	    vTaskDelay(1000 / portTICK_RATE_MS);
	    ICS41350_disp_buf((uint8_t*) BUFFER, READ_LEN);
	    vTaskDelay(1000 / portTICK_RATE_MS);

	    ESP_LOGI("I2S", "Replay");
	    len = i2s_write_bytes(I2S_NUM_0, (char*) BUFFER, READ_LEN, portMAX_DELAY);
		ESP_LOGI("I2S", "WRITE len:%d", len);
	    vTaskDelay(1000 / portTICK_RATE_MS);
	    break;
//		//4. Play an example audio file(file format: 8bit/16khz/single channel)
//		printf("Playing file example: \n");
//		len = i2s_write_bytes(I2S_NUM_0, (const char*) audio_table, sizeof(audio_table), portMAX_DELAY);
//		ESP_LOGI("I2S", "WRITE len:%d", len);
//		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void app_main()
{
	ICS41350_i2s_init();
    xTaskCreate(ICS41350_record_task, "ICS41350_record_task", 1024 * 2, NULL, 5, NULL);
}



//---------------------------------------------------------------------------------------------------------------------------------------------
//#include <stdio.h>
//#include <string.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_spi_flash.h"
//#include "esp_err.h"
//#include "esp_log.h"
//#include "esp_partition.h"
//#include "driver/i2s.h"
//#include "driver/adc.h"
//#include "audio_example_file.h"
//#include "esp_adc_cal.h"
//
//static const char* TAG = "ad/da";
//#define V_REF   1100
//#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_7)
//
//#define ICS41350_PIN_DATA 2
//#define ICS41350_PIN_CLK  4
//
//#define PARTITION_NAME   "storage"
//
///*---------------------------------------------------------------
//                            EXAMPLE CONFIG
//---------------------------------------------------------------*/
////enable record sound and save in flash
//#define RECORD_IN_FLASH_EN        (1)
////enable replay recorded sound in flash
//#define REPLAY_FROM_FLASH_EN      (1)
//
////i2s number
//#define EXAMPLE_I2S_NUM           (0)
////i2s sample rate
//#define EXAMPLE_I2S_SAMPLE_RATE   (48000)//(16000)
////i2s data bits
//#define EXAMPLE_I2S_SAMPLE_BITS   (I2S_BITS_PER_SAMPLE_16BIT)
////enable display buffer for debug
//#define EXAMPLE_I2S_BUF_DEBUG     (1)
////I2S read buffer length
//#define EXAMPLE_I2S_READ_LEN      (16 * 1024)
////I2S data format
//#define EXAMPLE_I2S_FORMAT        (I2S_CHANNEL_FMT_RIGHT_LEFT)//(I2S_CHANNEL_FMT_ONLY_RIGHT)
////I2S channel number
//#define EXAMPLE_I2S_CHANNEL_NUM   ((EXAMPLE_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
////I2S built-in ADC unit
//#define I2S_ADC_UNIT              ADC_UNIT_1
////I2S built-in ADC channel
//#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0
//
////flash record size, for recording 5 seconds' data
//#define FLASH_RECORD_SIZE         (EXAMPLE_I2S_CHANNEL_NUM * EXAMPLE_I2S_SAMPLE_RATE * EXAMPLE_I2S_SAMPLE_BITS / 8 * 5)
//#define FLASH_ERASE_SIZE          (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
////sector size of flash
//#define FLASH_SECTOR_SIZE         (0x1000)
////flash read / write address
//#define FLASH_ADDR                (0x200000)
//
//
///**
// * @brief I2S ADC/DAC mode init.
// */
//void example_i2s_init()
//{
//	 int i2s_num = EXAMPLE_I2S_NUM;
//	 i2s_config_t i2s_config = {
//        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_PDM,
//        .sample_rate =  EXAMPLE_I2S_SAMPLE_RATE,
//        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS,
//	    .communication_format = I2S_COMM_FORMAT_PCM | I2S_COMM_FORMAT_PCM_LONG, //I2S_COMM_FORMAT_I2S_MSB,
//	    .channel_format = EXAMPLE_I2S_FORMAT,
//	    .intr_alloc_flags = 0,
//	    .dma_buf_count = 3,
//	    .dma_buf_len = 32,
//	    .use_apll = 0
//	 };
//
//	 i2s_pin_config_t pin_config = {
//	    .ws_io_num   = ICS41350_PIN_CLK,
//	    .data_in_num = ICS41350_PIN_DATA,
//	 };
//
//	 //install and start i2s driver
//	 i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
//	 //init DAC pad
//	 i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
//	 i2s_set_pin(EXAMPLE_I2S_NUM, &pin_config);
//}
//
///*
// * @brief erase flash for recording
// */
//void example_erase_flash()
//{
//#if RECORD_IN_FLASH_EN
//    printf("Erasing flash \n");
//    const esp_partition_t *data_partition = NULL;
//    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
//            ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
//    if (data_partition != NULL) {
//        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
//    }
//    printf("Erase size: %d Bytes\n", FLASH_ERASE_SIZE);
//    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, FLASH_ERASE_SIZE));
//#else
//    printf("Skip flash erasing...\n");
//#endif
//}
//
///**
// * @brief debug buffer data
// */
//void example_disp_buf(uint8_t* buf, int length)
//{
//#if EXAMPLE_I2S_BUF_DEBUG
//    printf("======\n");
//    for (int i = 0; i < length; i++) {
//        printf("%02x ", buf[i]);
//        if ((i + 1) % 8 == 0) {
//            printf("\n");
//        }
//    }
//    printf("======\n");
//#endif
//}
//
///**
// * @brief Reset i2s clock and mode
// */
//void example_reset_play_mode()
//{
//    i2s_set_clk(EXAMPLE_I2S_NUM, EXAMPLE_I2S_SAMPLE_RATE, EXAMPLE_I2S_SAMPLE_BITS, EXAMPLE_I2S_CHANNEL_NUM);
//}
//
///**
// * @brief Set i2s clock for example audio file
// */
//void example_set_file_play_mode()
//{
//    i2s_set_clk(EXAMPLE_I2S_NUM, 16000, EXAMPLE_I2S_SAMPLE_BITS, 1);
//}
//
///**
// * @brief Scale data to 16bit/32bit for I2S DMA output.
// *        DAC can only output 8bit data value.
// *        I2S DMA will still send 16 bit or 32bit data, the highest 8bit contains DAC data.
// */
//int example_i2s_dac_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len)
//{
//    uint32_t j = 0;
//#if (EXAMPLE_I2S_SAMPLE_BITS == 16)
//    for (int i = 0; i < len; i++) {
//        d_buff[j++] = 0;
//        d_buff[j++] = s_buff[i];
//    }
//    return (len * 2);
//#else
//    for (int i = 0; i < len; i++) {
//        d_buff[j++] = 0;
//        d_buff[j++] = 0;
//        d_buff[j++] = 0;
//        d_buff[j++] = s_buff[i];
//    }
//    return (len * 4);
//#endif
//}
//
///**
// * @brief Scale data to 8bit for data from ADC.
// *        Data from ADC are 12bit width by default.
// *        DAC can only output 8 bit data.
// *        Scale each 12bit ADC data to 8bit DAC data.
// */
//void example_i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
//{
//    uint32_t j = 0;
//    uint32_t dac_value = 0;
//#if (EXAMPLE_I2S_SAMPLE_BITS == 16)
//    for (int i = 0; i < len; i += 2) {
//        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
//        d_buff[j++] = 0;
//        d_buff[j++] = dac_value * 256 / 4096;
//    }
//#else
//    for (int i = 0; i < len; i += 4) {
//        dac_value = ((((uint16_t)(s_buff[i + 3] & 0xf) << 8) | ((s_buff[i + 2]))));
//        d_buff[j++] = 0;
//        d_buff[j++] = 0;
//        d_buff[j++] = 0;
//        d_buff[j++] = dac_value * 256 / 4096;
//    }
//#endif
//}
//
///**
// * @brief I2S ADC/DAC example
// *        1. Erase flash
// *        2. Record audio from ADC and save in flash
// *        3. Read flash and replay the sound via DAC
// *        4. Play an example audio file(file format: 8bit/8khz/single channel)
// *        5. Loop back to step 3
// */
//void example_i2s_adc_dac(void*arg)
//{
//    const esp_partition_t *data_partition = NULL;
//    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
//            ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
//    if (data_partition != NULL) {
//        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
//    } else {
//        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
//        vTaskDelete(NULL);
//    }
//    //1. Erase flash
//    example_erase_flash();
//    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
//    int flash_wr_size = 0;
//
//    //2. Record audio from ADC and save in flash
//#if RECORD_IN_FLASH_EN
//    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
//    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
//    //i2s_adc_enable(EXAMPLE_I2S_NUM);
//    while (flash_wr_size < FLASH_RECORD_SIZE) {
//        //read data from I2S bus, in this case, from ADC.
//        i2s_read_bytes(EXAMPLE_I2S_NUM, (char*) i2s_read_buff, i2s_read_len, portMAX_DELAY);
//        example_disp_buf((uint8_t*) i2s_read_buff, 64);
//        //save original data from I2S(ADC) into flash.
//        esp_partition_write(data_partition, flash_wr_size, i2s_read_buff, i2s_read_len);
//        flash_wr_size += i2s_read_len;
//        ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
//    }
//    //i2s_adc_disable(EXAMPLE_I2S_NUM);
//    free(i2s_read_buff);
//    i2s_read_buff = NULL;
//    free(flash_write_buff);
//    flash_write_buff = NULL;
//#endif
//
//    uint8_t* flash_read_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
//    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
//    while (1) {
//
//        //3. Read flash and replay the sound via DAC
//#if REPLAY_FROM_FLASH_EN
//        for (int rd_offset = 0; rd_offset < flash_wr_size; rd_offset += FLASH_SECTOR_SIZE) {
//            //read I2S(ADC) original data from flash
//            esp_partition_read(data_partition, rd_offset, flash_read_buff, FLASH_SECTOR_SIZE);
//            //process data and scale to 8bit for I2S DAC.
//            example_i2s_adc_data_scale(i2s_write_buff, flash_read_buff, FLASH_SECTOR_SIZE);
//            //send data
//            i2s_write_bytes(EXAMPLE_I2S_NUM, (char*) i2s_write_buff, FLASH_SECTOR_SIZE, portMAX_DELAY);
//            printf("playing: %d %%\n", rd_offset * 100 / flash_wr_size);
//        }
//#endif
//
//        //4. Play an example audio file(file format: 8bit/16khz/single channel)
//        printf("Playing file example: \n");
//        int offset = 0;
//        int tot_size = sizeof(audio_table);
//        example_set_file_play_mode();
//        while (offset < tot_size) {
//            int play_len = ((tot_size - offset) > (4 * 1024)) ? (4 * 1024) : (tot_size - offset);
//            int i2s_wr_len = example_i2s_dac_data_scale(i2s_write_buff, (uint8_t*)(audio_table + offset), play_len);
//            i2s_write_bytes(EXAMPLE_I2S_NUM, (const char*) i2s_write_buff, i2s_wr_len, portMAX_DELAY);
//            offset += play_len;
//            example_disp_buf((uint8_t*) i2s_write_buff, 32);
//        }
//        vTaskDelay(100 / portTICK_PERIOD_MS);
//        example_reset_play_mode();
//    }
//    free(flash_read_buff);
//    free(i2s_write_buff);
//    vTaskDelete(NULL);
//}
//
//void adc_read_task(void* arg)
//{
//    adc1_config_width(ADC_WIDTH_12Bit);
//    adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_11db);
//    esp_adc_cal_characteristics_t characteristics;
//    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);
//    while(1) {
//        uint32_t voltage;
//        esp_adc_cal_get_voltage(ADC1_TEST_CHANNEL, &characteristics, &voltage);
//        ESP_LOGI(TAG, "%d mV", voltage);
//        vTaskDelay(200 / portTICK_RATE_MS);
//    }
//}
//
//esp_err_t app_main()
//{
//    example_i2s_init();
//    esp_log_level_set("I2S", ESP_LOG_INFO);
//    xTaskCreate(example_i2s_adc_dac, "example_i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
//    //xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);
//    return ESP_OK;
//}
//
//
