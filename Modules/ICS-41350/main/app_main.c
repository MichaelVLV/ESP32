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
 * BLUE   - SELECT
 * YELLOW - DATA
 * WHITE  - CLOCK
 * */



#define READ_LEN (16 * 1024)
#define ICS41350_PIN_DATA 2
#define ICS41350_PIN_CLK  4

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
        printf("%02x ", buf[i]);
        if ((i + 1) % 8 == 0) {
            printf("\n");
        }
    }
    printf("======\n");
}

void ICS41350_i2s_init()
{
	 int i2s_num = 0;
	 i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM,
        .sample_rate =  48000,
        .bits_per_sample = 16,
	    .communication_format = I2S_COMM_FORMAT_PCM | I2S_COMM_FORMAT_PCM_SHORT,
	    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = 2,
	    .dma_buf_len = 1024
	 };

	 i2s_pin_config_t pin_config = {
	    .ws_io_num   = ICS41350_PIN_CLK,
	    .data_in_num = ICS41350_PIN_DATA,
	 };

	 //install and start i2s driver
	 i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
	 i2s_set_pin(I2S_NUM_0, &pin_config);
}

void ICS41350_record_task (void* arg)
{
	//int i2s_read_len = READ_LEN;
    //char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    //i2s_read_bytes(I2S_NUM_0, (char*) BUFFER, READ_LEN, portMAX_DELAY);

	while (1)
	{
		i2s_read_bytes(I2S_NUM_0, (char*) BUFFER, READ_LEN, (1000 / portTICK_RATE_MS));
	    //i2s_pop_sample
	    ICS41350_disp_buf((uint8_t*) BUFFER, 64);
	    vTaskDelay(1000 / portTICK_RATE_MS);
	}

}

void app_main()
{
	ICS41350_i2s_init();
    xTaskCreate(ICS41350_record_task, "ICS41350_record_task", 1024 * 2, NULL, 5, NULL);
}


