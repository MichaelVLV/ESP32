#include "includes/data.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

//---------------------------
#define FW_VERSION   "1.0.2"
//---------------------------


FlowMeterData_t FlowMeterData = { 0 };

void print_chip_inform(void) // MISC
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("FW_VERSION: %s\n", FW_VERSION);
    //esp_efuse_mac_get_custom(&chipid);
    esp_efuse_mac_get_default((uint8_t*)&FlowMeterData.adapter_ID);
    printf("ID:%02X_%02X_%02X_%02X_%02X_%02X\n",FlowMeterData.adapter_ID[0], FlowMeterData.adapter_ID[1], FlowMeterData.adapter_ID[2],
    			                                FlowMeterData.adapter_ID[3], FlowMeterData.adapter_ID[4], FlowMeterData.adapter_ID[5]);
}
