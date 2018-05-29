#include <string.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "ble.h"
#include "data.h"

static const char remote_device_name[] = "Nordic_UART";
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;


static esp_bt_uuid_t remote_filter_service_a_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID_A,},
};

static esp_bt_uuid_t remote_filter_service_b_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID_B,},
};

static esp_bt_uuid_t remote_filter_service_c_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID_C,},
};


static esp_bt_uuid_t remote_filter_char_a_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID_A,},
};

static esp_bt_uuid_t remote_filter_char_b_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID_B,},
};

//uint8_t remote_notify_char_uuid_c[ESP_UUID_LEN_128] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};

static esp_bt_uuid_t remote_filter_char_c_uuid = {
    .len = ESP_UUID_LEN_128,
    //.uuid = {.uuid128 = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E},},
	.uuid = {.uuid128 = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E},},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
//    [PROFILE_A_APP_ID] = {
//        .gattc_cb = gattc_profile_a_event_handler,
//        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//    },
//    [PROFILE_B_APP_ID] = {
//        .gattc_cb = gattc_profile_b_event_handler,
//        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//    },
    [PROFILE_C_APP_ID] = {
        .gattc_cb = gattc_profile_c_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};


void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(BLE_TAG, "(A)REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(BLE_TAG, "(A)set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(A)REMOTE BDA:");
        esp_log_buffer_hex(BLE_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(BLE_TAG, "(A)config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(A)open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(A)open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG,"(A)config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_a_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_SEARCH_RES_EVT");
        esp_gatt_srvc_id_t *srvc_id =(esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16 && srvc_id->id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID_A) {
            ESP_LOGI(BLE_TAG, "(A)service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(BLE_TAG, "(A)UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(A)search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(A)esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(BLE_TAG, "(A)gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
															 remote_filter_char_a_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(A)esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                    	ESP_LOGI(BLE_TAG, "(A)char count: %d\n",count);
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_read_char(gattc_if,p_data->search_cmpl.conn_id,gl_profile_tab[PROFILE_A_APP_ID].char_handle,ESP_GATT_AUTH_REQ_NONE);
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(BLE_TAG, "(A)no char found");
            }
        }
         break;

    case ESP_GATTC_READ_CHAR_EVT: {
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_READ_CHAR_EVT");
        const char *char_pointer = (char*)p_data->read.value; //p_data->read.value: p_data is pointer of union type esp_ble_gattc_cb_param_t
                                                            //for accesing the read struc we need to use p_data->read.value sintaxis.
        //read member is a struct of type gattc_read_char_evt_param.
        //https://github.com/espressif/esp-idf/blob/a255622/components/bt/bluedroid/api/include/api/esp_gattc_api.h
        //https://stackoverflow.com/questions/16804650/accessing-c-union-members-via-pointers
        //https://www.tutorialspoint.com/cprogramming/c_unions.htm
        // p_data->read.value is a pointer to unit8_t type.
        //We need to convert from *unit8_t to *char https://stackoverflow.com/questions/12296984/c-uint8-datatype-conversion-to-const-char-datatype
        //When we have the pointer to char, we need to cast it int in order to read the numerical value of the iTag.
        //For example, p_data->read.value returns 'c' = 0x63 = 99

        int battery_percentage = (int)*char_pointer;
        ESP_LOGI(BLE_TAG, "(A)(p_data->read.value) Battery level: %d %%",battery_percentage); //https://stackoverflow.com/questions/1860159/how-to-escape-the-percent-sign-in-cs-printf

        ESP_LOGI(BLE_TAG, "(A)p_data->read.value_len %d",p_data->read.value_len);
        //int battery_percentage = atoi((const char*) p_data->read.value);
        //int battery_percentage = strtol((const char*)p_data->read.value, NULL, 10);

        //printf("Battery Level: %d ",battery_percentage);
        break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(A)REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(A)esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(BLE_TAG, "(A)malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(A)esp_ble_gattc_get_descr_by_char_handle error");
                    }

                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(A)esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(BLE_TAG, "(A)decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(BLE_TAG, p_data->notify.value, p_data->notify.value_len);
        //
        const char *char_pointer = (char*)p_data->notify.value; //p_data->read.value: p_data is pointer of union type esp_ble_gattc_cb_param_t
                                                            //for accesing the read struc we need to use p_data->read.value sintaxis.
        int battery_percent = (int)*char_pointer;
        ESP_LOGI(BLE_TAG, "(A)(p_data->notify.value) Battery level: %d %%",battery_percent); //https://stackoverflow.com/questions/1860159/how-to-escape-the-percent-sign-in-cs-printf

        ESP_LOGI(BLE_TAG, "(A)p_data->notify.value_len %d",p_data->notify.value_len);
        //
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(A)write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(A)write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(BLE_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(A)write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(A)write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(BLE_TAG, "(A)ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(BLE_TAG, "(B)REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(BLE_TAG, "(B)set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_B_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(B)REMOTE BDA:");
        esp_log_buffer_hex(BLE_TAG, gl_profile_tab[PROFILE_B_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(BLE_TAG, "(B)config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(B)open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(B)open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG,"(B)config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_b_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_SEARCH_RES_EVT");
        esp_gatt_srvc_id_t *srvc_id =(esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16 && srvc_id->id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID_B) {
            ESP_LOGI(BLE_TAG, "(B)service found");
            get_server = true;
            gl_profile_tab[PROFILE_B_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_B_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(BLE_TAG, "(B)UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(B)search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(B)esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(BLE_TAG, "(B)gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                             remote_filter_char_b_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(B)esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                    	ESP_LOGI(BLE_TAG, "(B)char count: %d\n",count);
                        gl_profile_tab[PROFILE_B_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_read_char(gattc_if,p_data->search_cmpl.conn_id,gl_profile_tab[PROFILE_B_APP_ID].char_handle,ESP_GATT_AUTH_REQ_NONE);
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_B_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(BLE_TAG, "(B)no char found");
            }
        }
         break;

    case ESP_GATTC_READ_CHAR_EVT: {
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_READ_CHAR_EVT");
        //const char *char_pointer = (char*)p_data->read.value; //p_data->read.value: p_data is pointer of union type esp_ble_gattc_cb_param_t
                                                            //for accesing the read struc we need to use p_data->read.value sintaxis.
        //read member is a struct of type gattc_read_char_evt_param.
        //https://github.com/espressif/esp-idf/blob/a255622/components/bt/bluedroid/api/include/api/esp_gattc_api.h
        //https://stackoverflow.com/questions/16804650/accessing-c-union-members-via-pointers
        //https://www.tutorialspoint.com/cprogramming/c_unions.htm
        // p_data->read.value is a pointer to unit8_t type.
        //We need to convert from *unit8_t to *char https://stackoverflow.com/questions/12296984/c-uint8-datatype-conversion-to-const-char-datatype
        //When we have the pointer to char, we need to cast it int in order to read the numerical value of the iTag.
        //For example, p_data->read.value returns 'c' = 0x63 = 99

        //int battery_percentage = (int)*char_pointer;
        //ESP_LOGI(BLE_TAG, "(B)(p_data->read.value) Battery level: %d %%",battery_percentage); //https://stackoverflow.com/questions/1860159/how-to-escape-the-percent-sign-in-cs-printf

        //ESP_LOGI(BLE_TAG, "(B)p_data->read.value_len %d",p_data->read.value_len);
        //int battery_percentage = atoi((const char*) p_data->read.value);
        //int battery_percentage = strtol((const char*)p_data->read.value, NULL, 10);

        //printf("Battery Level: %d ",battery_percentage);
        break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(B)REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(B)esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(BLE_TAG, "(B)malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(B)esp_ble_gattc_get_descr_by_char_handle error");
                    }

                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(B)esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(BLE_TAG, "(B)decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(BLE_TAG, p_data->notify.value, p_data->notify.value_len);
        memcpy(LockData.BLE_buf, p_data->notify.value, p_data->notify.value_len);
        LockData.BLE_len = p_data->notify.value_len;
        LockData.BLE_got_packet = true;

        //const char *char_pointer = (char*)p_data->notify.value; //p_data->read.value: p_data is pointer of union type esp_ble_gattc_cb_param_t
                                                            //for accesing the read struc we need to use p_data->read.value sintaxis.
        //int battery_percent = (int)*char_pointer;
        //ESP_LOGI(BLE_TAG, "(B)(p_data->notify.value) Battery level: %d %%",battery_percent); //https://stackoverflow.com/questions/1860159/how-to-escape-the-percent-sign-in-cs-printf

        ESP_LOGI(BLE_TAG, "(B)p_data->notify.value_len %d",p_data->notify.value_len);
        //
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(B)write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(B)write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_B_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(BLE_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(B)write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(B)write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(BLE_TAG, "(B)ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

void gattc_profile_c_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(BLE_TAG, "(C)REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(BLE_TAG, "(C)set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_C_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_C_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(C)REMOTE BDA:");
        esp_log_buffer_hex(BLE_TAG, gl_profile_tab[PROFILE_C_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(BLE_TAG, "(C)config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(C)open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(C)open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG,"(C)config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_SEARCH_RES_EVT");
        esp_gatt_srvc_id_t *srvc_id =(esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) { //&& srvc_id->id.uuid.uuid.uuid128 == REMOTE_SERVICE_UUID_C) {
            ESP_LOGI(BLE_TAG, "(C)service found");
            get_server = true;
            gl_profile_tab[PROFILE_C_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_C_APP_ID].service_end_handle = p_data->search_res.end_handle;
            for (uint8_t i = 0; i < 16; i++ )
            {
                ESP_LOGI(BLE_TAG, "(C)UUID128: %d [%X]", i,srvc_id->id.uuid.uuid.uuid128[i]);
            }
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(C)search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(C)esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(BLE_TAG, "(C)gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_C_APP_ID].service_end_handle,
															 remote_filter_char_c_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(C)esp_ble_gattc_get_char_by_uuid error, code:%X",status);
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                    	ESP_LOGI(BLE_TAG, "(C)char count: %d\n",count);
                        gl_profile_tab[PROFILE_C_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_read_char(gattc_if,p_data->search_cmpl.conn_id,gl_profile_tab[PROFILE_C_APP_ID].char_handle,ESP_GATT_AUTH_REQ_NONE);
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_C_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(BLE_TAG, "(C)no char found");
            }
        }
         break;

    case ESP_GATTC_READ_CHAR_EVT: {
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_READ_CHAR_EVT");
        const char *char_pointer = (char*)p_data->read.value; //p_data->read.value: p_data is pointer of union type esp_ble_gattc_cb_param_t
                                                            //for accesing the read struc we need to use p_data->read.value sintaxis.
        //read member is a struct of type gattc_read_char_evt_param.
        //https://github.com/espressif/esp-idf/blob/a255622/components/bt/bluedroid/api/include/api/esp_gattc_api.h
        //https://stackoverflow.com/questions/16804650/accessing-c-union-members-via-pointers
        //https://www.tutorialspoint.com/cprogramming/c_unions.htm
        // p_data->read.value is a pointer to unit8_t type.
        //We need to convert from *unit8_t to *char https://stackoverflow.com/questions/12296984/c-uint8-datatype-conversion-to-const-char-datatype
        //When we have the pointer to char, we need to cast it int in order to read the numerical value of the iTag.
        //For example, p_data->read.value returns 'c' = 0x63 = 99

        int battery_percentage = (int)*char_pointer;
        ESP_LOGI(BLE_TAG, "(C)(p_data->read.value) : %d %%",battery_percentage);
        ESP_LOGI(BLE_TAG, "(C)p_data->read.value_len %d",p_data->read.value_len);
        //int battery_percentage = atoi((const char*) p_data->read.value);
        //int battery_percentage = strtol((const char*)p_data->read.value, NULL, 10);

        //printf("Battery Level: %d ",battery_percentage);
        break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(C)REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_C_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(C)esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(BLE_TAG, "(C)malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(C)esp_ble_gattc_get_descr_by_char_handle error");
                    }

                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(C)esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(BLE_TAG, "(C)decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(BLE_TAG, p_data->notify.value, p_data->notify.value_len);

        memcpy(LockData.BLE_buf, p_data->notify.value, p_data->notify.value_len);
        LockData.BLE_len = p_data->notify.value_len;
        LockData.BLE_got_packet = true;

        //const char *char_pointer = (char*)p_data->notify.value;
        //ESP_LOGI(BLE_TAG, "(C)(p_data->notify.value) Nordic TX characteristic: %d",nordic_uart_data);
        ESP_LOGI(BLE_TAG, "(C)p_data->notify.value_len %d",p_data->notify.value_len);
        //
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(C)write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(C)write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_C_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_C_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(BLE_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(C)write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(C)write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(BLE_TAG, "(C)ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}
//

void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(BLE_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(BLE_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(BLE_TAG, "searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(BLE_TAG, adv_name, adv_name_len);
            ESP_LOGI(BLE_TAG, "\n");
            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(BLE_TAG, "searched device %s\n", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(BLE_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, true);
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_B_APP_ID].gattc_if, scan_result->scan_rst.bda, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(BLE_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(BLE_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(BLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
    	ESP_LOGI(BLE_TAG, "esp_gattc_cb:ESP_GATTC_REG_EVT");
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(BLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
        	ESP_LOGI(BLE_TAG, "gl_profile_tab[idx=%d]", idx);
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}
