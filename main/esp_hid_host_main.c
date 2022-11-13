/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "esp_hidh.h"
#include "esp_hidh_api.h"
#include "esp_hid_gap.h"

#define BUF_SIZE 3

//#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14


static const char *TAG = "ESP_HIDH_DEMO";
static bool set_lights = false;
static bool calibrating = false;
static uint16_t calibration[3][4] = {
  {0xffff, 0xffff, 0xffff, 0xffff}, //  0kg
  {0xffff, 0xffff, 0xffff, 0xffff}, // 17kg
  {0xffff, 0xffff, 0xffff, 0xffff}  // 34kg
};

static uint8_t send_buf[3];
static uint8_t recv_buf[3];


static spi_slave_transaction_t spi_trans;

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST HSPI_HOST
#else
#define RCV_HOST SPI2_HOST
#endif

/* void my_post_setup_cb(spi_slave_transaction_t *trans) { */
/*   gpio_set_level(GPIO_HANDSHAKE, 1); */
/* } */

/* void my_post_trans_cb(spi_slave_transaction_t *trans) { */
/*   gpio_set_level(GPIO_HANDSHAKE, 0); */
/* } */

esp_hidh_dev_t* balance_dev = NULL;
size_t map_index = 0;

#define TOP_RIGHT 0
#define BOTTOM_RIGHT 1
#define TOP_LEFT 2
#define BOTTOM_LEFT 3

float calibrated_value(int pos, uint16_t* p_value) {
  uint16_t value = *p_value;
  value = (value >> 8) | (value << 8);
  if(value < calibration[0][pos]) {
    return 0.0f;
  }
  else if(value < calibration[1][pos]) {
    return 17.0f * ((value - calibration[0][pos]) /
		    ((float)(calibration[1][pos] -
			     calibration[0][pos])));
  }
  // else {
  return 17.0f + 17.0f * ((value - calibration[1][pos]) /
			  ((float)(calibration[2][pos] -
				   calibration[1][pos])));
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
    
    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
      fprintf(stderr, "\tOpen Event\n");
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
	    //xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
      
      switch(param->input.report_id) {
      case 33:; // Calibration
	uint8_t length = (param->input.data[2] >> 4) + 1;
	if(calibrating) {
	  if(length == 16) { // packet 1
	    for(int i = 0; i < 4; i++) {
	      calibration[0][i] = *(unsigned short*)(param->input.data + 5 + i*2);
	      calibration[1][i] = *(unsigned short*)(param->input.data + 13 + i*2);
	      calibration[0][i] = (calibration[0][i] >> 8) | (calibration[0][i] << 8);
	      calibration[1][i] = (calibration[1][i] >> 8) | (calibration[1][i] << 8);
	      printf("cal 0kg %04X\n", calibration[0][i]);
	      printf("cal 17kg %04X\n", calibration[1][i]);
	    }
	  }
	  else if(length < 16) { // packet 2
	    for(int i = 0; i < 4; i++) {
	      calibration[2][i] = *(unsigned short*)(param->input.data + 5 + i*2);
	      calibration[2][i] = (calibration[2][i] >> 8) | (calibration[2][i] << 8);
	      printf("cal 34kg %04X\n", calibration[2][i]);
	    }
	    calibrating = false;
	    
	    uint8_t data[] = {
	      0x04, 0xA4, 0x00, 0x00, 0x00, 0x0b
	    };
	    esp_hidh_dev_output_set(param->input.dev, param->input.map_index,
				    0x17, data, 6);
	    if(balance_dev == NULL) {
	      balance_dev = param->input.dev;
	      map_index = param->input.map_index;
	    }
	  }
	}
	else if(length == 0x0b) { // Balance Report
	  float top_left = calibrated_value(TOP_LEFT, (param->input.data + 9));
	  float top_right = calibrated_value(TOP_RIGHT, (param->input.data + 5));
	  float bottom_left = calibrated_value(BOTTOM_LEFT, (param->input.data + 11)) - 1.0f;
	  if(bottom_left < 0.0f) bottom_left = 0.0f;
	  float bottom_right = calibrated_value(BOTTOM_RIGHT, (param->input.data + 7));
	  float total_weight =
	    top_left + top_right + bottom_left + bottom_right;

	  float x = 0.0f;
	  float y = 0.0f;
	  if(total_weight > 2.0f) {
	    x += -(top_left/total_weight);
	    x += -(bottom_left/total_weight);
	    x += (top_right/total_weight);
	    x += (bottom_right/total_weight);

	    y += -(top_left/total_weight);
	    y += -(top_right/total_weight);
	    y += (bottom_right/total_weight);
	    y += (bottom_right/total_weight);
	  }

	  printf("%.2f %.2f\n\n%.2f %.2f\n", top_left, top_right,
		 bottom_left, bottom_right);
	  printf("[pos(x,y)]: (%.2f, %.2f)\n", x, y);
	  printf("[total_weight]: %.2f\n", total_weight);
	  
	  send_buf[0] = param->input.data[1];
	  send_buf[1] = (uint8_t)(x * 0x7f + 0x7f);
	  send_buf[2] = (uint8_t)(y * 0x7f + 0x7f);
	  esp_err_t ret = spi_slave_transmit(RCV_HOST, &spi_trans, portMAX_DELAY);
	  printf("Recieved: 0x%02X%02X%02X\n",
		 recv_buf[0], recv_buf[1], recv_buf[2]);

	  uint8_t data[] = {
	    0x04, 0xA4, 0x00, 0x00, 0x00, 0x0b
	  };
	  esp_hidh_dev_output_set(param->input.dev, param->input.map_index,
				  0x17, data, 6);
	}
	
	
	printf("Memory Data: ");
	for(int i = 0; i < param->input.length; i++) {
	  printf("%02X ", param->input.data[i]);
	}
	printf("\n");
	/* uint8_t data[] = { */
	/*   0x08, 0xA4, 0x00, 0x00, 0x00, 0x0b */
	/* }; */
	  
	
	break;
      case 48:
	if(!set_lights) {
	  uint8_t data[] = {
	    0x10
	  };
	  esp_hidh_dev_output_set(param->input.dev, param->input.map_index,
				  0x11, data, 1);
	  set_lights = true;
	}
	else if(calibration[0][0] == 0xffff && calibration[1][3] == 0xffff) {
	  calibrating = true;
	  uint8_t data[] = {
	    0x04, 0xA4, 0x00, 0x24, 0x00, 0x18
	  };
	  esp_hidh_dev_output_set(param->input.dev, param->input.map_index,
				  0x17, data, 6);
	  printf("Send Get Calibration Data\n");
	}
	else {
	  uint8_t data[] = {
	    0x04, 0xA4, 0x00, 0x00, 0x00, 0x0b
	  };
	  esp_hidh_dev_output_set(param->input.dev, param->input.map_index,
				  0x17, data, 6);
	}
	break;
      default:;
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
	break;
      }
      /* if(param->input.report_id == 33) { */
      /*   float total_right = param->input.data[5] + param->input.data[7]; */
      /*   float total_left = param->input.data[9] + param->input.data[11]; */

      /*   float total_top = param->input.data[5] + param->input.data[9]; */
      /*   float total_bottom = param->input.data[7] + param->input.data[11]; */

      /* 	float x, y; */
      /* 	x = total_right/total_left; */
      /* 	if(x > 1) x = 1 - total_right / total_left; */
      /* 	else x -= 1; */

      /* 	y = total_bottom/total_top; */
      /* 	if(y > 1) y = 1 - total_bottom / total_top; */
      /* 	else y -= 1; */
	
      /* 	printf("[Position]: (%.2f, %.2f)\n", x, y); */
      /* 	vTaskDelay(100); */
      /* 	esp_hidh_dev_output_set(param->input.dev, param->input.map_index, */
      /* 				0x17, data, 6); */
      /* 	break; */
      /* } */

	/* if(!set_lights) { */
	  
	/*   set_lights = true; */
	/* } */

	/* if(param->input.report_id == 33) { */
	/*   vTaskDelay(100); */
	/* } */
	/* esp_hidh_dev_output_set(param->input.dev, param->input.map_index, */
	/* 		      0x17, data, 6); */
	//esp_hidh_dev_get_report(param->input.dev, param->input.map_index
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 3

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
	  //open the last result
	  esp_hidh_dev_t *dev = esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);

	  fprintf(stderr, "%ld\n", (long int)dev);
        }
        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "HELLO WORLD\n");
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);


    spi_bus_config_t buscfg={
      .mosi_io_num=GPIO_MOSI,
      .miso_io_num=GPIO_MISO,
      .sclk_io_num=GPIO_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg={
      .mode=0,
      .spics_io_num=GPIO_CS,
      .queue_size=3,
      .flags=0,
      .post_setup_cb=NULL,
      .post_trans_cb=NULL
    };

    /* gpio_config_t io_conf={ */
    /*   .intr_type=GPIO_INTR_DISABLE, */
    /*   .mode=GPIO_MODE_OUTPUT, */
    /*   .pin_bit_mask=(1<<GPIO_HANDSHAKE) */
    /* }; */

    /* gpio_config(&io_conf); */

    /* gpio_config(&io_conf); */

    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret=ESP_OK);
    spi_trans.length = 3*8;
    spi_trans.tx_buffer = send_buf;
    spi_trans.rx_buffer = recv_buf;
    
}
