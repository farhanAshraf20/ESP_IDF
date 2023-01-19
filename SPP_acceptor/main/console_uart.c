/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "driver/uart.h"
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "console_uart.h"
#include "app_spp_msg_prs.h"

#include "esp_spp_api.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"


#define CONSOLE_UART_NUM    UART_NUM_0

#define UART_Buff_size  127

uint8_t Rx_Data[UART_Buff_size] = {0};

static QueueHandle_t uart_queue;
QueueHandle_t xQueue2;
extern esp_spp_cb_param_t lParam;
static spp_msg_prs_cb_t spp_msg_parser;


static const uart_config_t uart_cfg = {
    .baud_rate = 115200,   //1.5M
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 127,
};

extern void spp_msg_args_parser(char *buf, int len);


void UART_Rec_Spp_task( void * pvParameters )
{
	uint8_t t_buff[UART_Buff_size]={0};

	while(1){
		if(xQueue2 != NULL){
			if( xQueueReceive(xQueue2, &t_buff, (TickType_t)10)){
				//printf("UART Received data from queue == %s\n", t_buff);
				if(lParam.open.status == ESP_SPP_SUCCESS)
				{
					int f_size = strlen((const char *)t_buff );
					t_buff[f_size+1] = '\r';
					esp_spp_write(lParam.srv_open.handle, (f_size+2), t_buff);
				}
				vTaskDelay(100/ portTICK_RATE_MS);
			}
		}
	}
	vTaskDelete(NULL);
}


void spp_msg_handler(char *buf, int len)
{
    ESP_LOGE(TAG_CNSL, "Command [%s]", buf);
    spp_msg_args_parser(buf, len);
}

static void console_uart_task(void *pvParameters)
{
    int len;
    uart_event_t event;
    spp_msg_prs_cb_t *parser = &spp_msg_parser;
    spp_msg_parser_reset_state(parser);
    spp_msg_parser_register_callback(parser, spp_msg_handler);
    spp_msg_show_usage();
#define TMP_BUF_LEN 128
    uint8_t tmp_buf[128] = {0};

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
                //Event of UART receving data
                case UART_DATA:
                {
                    len = uart_read_bytes(CONSOLE_UART_NUM, tmp_buf, TMP_BUF_LEN, 0);
                    /*for (int i = 0; i < len; i++) {
                        spp_msg_parse(tmp_buf[i], parser);
                        Rx_Data[i]=tmp_buf[i];
                        }*/
                    if((strlen((const char *)tmp_buf) != 0) && (tmp_buf[0] != '\r') )
                    {
                        xQueueSend( xQueue2, (void *)&tmp_buf, ( TickType_t ) 10 );
                    }
                    memset(tmp_buf,0,len);
                    break;
                }
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                {
                    ESP_LOGI(TAG_CNSL, "hw fifo overflow");
                    break;
                }
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                {
                    ESP_LOGI(TAG_CNSL, "ring buffer full");
                    break;
                }
                //Event of UART RX break detected
                case UART_BREAK:
                {
                    ESP_LOGI(TAG_CNSL, "uart rx break");
                    break;
                }
                //Event of UART parity check error
                case UART_PARITY_ERR:
                {
                    ESP_LOGI(TAG_CNSL, "uart parity error");
                    break;
                }
                //Event of UART frame error
                case UART_FRAME_ERR:
                {
                    ESP_LOGI(TAG_CNSL, "uart frame error");
                    break;
                }
                //Others
                default:
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}


esp_err_t console_uart_init(void)
{
    esp_err_t ret;

    ret =  uart_param_config(CONSOLE_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CNSL, "Uart %d initialize err %04x", CONSOLE_UART_NUM, ret);
        return ret;
    }

    uart_set_pin(CONSOLE_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CONSOLE_UART_NUM, 1024, 1024, 8, &uart_queue, 0);
    xTaskCreate(console_uart_task, "uTask", 2048, NULL, 8, NULL);




    /* Create the task for Spp Tx Data, storing the handle. */
       xQueue2 = xQueueCreate( 10, 126 );
       xTaskCreate(
       					UART_Rec_Spp_task,       /* Function that implements the task. */
       					"utask2",          /* Text name for the task. */
       					2048,      /* Stack size in words, not bytes. */
       					NULL,    /* Parameter passed into the task. */
       					8,/* Priority at which the task is created. */
       					NULL );      /* Used to pass out the created task's handle. */

    return ESP_OK;
}
