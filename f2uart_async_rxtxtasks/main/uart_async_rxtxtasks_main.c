/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 240;
#define C_W_Buff_size	242
volatile uint16_t count = 0;
volatile uint16_t count1 = 0;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
QueueHandle_t xQueue2;

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    //uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void UART_Rec_Spp_task( void * pvParameters )
{
	uint8_t t_buff[C_W_Buff_size]={0};
	memset(t_buff,0,C_W_Buff_size);
	while(1){
		if(xQueue2 != NULL){
			if(xQueueReceive(xQueue2, &t_buff, portMAX_DELAY)){
				count++;
				printf("<--[%d]\n",count);
				//printf("%s",t_buff);
				//ESP_EARLY_LOGI("utask2_4", "Data waiting in myQueue : %d, Space available : %d", uxQueueMessagesWaiting(xQueue2), uxQueueSpacesAvailable(xQueue2));
			}
		}else{
			//printf("Fail dQueue\n");
		}
		vTaskDelay(5/ portTICK_RATE_MS);
	}
vTaskDelete(NULL);
}





int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        //sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    memset(data,0,RX_BUF_SIZE+1);

    //while (1)
    for (;;){
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE,portMAX_DELAY);// 1000 / portTICK_RATE_MS);
        printf("-->[%d]		",rxBytes);
        if (rxBytes != 0) {
            data[rxBytes] = 0;
            count1++;
            printf("-->[%d]		",count1);
            if(xQueueSend( xQueue2, (void *)data,0) == pdPASS){
            	memset(data,0,RX_BUF_SIZE+1);
            	uart_flush_input(UART_NUM_0);
            }else{
				printf("Failed To Queue\n");
			}

			//ESP_EARLY_LOGI("utask2_4", "Data waiting in myQueue : %d, Space available : %d", uxQueueMessagesWaiting(xQueue2), uxQueueSpacesAvailable(xQueue2));
            //ESP_LOGI(RX_TASK_TAG, "Read [%d] bytes\n -->%s", rxBytes, data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
       // vTaskDelay(20/ portTICK_RATE_MS);
    }
    free(data);
    data=NULL;
}

void app_main(void)
{
	init();
	xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
	//xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

	xQueue2 = xQueueCreate( 20, C_W_Buff_size );
	xTaskCreate(
			UART_Rec_Spp_task,		/* Function that implements the task. */
			"utask2",				/* Text name for the task. */
			2048,      				/* Stack size in words, not bytes. */
			NULL,    				/* Parameter passed into the task. */
			configMAX_PRIORITIES,	/* Priority at which the task is created. */
			NULL );      			/* Used to pass out the created task's handle. */
}
