/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "myUart.h"


#define UART_SEND_SCHEME    0
#define TIMER_SEND_SCHEME   1

// UART 0 used for PC communication
#define PC_UART_PORT        0
#define PC_UART_RX_PIN      3
#define PC_UART_TX_PIN      1
#define PC_UARTS_BAUD_RATE  (115200)

// UART 2 used for IR RX
#define IR_RX_UART_PORT     2
#define IR_RX_RX_PIN        16
#define IR_RX_TX_PIN        17

#define UART_BUF_SIZE       (1024)

#define IR_FREQ             38000
#define MAX_PACKET_SIZE     255

#define IR_TX_TX_PIN        15

#define SCHEME TIMER_SEND_SCHEME
#if SCHEME == UART_SEND_SCHEME
// UART 1 used for IR TX
#define IR_TX_UART_PORT     1
#define IR_TX_RX_PIN        4
#define IR_BIT_SIZE         7
#define IR_MUL              3
#define IR_LOW              0x5B   //0b101 1011
#define IR_HIGH             0x00
#define UART_IR_DIV         6
#define IR_TX_BAUDS         (IR_FREQ*IR_MUL)
#define IR_RX_BAUDS         ((IR_FREQ / (IR_BIT_SIZE + 2)) / UART_IR_DIV) //= ~700
#else // Timer mode
#include "driver/ledc.h"
#include "freertos/queue.h"
#include "driver/timer.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (IR_TX_TX_PIN) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               ((((1 << 8) - 1) * 2) / 3) // Set duty to 66%. 
#define LEDC_FREQUENCY          (38000) // Frequency in Hertz. Set frequency at 5 kHz
#define IR_RX_BAUDS             (1000)

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/* Timer interrupt service routine */
static void IRAM_ATTR timer0_ISR(void *ptr)
{
    static uint8_t bState = 0;
    if (bState)
    {
        bState = 0;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    }
    else
    {
        bState = 1;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    }
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

#define TIMER_INTR_US          1000                                 // Execution time of each ISR interval in micro-seconds
#define TIMER_DIVIDER         (16)                                  //  Hardware timer clock divider
#define TIMER_TICKS            (TIMER_BASE_CLK / TIMER_DIVIDER)     // TIMER_BASE_CLK = APB_CLK = 80MHz
#define SEC_TO_MICRO_SEC(x)    ((x) / 1000 / 1000)                  // Convert second to micro-second
#define ALARM_VAL_US           SEC_TO_MICRO_SEC(TIMER_INTR_US * TIMER_TICKS)     // Alarm value in micro-seconds
/* Timer group0 TIMER_0 initialization */
static void timer0_init(void)
{
    esp_err_t ret;
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .intr_type = TIMER_INTR_LEVEL,
        .auto_reload = 1,
    };

    ret = timer_init(TIMER_GROUP_0, TIMER_0, &config);
    ESP_ERROR_CHECK(ret);
    ret = timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    ESP_ERROR_CHECK(ret);
    ret = timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ALARM_VAL_US);
    ESP_ERROR_CHECK(ret);
    ret = timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    ESP_ERROR_CHECK(ret);
    /* Register an ISR handler */
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer0_ISR, NULL, 0, NULL);
}

#endif


void uartInit(uart_port_t uart_num, uint32_t baudrate, uint8_t size, uint8_t parity, uint8_t stop, uint8_t txPin, uint8_t rxPin)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = size-5,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, txPin, rxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

}

void delayMs(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void uartClrScr(uart_port_t uart_num)
{
    // Uso "const" para sugerir que el contenido del arreglo lo coloque en Flash y no en RAM
    const char caClearScr[] = "\e[2J";
    uart_write_bytes(uart_num, caClearScr, sizeof(caClearScr));
}
void uartGoto11(uart_port_t uart_num)
{
    // Limpie un poco el arreglo de caracteres, los siguientes tres son equivalentes:
     // "\e[1;1H" == "\x1B[1;1H" == {27,'[','1',';','1','H'}
    const char caGoto11[] = "\e[1;1H";
    uart_write_bytes(uart_num, caGoto11, sizeof(caGoto11));
}

bool uartKbhit(uart_port_t uart_num)
{
    uint8_t length;
    uart_get_buffered_data_len(uart_num, (size_t*)&length);
    return (length > 0);
}

void uartPutchar(uart_port_t uart_num, char c)
{
    uart_write_bytes(uart_num, &c, sizeof(c));
}

char uartGetchar(uart_port_t uart_num)
{
    char c;
    // Wait for a received byte
    while(!uartKbhit(uart_num))
    {
        delayMs(10);
    }
    // read byte, no wait
    uart_read_bytes(uart_num, &c, sizeof(c), 0);

    return c;
}

void IR_SendBit(uint8_t bit)
{
    if (bit)
    {   
#if SCHEME == UART_SEND_SCHEME
        for (uint8_t i = 0; i < UART_IR_DIV; i++)
        {
            uartPutchar(IR_TX_UART_PORT, IR_HIGH);
            uartPutchar(IR_TX_UART_PORT, IR_HIGH);
            uartPutchar(IR_TX_UART_PORT, IR_HIGH);
        }
#else
    // start timer and put value in circular buffer
#endif
    }
    else
    {        
#if SCHEME == UART_SEND_SCHEME
        for (uint8_t i = 0; i < UART_IR_DIV; i++)
        {
            uartPutchar(IR_TX_UART_PORT, IR_LOW);
            uartPutchar(IR_TX_UART_PORT, IR_LOW);
            uartPutchar(IR_TX_UART_PORT, IR_LOW);
        }
#else
    // start timer and put value in circular buffer
#endif
    }
}

void IR_SendByte(uint8_t data)
{
    //Start Bit
    IR_SendBit(0);

    for (uint8_t bitIdx = 0; bitIdx < 8; bitIdx++)
    {
        IR_SendBit(data & 1);  
        data >>= 1;  
    }
    //Stop Bit
    IR_SendBit(1);
}

void IR_SendPacket(uint8_t *data, uint8_t len)
{
    // Preambulo 0xCO, 0xD1, 0xCE
    // Header
        // Payload Length
        // Checksum8  
    // Payload = Data
    // Checksum16
}

uint8_t IR_ReceivePacket(uint8_t *data)
{
    // Preambulo // verify
    // Header   //verify
    // Payload  //process
    // Checksum
    return 0;
}

void app_main(void)
{
    uartInit(PC_UART_PORT, PC_UARTS_BAUD_RATE, 8, 0, 1, PC_UART_TX_PIN, PC_UART_RX_PIN);
#if SCHEME == UART_SEND_SCHEME
    uartInit(IR_TX_UART_PORT, IR_TX_BAUDS, IR_BIT_SIZE, 0, 1, IR_TX_TX_PIN, IR_TX_RX_PIN);
#else
    ledc_init();
    timer0_init();
#endif
    uartInit(IR_RX_UART_PORT, IR_RX_BAUDS, 8, 0, 1, IR_RX_TX_PIN, IR_RX_RX_PIN);
    
    delayMs(500);
    uartGoto11(PC_UART_PORT);
    delayMs(500);
    uartPutchar(PC_UART_PORT, '#');
    uartClrScr(PC_UART_PORT);

    // Wait for input
    delayMs(500);
    
    // echo forever
    while(1)
    {
        uartPutchar(PC_UART_PORT,uartGetchar(PC_UART_PORT));
        IR_SendByte('H');
        IR_SendByte('e');
        IR_SendByte('l');
        IR_SendByte('l');
        IR_SendByte('o');
        IR_SendByte('!');
        delayMs(50);
        while (uartKbhit(IR_RX_UART_PORT))
        {
            uartPutchar(PC_UART_PORT,'{');
            uartPutchar(PC_UART_PORT,uartGetchar(IR_RX_UART_PORT));
            uartPutchar(PC_UART_PORT,'}');
        }
        
    }
}
