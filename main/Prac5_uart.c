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

#define SCHEME UART_SEND_SCHEME
#if SCHEME == UART_SEND_SCHEME
// UART 1 used for IR TX
#define IR_TX_UART_PORT     1
#define IR_TX_TX_PIN        15
#define IR_TX_RX_PIN        4

#define IR_BIT_SIZE         7
#define IR_MUL              3
#define IR_LOW              0x5B   //0b101 1011
#define IR_HIGH             0x00
#define UART_IR_DIV         6
#define IR_TX_BAUDS         (IR_FREQ*IR_MUL)
#define IR_RX_BAUDS         ((IR_FREQ / (IR_BIT_SIZE + 2)) / UART_IR_DIV) //= ~700
#else
#define IR_BIT_SIZE     8
#define IR_LOW          0xF0//0x38   //0b 111000
#define IR_MUL          9
#define IR_TX_BAUDS     380000UL //(38000*IR_MUL)

char caIrHigh[] = {IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,
                    IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,IR_HIGH,};
char caIrLow[] = {IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,
                    IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW,IR_LOW};

#endif

/* Message printed by the "consoletest" command.
 * It will also be used by the default UART to check the reply of the second
 * UART. As end of line characters are not standard here (\n, \r\n, \r...),
 * let's not include it in this string. */
const char test_message[] = "This is an example string, if you can read this, the example is a success!";

/**
 * @brief Configure and install the default UART, then, connect it to the
 * console UART.
 */
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
    uart_write_bytes(IR_TX_UART_PORT, caIrHigh, sizeof(caIrHigh));
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
    uart_write_bytes(IR_TX_UART_PORT, caIrLow, sizeof(caIrLow));
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
