#pragma once

#define DRIVER_0_ADDRES           0
#define DRIVER_0_ENABLE           GPIO_NUM_18   // H= disable motor output

#define VCC_IO                    GPIO_NUM_21  // L = reset všech driverů, H = provozní stav
#define ENCODER_0_A               GPIO_NUM_36  // vstup z indexu driveru 0
#define DRIVERS_UART              UART_NUM_1
#define DRIVERS_UART_TXD          GPIO_NUM_15
#define DRIVERS_UART_RXD          GPIO_NUM_5
#define DRIVERS_UART_BUF_SIZE     256
#define DRIVERS_RX_TIMEOUT        (20 / portTICK_RATE_MS)
#define DRIVERS_UART_START_BYTE   0x05

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<DRIVER_0_ENABLE) | (1ULL<<VCC_IO))

#define MOTOR_SPEED_COEFICIENT    71608     // 71608 = 1RPS

#define ENCODER_H_LIM_VAL         1000
#define ENCODER_L_LIM_VAL        -1000

// globální proměnné pro pokusy s grafickým rozhraním
volatile int motor_speed = 2 * MOTOR_SPEED_COEFICIENT; 
volatile int motor_load = 0;
volatile int motor_stop_sensitivity = 100;
volatile int potenciometr = 0;
volatile int i_run = 8;
volatile int i_hold = 0;
volatile bool start_stop = false;
volatile unsigned mot_load[2048];
volatile unsigned mot_pos[2048];