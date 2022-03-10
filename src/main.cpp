#define GRIDUI_LAYOUT_DEFINITION
#include "layout.hpp"           //  pro grafické rozhraní
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "defines.hpp"
#include "utils.hpp"
#include "init.hpp"
#include "uart.hpp"
#include "driver.hpp"
#include "gridui.h"         //  pro grafické rozhraní          
#include "rbprotocol.h"     //  pro grafické rozhraní
#include "rbwebserver.h"    //  pro grafické rozhraní
#include "rbwifi.h"         //  pro grafické rozhraní

using namespace rb;

static void initGridUi() {
    using namespace gridui;
    // Initialize WiFi
    WiFi::startAp("Retractor", "12345678");     //esp vytvoří wifi sít
    // WiFi::connect("Jmeno wifi", "Heslo");    //připojení do místní sítě
    
    // Initialize RBProtocol
    auto *protocol = new Protocol("FrantaFlinta", "Retractor", "Compiled at " __DATE__ " " __TIME__, [](const std::string& cmd, rbjson::Object* pkt) {
        UI.handleRbPacket(cmd, pkt);
    });
    protocol->start();
    // Start serving the web page
    rb_web_start(80);
    // Initialize the UI builder
    UI.begin(protocol);
    // Build the UI widgets. Positions/props are set in the layout, so most of the time,
    // you should only set the event handlers here.
    auto builder = gridui::Layout.begin();
    //builder.MotorSpeed.min(MOTOR_SPEED_MIN);
    //builder.MotorSpeed.max(MOTOR_SPEED_MAX);
    builder.StartStopButton.onPress([](Button &){
        start_stop = true;
    });

    builder.StartStopButton.onRelease([](Button &){
        start_stop = false;
    });
    builder.MotorSpeed.onChanged([](Slider &s) {
        motor_speed = int(MOTOR_SPEED_COEFICIENT * s.value());
        //printf("motor_speed: %f -> %d\n", s.value(), motor_speed);    
    });

     builder.StopSensitivity.onChanged([](Slider &s) {
        motor_stop_sensitivity = int(s.value());
       // printf("stop sensitivity:%f -> %d\n",s.value(), motor_stop_sensitivity);
    });

    builder.IRun.onChanged([](Slider &s) {
        i_run = int(s.value());
        //printf("I_Run / 32:%f -> %d\n",s.value(), i_run);
    });

    // Commit the layout. Beyond this point, calling any builder methods on the UI is invalid.
    builder.commit();
}

static void initDriver(Driver& driver, const int iRun, const int iHold) {
    driver.init();
    vTaskDelay(100 / portTICK_PERIOD_MS);   
    uint32_t data =0;
    int result = driver.get_PWMCONF(data);
    if (result != 0)
        printf("PWMCONF driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("PWMCONF driveru %d =  %08X\n", driver.address(), data);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    result = driver.get_DRV_STATUS(data);
    if (result != 0)
        printf("DRV_STATUS driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("DRV_STATUS driveru  %d =  %08X\n", driver.address(), data);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    driver.set_speed(0);                      // otáčení motoru se nastavuje zápisem rychlosti do driveru přes Uart
    driver.set_IHOLD_IRUN (8, 8);             // proud IHOLD (při stání) =8/32, IRUN (při běhu)= 8/32 (8/32 je minimum, 16/32 je maximum pro dluhodobější provoz)
    driver.enable();                          //zapnutí mptoru
    vTaskDelay(300 / portTICK_PERIOD_MS);     //doba stání pro nastavení automatiky driveru
    driver.set_IHOLD_IRUN (iRun, iHold);             //proud IHOLD =0, IRUN = 8/32 (při stání je motor volně otočný)
}

#include <stdio.h>
extern "C" void app_main(void)
{
    printf("Simple Motor \n\tbuild %s %s\n", __DATE__, __TIME__);

    check_reset();
    iopins_init();
    gpio_set_level(VCC_IO, 0);              // reset driveru
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(VCC_IO, 1);              //zapíná VCCIO driveru
    nvs_init();                             //inicializace pro zápis do flash paměti
    initGridUi();
    Uart drivers_uart {
        DRIVERS_UART,
        Uart::config_t {
            .baud_rate = 750000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .use_ref_tick = false
        },
        Uart::pins_t {
            .pin_txd = DRIVERS_UART_TXD,
            .pin_rxd = DRIVERS_UART_RXD,
            .pin_rts = UART_PIN_NO_CHANGE,
            .pin_cts = UART_PIN_NO_CHANGE
        },
        Uart::buffers_t {
            .rx_buffer_size = DRIVERS_UART_BUF_SIZE,
            .tx_buffer_size = 0,
            .event_queue_size = 0
        }
    };
    Driver driver0 { drivers_uart, DRIVER_0_ADDRES, DRIVER_0_ENABLE };
    initDriver(driver0, 8, 0);
   // driver0.set_speed(MOTOR_SPEED_COEFICIENT);
    //vTaskDelay(2000/portTICK_PERIOD_MS);
    //driver0.set_speed(0);
    for(;;) {
        if(start_stop == true) {
            gridui::Layout.StartStopButton.setText("Stop");
            start_stop = false;
            while (1) {
                driver0.set_IHOLD_IRUN(i_run, i_hold);
                driver0.set_speed(motor_speed);     //71608 = 1RPS  
                vTaskDelay(50 / portTICK_PERIOD_MS);
                uint32_t data = 0;
                int result = driver0.get_SG(data);
                if(result != 0){
                    printf("SG_RESULT: ERROR  %d\n", result);
                }
                else {
                    motor_load = (data >> 1); 
                    gridui::Layout.MotorLoad.setValue(motor_load);
                    if ((motor_load < motor_stop_sensitivity) or (start_stop == true)) {                       
                        driver0.set_speed(0);
                        vTaskDelay(200 / portTICK_PERIOD_MS);
                        gridui::Layout.StartStopButton.setText("Start");
                        start_stop = false;
                        break;
                    }
                }
            }
             vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
