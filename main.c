/*
 * Implemenetation of lock mechanism accessory for a magnet lock.
 * When unlocked, it changes relay state (unlocks) for configured period and then
 * changes it back.
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>
#include "ota-api.h"

#include "button.h"

// The GPIO pin that is connected to a relay
const int relay_gpio = 4;
// The GPIO pin that is connected to a LED
// const int led_gpio = 13;
//const int led_gpio = 2;
// The GPIO pin that is connected to a button
// const int button_gpio = 0;
const int button_gpio = 5;

// Timeout in seconds to open lock for
const int unlock_period = 3000;  // 5 seconds
// Which signal to send to relay to open the lock (0 or 1)
const int relay_open_signal = 700;
TimerHandle_t xUnlockTimer = NULL;
TimerHandle_t xRelayTimer = NULL;


void lock_lock();
void lock_unlock();

void relay_write(int value) {
    gpio_write(relay_gpio, value ? 1 : 0);
}



void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i=0; i<3; i++) {
        //led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    printf("Resetting Wifi Config\n");

    wifi_config_reset();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Resetting HomeKit Config\n");

    homekit_server_reset();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Restarting\n");

    sdk_system_restart();

    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

void gpio_init() {
    gpio_enable(relay_gpio, GPIO_OUTPUT);
    relay_write(!relay_open_signal);
}

void button_callback(uint8_t gpio, button_event_t event) {
    switch (event) {
        case button_event_single_press:
            printf("Toggling relay\n");
            lock_unlock();
            break;
        case button_event_long_press:
            reset_configuration();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}


void lock_identify(homekit_value_t _value) {
    printf("Lock identify\n");
   // xTaskCreate(lock_identify_task, "Lock identify", 128, NULL, 2, NULL);
}

int lock_state = 1;
typedef enum {
    lock_state_unsecured = 0,
    lock_state_secured = 1,
    lock_state_jammed = 2,
    lock_state_unknown = 3,
    
} lock_state_t;



homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Lock");

homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;

homekit_characteristic_t current_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_CURRENT_STATE,
    lock_state_secured,
);

void lock_target_state_setter(homekit_value_t value);

homekit_characteristic_t lock_target = HOMEKIT_CHARACTERISTIC_(
    LOCK_TARGET_STATE,
    lock_state_secured,
    .setter=lock_target_state_setter,
);

void lock_target_state_setter(homekit_value_t value) {
    lock_target.value = value;
     printf("Value %d\n", value.int_value);
	

    if (value.int_value == 0) {
        
       lock_unlock();
         
    } else {
        lock_lock();
      
    }
}


void lock_control_point(homekit_value_t value) {
    // Nothing to do here
}




void lock_lock() {
 printf("Lock\n");
      relay_write(0);
      if (current_state.value.int_value != lock_state_secured) {
        current_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&current_state, current_state.value);
    }
}


void lock_unlock() {
printf("Unlock\n");
  relay_write(1);
  current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
   homekit_characteristic_notify(&current_state, current_state.value);
  xTimerStart(xUnlockTimer, 10 );
  xTimerStart(xRelayTimer, 10 );
}

void unlock_time(){
  if (lock_target.value.int_value != lock_state_secured) {
        lock_target.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_target, lock_target.value);
    }

    lock_lock();
    
 

}
void relay_timeout(){
	relay_write(0);
}

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_door_lock, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "KRD"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1234"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Lock"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, lock_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MECHANISM, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Door lock"),
            &current_state,
            &lock_target,
            &ota_trigger,
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MANAGEMENT, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(LOCK_CONTROL_POINT,
                .setter=lock_control_point
            ),
            HOMEKIT_CHARACTERISTIC(VERSION, "2"),
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Lock-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Lock-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);
    xUnlockTimer = xTimerCreate("Unlock Timer",(unlock_period/portTICK_PERIOD_MS),pdFALSE,0, unlock_time);
    xRelayTimer = xTimerCreate("Reley Timeout",(relay_open_signal/portTICK_PERIOD_MS),pdFALSE,0, relay_timeout);


    create_accessory_name();

    wifi_config_init("lock", NULL, on_wifi_ready);
    gpio_init();
    //lock_init();

    if (button_create(button_gpio, 0, 4000, button_callback)) {
        printf("Failed to initialize button\n");
    }
}
