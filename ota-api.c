#include <stdlib.h>  //for printf
#include <stdio.h>
#include <string.h>
//#include "ota-api.h"
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <rboot-api.h>
#include <sysparam.h>

// the first function is the ONLY thing needed for a repo to support ota after having started with ota-boot
// in ota-boot the user gets to set the wifi and the repository details and it then installs the ota-main binary

void ota_update(void *arg) {  //arg not used
    rboot_set_temp_rom(1); //select the OTA main routine
    sdk_system_restart();  //#include <rboot-api.h>
    // there is a bug in the esp SDK such that if you do not power cycle the chip after serial flashing, restart is unreliable
}
/*
// this function is optional to couple Homekit parameters to the sysparam variables and github parameters
unsigned int  ota_read_sysparam(char **manufacturer,char **serial,char **model,char **revision) {
    sysparam_status_t status;
    char *value;

    status = sysparam_get_string("ota_repo", &value);
    if (status == SYSPARAM_OK) {
        strchr(value,'/')[0]=0;
        *manufacturer=value;
        *model=value+strlen(value)+1;
    } else {
        *manufacturer="manuf_unknown";
        *model="model_unknown";
    }
    status = sysparam_get_string("ota_version", &value);
    if (status == SYSPARAM_OK) {
        *revision=value;
    } else *revision="0.0.0";

    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);
    *serial=malloc(18);
    sprintf(*serial,"%02X:%02X:%02X:%02X:%02X:%02X",macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);

    unsigned int c_hash=0;
    char version[16];
    char* rev=version;
    char* dot;
    strncpy(rev,*revision,16);
    if ((dot=strchr(rev,'.'))) {dot[0]=0; c_hash=            atoi(rev); rev=dot+1;}
    if ((dot=strchr(rev,'.'))) {dot[0]=0; c_hash=c_hash*1000+atoi(rev); rev=dot+1;}
                                          c_hash=c_hash*1000+atoi(rev);
                                            //c_hash=c_hash*10  +configuration_variant; //possible future extension
    printf("manuf=\'%s\' serial=\'%s\' model=\'%s\' revision=\'%s\' c#=%d\n",*manufacturer,*serial,*model,*revision,c_hash);
    return c_hash;
}
*/
void save_characteristic_to_flash(homekit_characteristic_t *ch, homekit_value_t value){
    
    sysparam_status_t status = SYSPARAM_OK;
    bool bool_value;
    int8_t int8_value;
    int32_t int32_value;
    float float_value;
    char *string_value=NULL;    

    printf ("Save characteristic to flash\n");
    switch (ch->format) {
        case homekit_format_bool:
            printf ("writing bool value to flash\n");
            status = sysparam_get_bool(ch->description, &bool_value);
            if (status == SYSPARAM_OK && bool_value != ch->value.bool_value) {
                status = sysparam_set_bool(ch->description, ch->value.bool_value);
            } else if (status == SYSPARAM_NOTFOUND) {
                status = sysparam_set_bool(ch->description, ch->value.bool_value);
            }
            break;
        case homekit_format_uint8:
            printf ("writing int8 value to flash\n");
            status = sysparam_get_int8(ch->description, &int8_value);
            if (status == SYSPARAM_OK && int8_value != ch->value.int_value) {
                status = sysparam_set_int8(ch->description, ch->value.int_value);
            } else  if (status == SYSPARAM_NOTFOUND) {
                status = sysparam_set_int8(ch->description, ch->value.int_value);
            }
            break;
        case homekit_format_uint16:
        case homekit_format_uint32:
            printf ("writing int32 value to flash\n");
            status = sysparam_get_int32(ch->description, &int32_value);
            if (status == SYSPARAM_OK && int32_value != ch->value.int_value) {
                status = sysparam_set_int32(ch->description, ch->value.int_value);
            } else  if (status == SYSPARAM_NOTFOUND) {
                status = sysparam_set_int32(ch->description, ch->value.int_value);
            }
            break;
        case homekit_format_string:
            printf ("writing string value to flash\n");
            status = sysparam_get_string(ch->description, &string_value);
            if (status == SYSPARAM_OK && !strcmp (string_value, ch->value.string_value)) {
                status = sysparam_set_string(ch->description, ch->value.string_value);
            }  else  if (status == SYSPARAM_NOTFOUND) {
                status = sysparam_set_string(ch->description, ch->value.string_value);
            }
            free(string_value);
            break;
        case homekit_format_float:
            printf ("writing float value to flash\n");
            status = sysparam_get_int32(ch->description, &int32_value);
            float_value = int32_value * 1.00f / 100;
            if (status == SYSPARAM_OK && float_value != ch->value.float_value) {
		int32_value = (int)ch->value.float_value*100;
		status = sysparam_set_int32(ch->description, int32_value);
            } else if (status == SYSPARAM_NOTFOUND) {
		int32_value = (int)ch->value.float_value*100;
                status = sysparam_set_int32(ch->description, int32_value);
            }
	    break;
        case homekit_format_uint64:
        case homekit_format_int:
        case homekit_format_tlv:
        default:
            printf ("Unknown characteristic format in save_charactersitics_to_flash\n");
    }
    if (status != SYSPARAM_OK){
        printf ("Error in sysparams error:%i writing characteristic\n", status);
    }
    
}

void load_characteristic_from_flash (homekit_characteristic_t *ch){
              
                    
    sysparam_status_t status = SYSPARAM_OK;
    bool bool_value;
    int8_t int8_value;
    int32_t int32_value;
    char *string_value = NULL;
    printf ("Loading sysparam %s\n",ch->description);
    switch (ch->format){
        case homekit_format_bool:
            printf("Loading bool\n");
            status = sysparam_get_bool(ch->description, &bool_value);
            if (status == SYSPARAM_OK ) {
                ch->value.bool_value = bool_value;
            }
            break;
        case homekit_format_uint8:
            printf("Loading int8\n");
            status = sysparam_get_int8(ch->description, &int8_value);
            if (status == SYSPARAM_OK) {
                ch->value.int_value = int8_value;
            }
            break;
        case homekit_format_uint16:
        case homekit_format_uint32:
            printf("Loading in32\n");
            status = sysparam_get_int32(ch->description, &int32_value);
            if (status == SYSPARAM_OK ) {
                ch->value.int_value = int32_value;
            }
            break;
        case homekit_format_string:
            printf("Loading string\n");
            status = sysparam_get_string(ch->description, &string_value);
            if (status == SYSPARAM_OK) {
                ch->value = HOMEKIT_STRING(string_value);
            }
            break;
        case homekit_format_float:
            printf("Loading float\n");
            status = sysparam_get_int32(ch->description, &int32_value);
            if (status == SYSPARAM_OK ) {
                ch->value.float_value = int32_value * 1.0f /100;
            }
            break;
        case homekit_format_uint64:
        case homekit_format_int:
        case homekit_format_tlv:
        default:
            printf ("Unknown characteristic format in save_charactersitics_to_flash\n");
    }
    if (status != SYSPARAM_OK){
        printf ("Error in sysparams error:%i loading characteristic\n", status);
    }
    
    
    
}


#include <homekit/characteristics.h>

#include <esplibs/libmain.h>
#include <etstimer.h>

static ETSTimer update_timer;

void ota_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        printf("Invalid ota-value format: %d\n", value.format);
        return;
    }
    if (value.bool_value) {
        //make a distinct light pattern or other feedback to the user = call identify routine
        sdk_os_timer_setfn(&update_timer, ota_update, NULL);
        sdk_os_timer_arm(&update_timer, 500, 0); //wait 0.5 seconds to trigger the reboot so gui can update and events sent
    }
}
