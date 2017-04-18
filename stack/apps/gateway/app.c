/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2015 University of Antwerp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * \author	maarten.weyn@uantwerpen.be
 */

#include "hwuart.h"
#include "hwleds.h"
#include "hwsystem.h"
#include "scheduler.h"
#include "timer.h"
#include "log.h"
#include "debug.h"
#include "platform.h"

#include <stdio.h>
#include <stdlib.h>

#include "hwlcd.h"
#include "d7ap_stack.h"
#include "hwuart.h"
#include "fifo.h"
#include "version.h"

#if HW_NUM_LEDS > 0
#include "hwleds.h"

#ifdef HAS_LCD
  #include "platform_lcd.h"
  #define LCD_WRITE_STRING(...) lcd_write_string(__VA_ARGS__)
  #define LCD_WRITE_LINE(line, ...) lcd_write_line(line, __VA_ARGS__)
#else
  #define LCD_WRITE_STRING(...)
  #define LCD_WRITE_LINE(...)
#endif

static int Received = 0;
static int Ref = -1;

void led_blink_off()
{
	led_off(0);
}

void led_blink()
{
	led_on(0);

	timer_post_task_delay(&led_blink_off, TIMER_TICKS_PER_SEC * 0.2);
}

#endif

static void on_unsolicited_response_received(d7asp_result_t d7asp_result, uint8_t *alp_command, uint8_t alp_command_size)
{
#if HW_NUM_LEDS > 0
  led_blink();
#endif


  uint16_t *pointer =  (uint16_t*) alp_command;
  ++pointer;
  ++pointer;

  float internal_temp = (*pointer++)/10.0;
  uint32_t tData = (*pointer++)*100;
  uint32_t rhData = (*pointer++)*100;
  uint16_t vdd = (*pointer++);
  uint16_t Count = (*pointer++);

  if(Count < Ref)
	  Ref=-1;

  if(Ref==-1){
	  Ref = Count-1;
	  Received = Count;
  }else {
	  ++Received;
  }

  char str[30];
  sprintf(str, "Int T: %2d.%d C", (int)internal_temp, (int)(internal_temp*10)%10);
  LCD_WRITE_LINE(2,str);
  log_print_string(str);
  sprintf(str, "Ext T: %d.%d C", (int)(tData/1000), (int)(tData%1000)/100);
  LCD_WRITE_LINE(3,str);
  log_print_string(str);
  sprintf(str, "Ext H: %d.%d", (int)(rhData/1000), (int)(rhData%1000)/100);
  LCD_WRITE_LINE(4,str);
  log_print_string(str);
  sprintf(str, "Batt %d mV", (int)vdd);
  LCD_WRITE_LINE(5,str);
  log_print_string(str);
  sprintf(str, "Recv %d / %d         ", Received-Ref, (int)Count-Ref);
  LCD_WRITE_LINE(7,str);
  log_print_string(str);
  sprintf(str, "RX -%d", (int)d7asp_result.rx_level);
  LCD_WRITE_LINE(9,str);
  log_print_string(str);
  sprintf(str, "LB %d", (int)d7asp_result.link_budget);
  LCD_WRITE_LINE(10,str);
  log_print_string(str);
}


static alp_init_args_t alp_init_args;

void bootstrap()
{
    dae_access_profile_t access_profiles[6] = {
        {
            .channel_header = {
                .ch_coding = PHY_CODING_PN9,
                .ch_class = PHY_CLASS_NORMAL_RATE,
                .ch_freq_band = PHY_BAND_868
            },
            .subprofiles[0] = {
                .subband_bitmap = 0x01, // only the first subband is selectable
                .scan_automation_period = 0,
            },
            .subbands[0] = (subband_t){
                .channel_index_start = 0,
                .channel_index_end = 0,
                .eirp = 10,
                .cca = -86,
                .duty = 0,
            }
        },
        {
            .channel_header = {
                .ch_coding = PHY_CODING_PN9,
                .ch_class = PHY_CLASS_HI_RATE,
                .ch_freq_band = PHY_BAND_868
            },
            .subprofiles[0] = {
                .subband_bitmap = 0x01, // only the first subband is selectable
                .scan_automation_period = 0,
            },
            .subbands[0] = (subband_t){
                .channel_index_start = 0,
                .channel_index_end = 0,
                .eirp = 10,
                .cca = -86,
                .duty = 0,
            }
        },
        {
            .channel_header = {
                .ch_coding = PHY_CODING_PN9,
                .ch_class = PHY_CLASS_LO_RATE,
                .ch_freq_band = PHY_BAND_868
            },
            .subprofiles[0] = {
                .subband_bitmap = 0x01, // only the first subband is selectable
                .scan_automation_period = 0,
            },
            .subbands[0] = (subband_t){
                .channel_index_start = 0,
                .channel_index_end = 0,
                .eirp = 10,
                .cca = -86,
                .duty = 0,
            }
        },
        {
            .channel_header = {
                .ch_coding = PHY_CODING_PN9,
                .ch_class = PHY_CLASS_NORMAL_RATE,
                .ch_freq_band = PHY_BAND_433
            },
            .subprofiles[0] = {
                .subband_bitmap = 0x01, // only the first subband is selectable
                .scan_automation_period = 0,
            },
            .subbands[0] = (subband_t){
                .channel_index_start = 0,
                .channel_index_end = 0,
                .eirp = 10,
                .cca = -86,
                .duty = 0,
            }
        },
        {
            .channel_header = {
                .ch_coding = PHY_CODING_PN9,
                .ch_class = PHY_CLASS_HI_RATE,
                .ch_freq_band = PHY_BAND_433
            },
            .subprofiles[0] = {
                .subband_bitmap = 0x01, // only the first subband is selectable
                .scan_automation_period = 0,
            },
            .subbands[0] = (subband_t){
                .channel_index_start = 0,
                .channel_index_end = 0,
                .eirp = 10,
                .cca = -86,
                .duty = 0,
            }
        },
        {
            .channel_header = {
                .ch_coding = PHY_CODING_PN9,
                .ch_class = PHY_CLASS_LO_RATE,
                .ch_freq_band = PHY_BAND_433
            },
            .subprofiles[0] = {
                .subband_bitmap = 0x01, // only the first subband is selectable
                .scan_automation_period = 0,
            },
            .subbands[0] = (subband_t){
                .channel_index_start = 0,
                .channel_index_end = 0,
                .eirp = 10,
                .cca = -86,
                .duty = 0,
            }
        }
    };

    fs_init_args_t fs_init_args = (fs_init_args_t){
        .fs_user_files_init_cb = NULL,
        .access_profiles_count = sizeof(access_profiles) / sizeof(dae_access_profile_t),
        .access_profiles = access_profiles,
        .access_class = 0x01 // use access profile 0 and select the first subprofile
    };

    alp_init_args.alp_received_unsolicited_data_cb = &on_unsolicited_response_received;
    d7ap_stack_init(&fs_init_args, &alp_init_args, true, NULL);

#ifdef HAS_LCD
    lcd_write_string("GW %s", _GIT_SHA1);
#endif

#if HW_NUM_LEDS > 0
    sched_register_task(&led_blink_off);
#endif
}

