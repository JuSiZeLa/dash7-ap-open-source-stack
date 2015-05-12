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

/*! \file dll.h
 * \addtogroup DLL
 * \ingroup D7AP
 * @{
 * \brief This module implements the DLL layer of the D7AP.
 *
 * \author glenn.ergeerts@uantwerpen.be
 * \author maarten.weyn@uantwerpen.be
 */

#ifndef OSS_7_DLL_H
#define OSS_7_DLL_H

#include "hwradio.h"

typedef struct packet packet_t;

typedef struct
{
    uint8_t subnet;
    union
    {
        uint8_t control;
        struct
        {
            bool control_target_address_set: 1;
            bool control_vid_used: 1;
            int8_t control_eirp_index: 6;
        };
    };
    uint8_t target_address[8]; // TODO assuming 8B UID for now
} dll_header_t;

void dll_init();
void dll_tx_frame(packet_t* packet);
void dll_start_foreground_scan();
uint8_t dll_assemble_packet_header(packet_t* packet, uint8_t* data_ptr);

#endif //OSS_7_DLL_H

/** @}*/
