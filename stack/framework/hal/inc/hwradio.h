/* \file
 *
 * The interface specification for accessing the radio interface.
 * Please note: the currently defined functions are nothing more than
 * placeholders to test out the simulator. These functions WILL change
 * when we start working with the PHY
 *
 * @author Daniel van den Akker
 *
 */
#ifndef __HW_RADIO_H_
#define __HW_RADIO_H_

#include "link_c.h"
#include "errors.h"
#include "hal_defs.h"
#include "timer.h"

#define HW_RSSI_INVALID 0x7FFF

/** \brief The possible states the radio can be in
 *
 */
typedef enum
{
    HW_RADIO_STATE_IDLE,
    HW_RADIO_STATE_TX,
    HW_RADIO_STATE_RX
} hw_radio_state_t;

/** \brief spectrum id used to identify the spectrum settings
 *
 * This struct adheres to the 'Channel ID' format the Dash7 PHY layer. (@17/03/2015)
 */
typedef struct
{
    union
    {	
        uint8_t channel_header; 	/**< The raw (8-bit) channel header */
        struct
        {
            uint8_t ch_coding: 2; 	/**< The 'coding' field in the channel header */
            uint8_t ch_class: 2;  	/**< The 'class' field in the channel header */
            uint8_t ch_freq_band: 4;	/**< The frequency 'band' field in the channel header */
        };
    };
    uint8_t center_freq_index;		/**< The center frequency index of the channel id */
} channel_id_t;

/** \brief The type for the result of a 'hardware' crc check
 *
 */
typedef enum
{
    HW_CRC_VALID = 0,
    HW_CRC_INVALID = 1,
    HW_CRC_UNAVAILABLE = 2
} hw_crc_t;

/** \brief type of the 'syncword class'
 *
 */
typedef uint8_t    syncword_class_t;

/** \brief type of the 'eirp' used to transmit packets
 *
 */
typedef int8_t	   eirp_t;

/** \brief The 'RX Configuration' for the radio. 
 *
 * This struct contains various settings that are used to configure the radio.
 * It must be passed as a parameter to hw_radio_set_rx() and is also a part of the hw_rx_metadata attached
 * to the received packets.
 *
 **/
typedef struct
{
    channel_id_t channel_id; 		/**< The channel_id of the D7A 'channel' to which the radio is tuned */ 
    syncword_class_t syncword_class;	/**< The 'syncword' class used */
} hw_rx_cfg_t;

/** \brief The 'TX Configuration' to use when sending a packet.
 *
 * This struct contains settings to be applied before transmitting a packet. These settings are applied
 * on a per-packet basis and must be supplied as a parameter to hw_radio_send_packet(). The settings used are also stored in the
 * hw_tx_metadata attached to the packet upon completion of the transmission
 *
 */
typedef struct
{
    channel_id_t channel_id; 		/**< The channel_id of the D7A 'channel' on which to send the packet */
    syncword_class_t syncword_class;	/**< The 'syncword' class used */
    eirp_t eirp;			/**< The transmission power level measured in dBm [-39,+10]. If the 
					 *   If the value specified is not supported by the driver, 
                                         *   the nearest supported value is used instead
					 */    
} hw_tx_cfg_t;

/** \brief The metadata attached to a received packet.
 *
 */
typedef struct
{
#ifdef HAL_RADIO_INCLUDE_TIMESTAMP
    timer_tick_t timestamp; 	/**< The clock_tick of the framework timer at which the first bit of the sync word was received. */
#endif
    hw_rx_cfg_t rx_cfg;		/**< The 'RX Configuration' used to receive the packet. */
    
    uint8_t lqi;		/**< The link quality indicator (LQI) reported by the radio for the received packet*/	
    int16_t rssi;		/**< The Received signal strength (RSSI) reported by the radio for the received packet. */

    uint8_t crc_status;		/**< The crc status of the packet
    				 *
    				 * HW_CRC_UNAVAILABLE 	if the driver does not support hardware crc checking
    				 * HW_CRC_INVALID 	if the CRC was not valid
    				 * HW_CRC_VALID	if the CRC was valid 
    				 */

//this struct is "packed" so it occupies minimal space
//and to ensure that, the 'prefix' fields (including length) of hw_radio_packet_t
//are 32-bits alligned (that is sizeof(hw_radio_packet_t)%4 == 0)
} __attribute__ ((packed)) hw_rx_metadata_t; 

/** \brief The metadata an TX settings attached to a packet ready to be transmitted / that has been 
 *  transmitted.
 */
typedef struct
{
#ifdef HAL_RADIO_INCLUDE_TIMESTAMP
    timer_tick_t timestamp; 	/**< The clock_tick of the framework timer at which the first bit of the SFD was sent. */
#endif
    hw_tx_cfg_t tx_cfg;		/**< The 'TX Configuration' used to receive the packet. */
    
//this struct is "packed" so it occupies minimal space
//and to ensure that, the 'prefix' fields (including length) of hw_radio_packet_t
//are 32-bits alligned (that is sizeof(hw_radio_packet_t)%4 == 0)
} __attribute__ ((packed)) hw_tx_metadata_t;

/** \brief A PHY layer packet that can be sent / received over the air using the HW radio interface.
 *
 * A hw_radio_packet_t consists of:
 *   - <rx_meta> / <tx_meta>: 		The metadata attached to the packet that is either collected by the radio driver
 *					upon reception of the packet or needed/returned by the radio driver upon 
 *					transmission of the packet.
 *   - the packet data itself:		a uint8[] of undetermined length that contains the actual packet data.
 *					for convenience, the first byte of this data array overlaps with the 
 *					'length' field containing the length of the data array.
 *
 * It should be noted that the <rx_meta> and <tx_meta> structs occupy the same memory space (unioned)
 * And that they have been defined so the fields of these structs that have the same meaning overlap.
 * Moreover, the fields of thw hw_radio_packet_t structure are alligned such that the 'data' of a packet
 * is always in the same place, regardless of whether it is transmitted or received.
 *
 */
typedef struct
{
    union
    {
        hw_rx_metadata_t rx_meta;		/**< The TX Metadata of the packet */
        hw_tx_metadata_t tx_meta;		/**< The RX Metadata of the packet */
    };

    union
    {
        uint8_t		length;			/**< The length of the packet. This fields overlaps with the first byte of the packet data */
        struct
        {
            uint8_t	__resv[0];		//this is ONLY here to keep the compiler happy: DO NOT USE
            uint8_t	data[];			/**< The packet data. data[0] overlaps with the 'length' field */
        };
    };    
    //this struct is "packed" so it occupies minimal space
    //and to ensure that, the 'prefix' fields (including length) of hw_radio_packet_t
    //are 32-bits alligned (that is sizeof(hw_radio_packet_t)%4 == 0)
} __attribute__ ((packed)) hw_radio_packet_t;


/** \brief Type definition for the 'new_packet' callback function
 *
 * The new_packet_callback_t function is called by the PHY driver each time a (new) buffer is needed to store a 
 * packet. This function is supplied with the *length* of the packet to be stored and is expected to 
 * return a pointer to a 'hw_radio_packet_t' that is sufficiently large to contain a packet that is at least
 * <length> bytes long. If no sufficiently large buffer can be allocated, this function MUST return NULL 
 * (0x0).
 *
 * It should be noted that this function is typically called while the packet is being received and as a 
 * result it is imperative that this function does at little processing as possible. Also, this function may 
 * be called both from a 'thread' and from an interrupt context. Implementors are advised to use atomic 
 * sections (where needed) to protect against concurrency issues.
 *
 * Once a packet has been allocated, it remains under the control of the PHY driver until it is `released' 
 * by a call to either the release_packet_callback or the rx_packet_callback function.
 *
 * \param length		The length of the packet for which a buffer must be allocated
 * \return hw_radio_packet_t*	The allocated packet buffer. The buffer MUST be large enough for the
 *				data field to contain at least <length> bytes. If no sufficiently large 
 *				buffer can be allocated, 0x0 is returned.
 */
typedef hw_radio_packet_t* (*alloc_packet_callback_t)(uint8_t length);

/** \brief definition of the callback used by the PHY driver to 'release' control of a previously allocated 
 *	   packet buffer.
 *
 * This callback is called by the PHY driver when it determines that a previously allocated packet buffer is 
 * no longer needed (for instance because the RX was interrupted). If the packet WAS received correctly, 
 * control of the buffer is released through a call to rx_callback_packet_t.
 *
 * As with new_packet_callback_t, this function is called from an interrupt context during 
 * time-critical PHY layer processing. As a result, this function should do as little processing as possible.
 *
 * \param packet	A pointer to the hw_radio_packet to release
 * 
 */
typedef void (*release_packet_callback_t)(hw_radio_packet_t* packet);

/** \brief Type definition for the rx callback function
 *
 * The rx_packet_callback_t function is called by the PHY driver every time a new packet is received. This 
 * function is supplied with a pointer the hw_radio_packet containing the received packet.
 *
 * It should be noted that the `packet' pointer supplied to this callback function is *ALWAYS* a pointer that 
 * was obtained through a call to the new_packet_callback (new_packet_callback_t) function. By calling this 
 * function, the PHY driver explicitly releases control of the previously allocated buffer back to the radio stack.
 * This means that, from the perspective of the radio driver, calling rx_packet_callback_t has the same 
 * effect as calling release_packet_callback_t.
 *
 * As with new_packet_callback_t, this function is called from an interrupt context.
 * While this function is being executed, no other interrupts can fire and no other packets can be 
 * received. As a result, this function should do as little processing as possible.
 *
 * \param	packet		A pointer to the received packet
 *
 */
typedef void (*rx_packet_callback_t)(hw_radio_packet_t* packet);

/** \brief Type definition for the tx callback function
 *
 * The tx_packet_callback_t function is called by the PHY driver upon completion of a packet transmission. 
 * This function is supplied with a pointer to the transmitted packet (the packet supplied to the 
 * hw_radio_send_packet function)
 *
 * As with new_packet_callback_t, this function is called from an interrupt context and should therefore do 
 * as little processing as possible.
 *
 * \param packet		A pointer to the transmitted packet
 *
 */
typedef void (*tx_packet_callback_t)(hw_radio_packet_t* packet);


/** \brief Type definition for the rssi_valid callback function.
 *
 * The rssi_valid callback is called by the PHY driver every time the RSSI measurements of the 
 * radio become valid after the radio is put in RX mode (see hw_radio_set_rx).
 *
 * A callback to the rssi_valid function is not only triggered by a call to hw_radio_set_rx, but also
 * but also by a call to hw_radio_send_packet. In that case, rssi_valid is called when the radio enters RX 
 * mode after the packet has been transmitted. (Unless of course the radio was put in IDLE mode during tx).
 *
 * This function is called from an interrupt context and should therefore do as little processing as possible.
 *
 */
typedef void (*rssi_valid_callback_t)(int16_t cur_rssi);

/** \brief Initialise the radio driver. 
 *
 * After initialisation, the radio is in IDLE state. The RX must be explicitly enabled by a call to
 * hw_radio_set_rx(...) before any packets can be received.
 *
 * \param p_alloc			The new_packet_callback_t function to call whenever a buffer is 
 *					needed to store a new packet. Please note that this function is 
 *					called from an *interrupt* context and therefore can only do minimal 
 *					processing.
 *
 * \param p_free			The release_packet_callback_t function to call whenever an allocated 
 *					buffer is not needed any more. (A buffer can also be release by a 
 *					call to p_callback) Please note that this function is called from an 
 *					*interrupt* context and therefore can only do minimal processing.
 *
 * \param rx_callback			The rx_packet_callback_t function to call whenever a packet is received
 * 					please note that this function is called from an *interrupt* context and therefore
 * 					can only do minimal processing.
 *
 * \param tx_callback			The tx_packet_callback_t function to call whenever a packet has been 
 *					sent by the radio. Please note that this function is called from an 
 *					*interrupt* context and therefore can only do minimal processing.
 *
 * \param rssi_callback			The rssi_valid_callback_t function to call whenever the RSSI value 
 *					becomes valid after the radio enters RX mode. Please note that this 
 *					function is called from an *interrupt* context and therefore can only do 
 *					minimal processing.
 *
 * \return	error_t			SUCCESS  if the radio driver was initialised successfully
 * 							EINVAL	 if any of the callback functions is 0x0
 * 							EALREADY if the radio driver was already initialised
 * 							FAIL	 if the radio driver could not be initialised
 */
__LINK_C error_t hw_radio_init(alloc_packet_callback_t p_alloc,
			       release_packet_callback_t p_free, 
			       rx_packet_callback_t rx_callback,
			       tx_packet_callback_t tx_callback,
			       rssi_valid_callback_t rssi_callback
			       );

/** \brief Set the radio in the IDLE mode.
 *
 * When the radio is IDLE, the tranceiver is disabled to reduce energy consumption. 
 * While in this state, it is not possible to receive any packets. The tranceiver can 
 * be re-enabled by a call to hw_radio_set_rx().
 *
 * It should be noted that althoug packet reception is not possible while in IDLE mode,
 * it is possible to transmit packets (by calling hw_radio_send_packet). In that case the 
 * radio will go directly from IDLE to TX mode to transmit the packet and then go back to 
 * IDLE mode once the packet has been sent.
 *
 * If the radio is placed in IDLE mode while a packet is being transmitted, the transmisison
 * is completed before the radio is placed in IDLE mode. If the radio is placed in IDLE mode 
 * while a packet is being received, the reception of the current packet is interrupted and the 
 * radio is placed in IDLE mode IMMEDIATELY.
 *
 * \return error_t	SUCCESS if the radio was put in IDLE mode (or will be after the current TX has finished)
 *			EALREADY if the radio was already in IDLE mode
 *			EOFF if the radio is not yet initialised.
 *
 */
__LINK_C error_t hw_radio_set_idle();

/** \brief Check whether or not the radio is in IDLE mode.
 *
 * Please note that if hw_radio_set_idle() was called while a transmission was still in progress, 
 * hw_radio_is_idle() will return TRUE even if the transmission is still in progress.
 *
 * \return bool		true if the radio is in idle mode, false if it is not.
 */
__LINK_C bool hw_radio_is_idle();

/** \brief Set the radio in RX mode.
 *
 * When the radio is placed in RX mode, the tranceiver is configured according to the settings in the 
 * supplied hw_rx_cfg_t struct after which it starts scanning the channel for possible packets.
 *
 * Entering RX Mode triggers the rssi_valid_callback_t function supplied to the hw_radio_init function is called
 * once the RSSI value becomes valid.
 *
 * Received packets are passed back to the user by calling the rx_packet_callback_t
 * function supplied to the hw_radio_init function. 
 *
 * If the radio is already in RX mode when this function is called, any current packet receptions
 * are interrupted, the new settings are applied and the radio restarts the channel scanning process.
 *
 * If the radio is in TX mode when this function is called, the current packet transmission is completed 
 * before the radio is placed in RX mode.
 *
 * \param rx_cfg	A pointer the the rx settings to apply before entering RX mode.
 *
 * \return error_t	SUCCESS if the radio was put in RX mode (or will be after the current TX has finished)
 *			EALREADY if the radio was already in RX mode.
 *			EINVAL if the supplied rx_cfg contains invalid parameters.
 *			EOFF if the radio is not yet initialised.
 */
__LINK_C error_t hw_radio_set_rx(hw_rx_cfg_t const* rx_cfg);

/** \brief Check whether or not the radio is in RX mode.
 *
 * Please note that if hw_radio_set_rx() was called while a transmission was still in progress, 
 * hw_radio_is_rx() will return TRUE even if the transmission is still in progress.
 *
 *  \return bool	true if the radio is in RX mode, false if not.
 *
 */
__LINK_C bool hw_radio_is_rx();

/** \brief Initiate a packet transmission over the air with the specified TX settings.
 *
 * This function sends the packet pointed to by the <packet> parameter over the air. Packets are always 
 * transmitted using the tx_cfg settings in the tx_meta field of the supplied <packet>. If these settings are 
 * not valid, EINVAL is returned and the packet is not transmitted. Moreover the 'length' field of the packet 
 * must also be properly configured. More specifically, the radio will send the first 'length' bytes of the 
 * packet's 'data' buffer over the air. It is the responsibility of the caller to ensure that the 'length' of 
 * the packet has been properly configured.
 *
 * Packet transmission is always done *asynchronously*, that is: a packet transmission is initiated by a call 
 * to this function, but is not completed until the tx_packet_callback_t function supplied to the 
 * hw_radio_init function is invoked by the radio driver. The <packet> buffer supplied to this function *MUST* 
 * remain available until the transmission has been completed. It should be noted that the tx_callback 
 * function will ONLY be called if the hw_radio_send_packet function returns SUCCESS.
 *
 *
 * If a transmission is initiated while the radio is in IDLE mode, the radio switches directly from IDLE mode to 
 * TX mode to transmit the packet. After the packet has been sent, it switches back to IDLE mode (unless 
 * hw_radio_set_rx() is called while the TX is in progress).
 *
 * If a transmission is initiated while the radio is in RX mode, the radio switches immediately to TX mode to 
 * transmit the packet. If a packet reception is in progress while this function is called, this packet is 
 * dropped. Once the packet has been sent, the radio switches back to RX mode with the original hw_rx_cfg_t 
 * settings used (unless either hw_radio_set_idle() or hw_radio_set_rx() with different rx parameters) is 
 * called while the TX is still in progress.)
 *
 *
 * \param packet	A pointer to the start of the packet to be transmitted
 * \param tx_cfg	A pointer to the transmission settings to be used
 * \param metadata	A pointer to the location where to store the metadata. 0x0 if no metadata should be 
 *			kept.
 *
 * \return error_t	SUCCESS if the packet transmission has been successfully initiated.
 *			EINVAL if the tx_cfg parameter contains invalid settings
 *			ESIZE if the packet is either too long or too small
 *			EOFF if the radio has not yet been initialised
 */
__LINK_C error_t hw_radio_send_packet(hw_radio_packet_t* packet);

/** \brief Check whether or not the radio is currently transmitting a packet
 *
 *  \return bool	true if the radio is currently transmitting a packet, false if it is not.
 */
__LINK_C bool hw_radio_tx_busy();

/** \brief Check whether or not the radio is currently receiving a packet
 *
 *  \return bool	true if the radio is currently receiving a packet, false if it is not.
 */
__LINK_C bool hw_radio_rx_busy();

/** \brief Check whether the RSSI value measured by the PHY driver is valid or not.
 *
 * The RSSI will only be valid if the radio has been initialised and has been in RX mode long enough
 * for the RSSI to become valid (this is signalled by a callback to the rssi_valid_callback_t function)
 * 
 *  is in RX mode
 
  and the rssi_valid callback has beem
 *
 * In general the following rule applies:
 *  - hw_radio_rssi_valid() == (hw_radio_get_rssi() == HW_RSSI_INVALID)
 *
 *
 *
 */
bool hw_radio_rssi_valid();

/** \brief Measure the current RSSI on the channel.
 *
 * If the RSSI is not valid (hw_radio_rssi_valied() returns false), the special value 'HW_RSSI_INVALID' is 
 * returned. Otherwise the current RSSI is returned. The RSSI is measured in dBm and must be rounded to the 
 * nearest 16-bit signed integer value.
 *
 * If HW_RSSI_INVALID is returned, the caller shoud ensure that the radio is actually in RX mode and,
 * if so, either schedule the RSSI to be read at a later time or wait for the rssi_valid callback to be 
 * invoked.
 * 
 * Measuring the RSSI is done using the 'current' RX settings of the radio and does NOT interrupt any receive
 * operation currently in progress. 
 *
 * Reading the RSSI for a different RX configuration can be done by: 
 *   -# Setting the required settings by calling hw_radio_set_rx
 *   -# Waiting for the rssi_valid callback to be invoked.
 *
 * //NOTE: a function to set the rx_settings and then query the rssi in a busy wait until the rssi is valid 
 * is NOT included here *ON PURPOSE* because this would cause an infinite loop with the *SIM* environment
 *
 */
__LINK_C int16_t hw_radio_get_rssi();

#endif //__HW_RADIO_H_
