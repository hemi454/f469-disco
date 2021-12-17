/**
 * @file       usbconnection.c
 * @brief      MicroPython uscard usb module: UsbReader class
 * @author     Rostislav Gurin <rgurin@embedity.dev>
 * @copyright  Copyright 2021 Crypto Advance GmbH. All rights reserved.
 */

#include "usbconnection.h"
#define USB_DEBUG

/// Minimal number of bytes for a valid ATR
#define ATR_MIN_BYTES                   2
/// TS0 byte: direct convention
#define TS_CONVENTION_DIRECT            0x3B
/// TS0 byte: inverse convention
#define TS_CONVENTION_INVERSE           0x3F
/// Bit within T0 or TD bytes indicating that TA is present
#define TA_BIT                          (1)
/// Bit within T0 or TD bytes indicating that TB is present
#define TB_BIT                          (1 << 1)
/// Bit within T0 or TD bytes indicating that TC is present
#define TC_BIT                          (1 << 2)
/// Bit within T0 or TD bytes indicating that TD is present
#define TD_BIT                          (1 << 3)
/// LRC size in bytes
#define LRC_SIZE                        1
/// CRC size in bytes
#define CRC_SIZE                        2

/// Transmission protocol or qualification of interface bytes
typedef enum {
  atr_prot_t0 = 0,  ///< T=0 protocol
  atr_prot_t1 = 1,  ///< T=1 protocol
  atr_globals = 15, ///< Global interface bytes
} protocol_id_t;

static void card_detection_task(usb_connection_obj_t* self,
                                USBH_HandleTypeDef* phost);
static bool card_present(usb_connection_obj_t* self, USBH_HandleTypeDef* phost);
STATIC void connection_ccid_receive(USBH_HandleTypeDef* phost, uint8_t* pbuff,
                                    uint32_t length);
STATIC void connection_ccid_transmit_raw_extended_apdu(usb_connection_obj_t* self,
                                         USBH_HandleTypeDef* phost,
                                         uint8_t* pbuff, uint32_t length);
STATIC void  connection_ccid_receive_extended_apdu_first_block(usb_connection_obj_t* self);
STATIC void connection_ccid_receive_extended_apdu_next_block(usb_connection_obj_t* self, uint8_t length, uint8_t offset);
STATIC void connection_ccid_transmit_raw(usb_connection_obj_t* self,
                                         USBH_HandleTypeDef* phost,
                                         uint8_t* pbuff, uint32_t length);
static inline void wait_response_blocking_extended_apdu(usb_connection_obj_t* self);
/**
 * @brief Constructor of UsbCardConnection
 *
 * Constructor
 * .. class:: UsbCardConnection(usbreader, connParams)
 *
 *  Constructs a connection object
 *
 *    - *usbreader* reader to which connection is bound, instance of Reader class
 *    - *connParams* connection parameters provided by the reader
 *
 * CardConnection object must not be constructed by user! Use
 * UsbReader.createConnection instead.
 *
 * @param type      pointer to type structure
 * @param n_args    number of arguments
 * @param n_kw      number of keyword arguments
 * @param all_args  array containing arguments
 * @return          a new instance of CardConnection class
 */
STATIC mp_obj_t connection_make_new(const mp_obj_type_t* type, size_t n_args,
                                    size_t n_kw, const mp_obj_t* all_args) {
  // Create a new connection object
  // Arguments
  enum {
    ARG_reader = 0,
  };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_reader, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
  };
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];

  // Parse and check parameters
  mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args),
                            allowed_args, args);
  if (args[ARG_reader].u_obj == MP_OBJ_NULL ||
      !mp_obj_is_type(args[ARG_reader].u_obj, &scard_UsbReader_type)) {
    raise_SmartcardException("connection not bound to usb reader");
  }

  usb_connection_obj_t* self = m_new_obj_with_finaliser(usb_connection_obj_t);
  memset(self, 0U, sizeof(usb_connection_obj_t));
  self->base.type = &scard_UsbCardConnection_type;
  self->reader = args[ARG_reader].u_obj;
  self->state = state_closed;
  self->state_ext_apdu = state_closed;
  self->CCID_Handle = NULL;
  self->timer = MP_OBJ_NULL;
  self->pbSeq = 0;
  self->protocol = NULL;
  self->proto_handle = NULL;
  self->observers = mp_obj_new_list(0, NULL);
  self->atr_timeout_ms = proto_prm_unchanged;
  self->rsp_timeout_ms = proto_prm_unchanged;
  self->max_timeout_ms = proto_prm_unchanged;
  self->raise_on_error = false;
  self->event_idx = 0;
  self->atr = MP_OBJ_NULL;
  self->response = MP_OBJ_NULL;
  self->next_protocol = protocol_na;
  self->presence_cycles = 0;
  self->presence_state = false;
  self->blocking = true;
  self->process_state = process_state_closed;
  self->processTimer = 150;
  self->dwFeatures = 0;
  self->TA_1 = 0x11;
  hUsbHostFS.iccSlotStatus = ICC_INIT;
  hUsbHostFS.procStatus = PROC_INACT;

  usb_timer_init(self);
  printf("\r\nNew USB smart card connection\n");
  return MP_OBJ_FROM_PTR(self);
}

/**
 * Returns minimal of two uint32_t operands
 * @param a  operand A
 * @param b  operand B
 * @return   minimal of A and B
 */
static inline uint32_t min_u32(uint32_t a, uint32_t b) { return a < b ? a : b; }

/**
 * Decreases timer counter and checks if it is elapsed
 *
 * This function also ensures that it is called at least twice before indicating
 * that timer is elapsed. It is done as protection measure against abnormally
 * quick timeout event in case timer task is called right after the timer
 * variable is initialized. As side effect, this function supports timeouts not
 * longer than 0x7FFFFFFF.
 * @param p_timer     pointer to timer variable
 * @param elapsed_ms  time in milliseconds passed since previous call
 * @return            true is timer is elapsed
 */
static bool connection_timer_elapsed(uint32_t* p_timer, uint32_t elapsed_ms) {
  if (*p_timer) {
    uint32_t time = *p_timer & 0x7FFFFFFFUL;
    if (!(time -= min_u32(elapsed_ms, time))) {
      if (*p_timer & 0x80000000UL) {  // Not first call?
        *p_timer = 0;
        return true;
      }
    }
    // Set higher bit to check for non-first call later
    *p_timer = time | 0x80000000UL;
  }
  return false;
}

/**
 * Returns length of a list
 *
 * @param list  list
 * @return      list length
 */
static size_t list_get_len(mp_obj_t list) {
  if (list != MP_OBJ_NULL) {
    size_t len;
    mp_obj_t* items;
    mp_obj_list_get(list, &len, &items);
    return len;
  }
  return 0U;
}

/**
 * Creates event and stores it in buffer
 *
 * @param self  instance of UsbCardConnection class
 * @param args  arguments of event passed to observer
 * @param n_kw  number of keyword arguments
 */
static void create_event(usb_connection_obj_t* self, mp_obj_t* args,
                         size_t n_kw) {
  if (self->event_idx + 1U > MAX_EVENTS || n_kw > EVENT_MAX_KW) {
    raise_SmartcardException("event buffer overflow");
  }

  // If we have any observers add event to buffer
  if (list_get_len(self->observers)) {
    event_t* p_event = self->event_buf + self->event_idx;
    for (size_t i = 0U; i < EVENT_POS_ARGS + 2U * n_kw; ++i) {
      p_event->args[i] = args[i];
    }
    p_event->n_kw = n_kw;

    // Schedule call of connection_notifyAll() on the first event
    if (++self->event_idx == 1U) {
      mp_obj_t handler = mp_load_attr(self, MP_QSTR__notifyAll);
      mp_sched_schedule(handler, mp_const_none);
    }
  }
}

/**
 * Notifies observers passing only type arguments
 *
 * @param self        instance of UsbCardConnection class
 * @param event_type  type argument
 */
static void notify_observers(usb_connection_obj_t* self,
                             event_type_t event_type) {
  mp_obj_t args[] = {
      MP_OBJ_FROM_PTR(self),             // connection
      MP_OBJ_NEW_QSTR((qstr)event_type)  // type
  };
  create_event(self, args, 0U);
}

/**
 * Notifies observers passing type and text arguments
 *
 * @param self        instance of UsbCardConnection class
 * @param event_type  type argument
 * @param text        text argument
 */
static void notify_observers_text(usb_connection_obj_t* self,
                                  event_type_t event_type, const char* text) {
  mp_obj_t text_obj = mp_obj_new_str(text, strlen(text));
  mp_obj_t args_list = mp_obj_new_list(1U, &text_obj);
  mp_obj_t args[] = {
      MP_OBJ_FROM_PTR(self),                                      // connection
      MP_OBJ_NEW_QSTR((qstr)event_type),                          // type
      MP_OBJ_NEW_QSTR(MP_QSTR_args), MP_OBJ_FROM_PTR(args_list),  // args
  };
  create_event(self, args, 1U);
}

/**
 * Notifies observers passing smart card command
 *
 * @param self   instance of UsbCardConnection class
 * @param bytes  command data
 */
static void notify_observers_command(usb_connection_obj_t* self,
                                     mp_obj_t bytes) {
  // Make a list with arguments [bytes, protocol]
  mp_obj_list_t* args_list = mp_obj_new_list(2U, NULL);
  args_list->items[0] = bytes;  // bytes

  mp_obj_t args[] = {
      MP_OBJ_FROM_PTR(self),                                      // connection
      MP_OBJ_NEW_QSTR((qstr)event_command),                       // type
      MP_OBJ_NEW_QSTR(MP_QSTR_args), MP_OBJ_FROM_PTR(args_list),  // args
  };
  create_event(self, args, 1U);
}

/**
 * Notifies observers passing smart card response
 *
 * @param self      instance of UsbCardConnection class
 * @param response  smart card response list
 */
static void notify_observers_response(usb_connection_obj_t* self,
                                      mp_obj_t response) {
  mp_obj_t args[] = {
      MP_OBJ_FROM_PTR(self),                                     // connection
      MP_OBJ_NEW_QSTR((qstr)event_response),                     // type
      MP_OBJ_NEW_QSTR(MP_QSTR_args), MP_OBJ_FROM_PTR(response),  // args
  };
  create_event(self, args, 1U);
}

/**
 * Creates a new response list from raw data
 *
 * @param data  smart card response data
 * @param len   number of data bytes
 * @return      list object [data, sw1, sw2]
 */
static mp_obj_t make_response_list(const uint8_t* data, size_t len) {
  mp_obj_list_t* response = MP_OBJ_NULL;

  if (len >= 2U) {
    response = mp_obj_new_list(3U, NULL);
    response->items[0] = mp_obj_new_bytes(data, len - 2U);      // data
    response->items[1] = MP_OBJ_NEW_SMALL_INT(data[len - 2U]);  // sw1
    response->items[2] = MP_OBJ_NEW_SMALL_INT(data[len - 1U]);  // sw2
  } else {
    response = mp_obj_new_list(1U, NULL);
    response->items[0] = mp_obj_new_bytes(data, len);  // data
  }

  return MP_OBJ_TO_PTR(response);
}

/**
 * __call__ special method running background tasks, usually by timer
 * @param self_in  instance of CardConnection class casted to mp_obj_t
 * @param n_args   number of arguments
 * @param n_kw     number of keywords arguments
 * @param args     array containing arguments
 * @return         None
 */
STATIC mp_obj_t connection_call(mp_obj_t self_in, size_t n_args, size_t n_kw,
                                const mp_obj_t* args) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  timer_task(self);
  return mp_const_none;
}

/**
 * Timer task
 *
 * @param self  instance of UsbCardConnection class
 */

static void timer_task(usb_connection_obj_t* self) {
  mp_uint_t ticks_ms = mp_hal_ticks_ms();
  mp_uint_t elapsed = scard_ticks_diff(ticks_ms, self->prev_ticks_ms);
  self->prev_ticks_ms = ticks_ms;
  if (connection_timer_elapsed(&self->processTimer, elapsed)) {
    USBH_Process(&hUsbHostFS);
    if (hUsbHostFS.gState == HOST_CLASS && hUsbHostFS.procStatus == PROC_ACT) {
      // Check smart card slot status
      if(hUsbHostFS.iccSlotStatus == ICC_INSERTED || hUsbHostFS.iccSlotStatus == ICC_REMOVED){
        self->process_state = process_state_ready;
      }
    } else {
      self->process_state = process_state_init;
    }
    card_detection_task(self, &hUsbHostFS);
    self->processTimer = 50;
  }
  // Run protocol timer task only when we are connecting or connected
  if (state_connecting == self->state || state_connected == self->state) {
    if (self->protocol) {
      self->protocol->timer_task(self->proto_handle, (uint32_t)elapsed);
    }
  }
}

/**
 * Creates a new machine.Timer object for background tasks
 * @param self      instance of CardConnection class
 * @param timer_id  numerical timer identifier
 * @return          new instance of machine.Timer
 */
static mp_obj_t create_timer(usb_connection_obj_t* self, mp_int_t timer_id) {
  mp_obj_t args[] = {                                 // Positional arguments
                     MP_OBJ_NEW_SMALL_INT(timer_id),  // id
                     // Keyword arguments
                     MP_OBJ_NEW_QSTR(MP_QSTR_callback), self,
                     MP_OBJ_NEW_QSTR(MP_QSTR_period),
                     MP_OBJ_NEW_SMALL_INT(TIMER_PERIOD_MS)};
  return machine_timer_type.make_new(&machine_timer_type, 1, 2, args);
}

/**
 * Init a new timer. Timer object for background tasks
 * @param self      instance of UsbCardConnection class
 */
STATIC void usb_timer_init(usb_connection_obj_t* self) {
  mp_int_t timer_id = -1;
  // Save system ticks and initialize timer
  self->prev_ticks_ms = mp_hal_ticks_ms();
  self->timer = create_timer(self, timer_id);
}

/**
 * Converts objects to byte buffer
 *
 * @param dst        destination buffer of bytes
 * @param objects    source array of objects
 * @param n_objects  number of objects to convert
 * @return           true if successful
 */
static bool objects_to_buf(uint8_t* buf, const mp_obj_t* objects,
                           size_t n_objects) {
  const mp_obj_t* p_obj = objects;
  const mp_obj_t* arr_end = objects + n_objects;
  uint8_t* p_dst = buf;
  mp_int_t value = 0;

  while (p_obj != arr_end) {
    if (mp_obj_get_int_maybe(*p_obj++, &value) && value >= 0U &&
        value <= UINT8_MAX) {
      *p_dst++ = (uint8_t)value;
    } else {
      return false;
    }
  }

  return true;
}

/**
 * Get voltage from CCID device descriptor
 *
 * @param ccidDescriptor  CCID device descriptor
 * @return                supported voltage
 */
static uint8_t getVoltageSupport(USBH_ChipCardDescTypeDef* ccidDescriptor) {
  uint8_t voltage = 0;
  if ((ccidDescriptor->dwFeatures & CCID_CLASS_AUTO_VOLTAGE) ||
      (ccidDescriptor->dwFeatures & CCID_CLASS_AUTO_ACTIVATION))
    voltage = 0; /* automatic voltage selection */
  else {
    if (ccidDescriptor->bVoltageSupport == 0x01 ||
        ccidDescriptor->bVoltageSupport == 0x07) {
      // 5V
      voltage = 0x01;
    }

    if (ccidDescriptor->bVoltageSupport == 0x02) {
      // 3V
      voltage = 0x02;
    }

    if (ccidDescriptor->bVoltageSupport == 0x04) {
      // 1.8V
      voltage = 0x03;
    }
  }
  return voltage;
}
/**
 * Converts integer value  to double word
 *
 * @param value        integer value
 * @param buffer       array of words
 */
static void i2dw(int value, uint8_t buffer[]) {
  buffer[0] = value & 0xFF;
  buffer[1] = (value >> 8) & 0xFF;
  buffer[2] = (value >> 16) & 0xFF;
  buffer[3] = (value >> 24) & 0xFF;
}

/**
 * Prepare PC_to_RDR_XfrBlock
 *
 * @param self       instance of CardConnection class
 * @param tx_buffer  transmit buffer
 * @param tx_length  transmit buffer length
 * @param cmd        CCID command
 * @param rx_length  expected length, in character mode only
 * @param bBWI extend block waiting timeout
 */
STATIC void connection_prepare_xfrblock(mp_obj_t self_in, uint8_t* tx_buffer,
                                        unsigned int tx_length, uint8_t* cmd,
                                        unsigned short rx_length,
                                        uint8_t bBWI) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  USBH_ChipCardDescTypeDef chipCardDesc =
      hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
  cmd[ccid_xfrblock_cmd_code] = 0x6F;                           /* XfrBlock */
  i2dw(tx_length, cmd + ccid_xfrblock_apdu_len);                /* APDU length */
  cmd[ccid_xfrblock_cmd_slot] = chipCardDesc.bCurrentSlotIndex; /* slot number */
  cmd[ccid_xfrblock_cmd_pbseq] = self->pbSeq++;
  cmd[ccid_xfrblock_cmd_bwi] = bBWI;             /* extend block waiting timeout */
  cmd[ccid_xfrblock_cmd_wlevelparam_0] = rx_length & 0xFF; /* Expected length, in character mode only */
  cmd[ccid_xfrblock_cmd_wlevelparam_1] = (rx_length >> 8) & 0xFF;
  memcpy(cmd + ccid_xfrblock_cmd_cmd_hdr_len, tx_buffer, tx_length);
}

/**
 * Send PC_to_RDR_GetParameters command
 *
 * @param self       instance of CardConnection class
 * @param phost      instance of USB host handle structure
 */
STATIC void connection_ccid_transmit_get_parameters(usb_connection_obj_t* self,
                                                    USBH_HandleTypeDef* phost) {
  USBH_ChipCardDescTypeDef chipCardDesc =
      hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
  uint8_t cmd[ccid_getparam_cmd_hdr_len];
  cmd[ccid_getparam_cmd_code] = 0x6C; /* GetParameters */
  cmd[ccid_getparam_cmd_dwlength_0] = cmd[ccid_getparam_cmd_dwlength_1] = cmd[ccid_getparam_cmd_dwlength_2] = cmd[ccid_getparam_cmd_dwlength_3] = 0;
  cmd[ccid_getparam_cmd_slot] = chipCardDesc.bCurrentSlotIndex; /* slot number */
  cmd[ccid_getparam_cmd_pbseq] = self->pbSeq++;
  cmd[ccid_getparam_cmd_rfu_1] = cmd[ccid_getparam_cmd_rfu_2] = cmd[ccid_getparam_cmd_rfu_3] = 0; /* RFU */
  USBH_CCID_Transmit(phost, cmd, sizeof(cmd));
  CCID_ProcessTransmission(phost);
}

/**
 * Send PC_to_RDR_SetParameters command
 *
 * @param self       instance of CardConnection class
 * @param phost      instance of USB host handle structure
 */
STATIC void connection_ccid_transmit_set_parameters(usb_connection_obj_t* self,
                                                    USBH_HandleTypeDef* phost) {
  USBH_ChipCardDescTypeDef chipCardDesc =
      hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
  uint8_t param[] = {
      self->TA_1, /* Fi/Di		  */
      0x10,       /* TCCKS		  */
      0x00,       /* GuardTime	*/
      0x4D,       /* BWI/CWI		*/
      0x00,       /* ClockStop	*/
      0x10,       /* IFSC			  */
      0x00        /* NADValue	  */
  };
  uint8_t cmd[ccid_setparam_cmd_hdr_len + sizeof(param)];
  cmd[ccid_setparam_cmd_code] = 0x61;                           /* SetParameters */
  i2dw(sizeof(param), cmd + ccid_setparam_apdu_len);            /* APDU length */
  cmd[ccid_setparam_cmd_slot] = chipCardDesc.bCurrentSlotIndex; /* slot number */
  cmd[ccid_setparam_cmd_pbseq] = self->pbSeq++;
  cmd[ccid_setparam_protocol_num] = 0x01;       /* bProtocolNum */
  cmd[ccid_setparam_cmd_rfu_0] = cmd[ccid_setparam_cmd_rfu_1] = 0; /* RFU */
  memcpy(cmd + ccid_setparam_cmd_hdr_len, param, sizeof(param));
  if ((self->dwFeatures & CCID_CLASS_SHORT_APDU) ||
    (self->dwFeatures & CCID_CLASS_EXTENDED_APDU))
  {
    connection_ccid_transmit_raw_extended_apdu(self, phost, cmd, sizeof(cmd));
  }
  else
  {
    USBH_CCID_Transmit(phost, cmd, sizeof(cmd));
    CCID_ProcessTransmission(phost);
  }
}

/**
 * Send PC_to_RDR_XfrBlock command
 *
 * @param self       instance of CardConnection class
 * @param pbuff      buffer to transmit
 * @param length     length of the buffer
 * @param phost      instance of USB host handle structure
 */
STATIC void connection_ccid_transmit_xfr_block(usb_connection_obj_t* self,
                                               USBH_HandleTypeDef* phost,
                                               uint8_t* pbuff,
                                               uint32_t length) {
  uint8_t cmd[CCID_ICC_HEADER_LENGTH + CCID_MAX_DATA_LENGTH] = {0};
  connection_prepare_xfrblock(self, pbuff, length, cmd, 0, 0);
  USBH_CCID_Transmit(phost, cmd, CCID_ICC_HEADER_LENGTH + length);
  CCID_ProcessTransmission(phost);
}

STATIC void connection_ccid_process_exchange_extended_apdu(usb_connection_obj_t* self,
                                         USBH_HandleTypeDef* phost,
                                         uint8_t* pbuff, uint32_t length) {
  uint8_t cmd[CCID_ICC_HEADER_LENGTH + CCID_MAX_DATA_LENGTH] = {0};
  uint8_t offset = 0;
  uint32_t apduLength = length + CCID_ICC_HEADER_LENGTH;
  uint16_t dwLengthSend = 0;
  bool sendLastBlock = false;
  if(apduLength <= self->CCID_Handle->DataItf.OutEpSize){
    connection_prepare_xfrblock(self, pbuff, length, cmd, 0, 0);
    connection_ccid_transmit_raw(self, phost, cmd, CCID_ICC_HEADER_LENGTH + length);
    connection_ccid_receive_extended_apdu_first_block(self); 
  }
  else {
    // Send the first block of the APDU message
    offset = apduLength - self->CCID_Handle->DataItf.OutEpSize;
    uint8_t fistBlockLength = self->CCID_Handle->DataItf.OutEpSize - CCID_ICC_HEADER_LENGTH;
    connection_prepare_xfrblock(self, pbuff, fistBlockLength, cmd, 0x01, 0);
    connection_ccid_transmit_raw(self, phost, cmd, self->CCID_Handle->DataItf.OutEpSize);
    // Receive the Reader_to_Pc_XfrBlock which informs us that we need to send the next block of our message
    connection_ccid_receive_extended_apdu_first_block(self);
    memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
    memset(cmd, 0, sizeof(cmd));
    // Send the next block of the APDU message
    uint8_t frameLength = 0;
    if(offset + CCID_ICC_HEADER_LENGTH > self->CCID_Handle->DataItf.OutEpSize){
      frameLength = length - fistBlockLength;
      while(1)
      {
        pbuff += fistBlockLength;
        connection_prepare_xfrblock(self, pbuff, fistBlockLength, cmd, 0x03, 0);
        connection_ccid_transmit_raw(self, phost, cmd, self->CCID_Handle->DataItf.OutEpSize);
        connection_ccid_receive_extended_apdu_first_block(self);
        memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
        memset(cmd, 0, sizeof(cmd));
        frameLength -= fistBlockLength;
        if(frameLength <= fistBlockLength)
        {
          sendLastBlock = true;
          break;
        }
      }
    }
    // Send the last block of the APDU message
    if(offset + CCID_ICC_HEADER_LENGTH <= self->CCID_Handle->DataItf.OutEpSize){
      connection_prepare_xfrblock(self, pbuff + fistBlockLength, offset, cmd, 0x02, 0);
      connection_ccid_transmit_raw(self, phost, cmd, CCID_ICC_HEADER_LENGTH + offset);
    }
    else{
      connection_prepare_xfrblock(self, pbuff, frameLength, cmd, 0x02, 0);
      connection_ccid_transmit_raw(self, phost, cmd, CCID_ICC_HEADER_LENGTH + frameLength);
    }

    // Receive first block of the message
    connection_ccid_receive_extended_apdu_first_block(self);
    //Receive last block of the message
    uint16_t receivedMessageLength = dw2i(hUsbHostFS.rawRxData, 1) + CCID_ICC_HEADER_LENGTH;
    if(receivedMessageLength > self->CCID_Handle->DataItf.InEpSize)
    {
      uint32_t remainingLength = receivedMessageLength - self->CCID_Handle->DataItf.InEpSize;
      if(remainingLength < self->CCID_Handle->DataItf.InEpSize){
        connection_ccid_receive_extended_apdu_next_block(self, remainingLength, self->CCID_Handle->DataItf.InEpSize);
      }
      else {
        uint8_t packetCounter = remainingLength / self->CCID_Handle->DataItf.InEpSize;
      }
    }
  }
  t1_apdu_t apdu_received;
  mp_obj_t response = make_response_list(hUsbHostFS.rawRxData + CCID_ICC_HEADER_LENGTH, dw2i(hUsbHostFS.rawRxData, 1));
  notify_observers_response(self, response);
  if (self->blocking) {
    self->response = response;
  }
}                                           

STATIC void connection_ccid_transmit_xfr_block_extended_apdu(usb_connection_obj_t* self,
                                         USBH_HandleTypeDef* phost,
                                         uint8_t* pbuff, uint32_t length) {
  uint8_t cmd[CCID_ICC_HEADER_LENGTH + CCID_MAX_DATA_LENGTH] = {0};
  uint8_t offset = 0;
  uint32_t apduLength = length + CCID_ICC_HEADER_LENGTH;
  uint8_t rcv[64];
  if(apduLength <= self->CCID_Handle->DataItf.OutEpSize){
    connection_prepare_xfrblock(self, pbuff, length, cmd, 0, 0);
    USBH_CCID_Transmit(phost, cmd, CCID_ICC_HEADER_LENGTH + length);
    CCID_ProcessTransmission(phost);
  }
  else {
    // Send the first block of the APDU message
    offset = apduLength - self->CCID_Handle->DataItf.OutEpSize;
    uint8_t fistBlockLength = length - offset;
    connection_prepare_xfrblock(self, pbuff, fistBlockLength, cmd, 0x01, 0);
    USBH_CCID_Transmit(phost, cmd, self->CCID_Handle->DataItf.OutEpSize);
    CCID_ProcessTransmission(phost);
    connection_ccid_receive_extended_apdu_first_block(self); 
    memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
    memset(cmd, 0, sizeof(cmd));
    if(offset < self->CCID_Handle->DataItf.OutEpSize){
      connection_prepare_xfrblock(self, pbuff + fistBlockLength, offset, cmd, 0x02, 0);
      USBH_CCID_Transmit(phost, cmd, CCID_ICC_HEADER_LENGTH + offset);
      CCID_ProcessTransmission(phost);
      connection_ccid_receive_extended_apdu_first_block(self);
      uint16_t dwLength = dw2i(hUsbHostFS.rawRxData, 1);
      if(dwLength > self->CCID_Handle->DataItf.OutEpSize - CCID_ICC_HEADER_LENGTH)
      {
        connection_ccid_receive_extended_apdu_next_block(self, 0x2, self->CCID_Handle->DataItf.OutEpSize);
      }
      t1_apdu_t apdu_received;
      mp_obj_t response = make_response_list(hUsbHostFS.rawRxData + CCID_ICC_HEADER_LENGTH, dwLength);
      notify_observers_response(self, response);
      if (self->blocking) {
        self->response = response;
      }
    }
  }   
}                                           

STATIC void connection_ccid_transmit_raw_extended_apdu(usb_connection_obj_t* self,
                                         USBH_HandleTypeDef* phost,
                                         uint8_t* pbuff, uint32_t length) {
  uint8_t offset = 0;
  USBH_URBStateTypeDef URB_Status = USBH_URB_IDLE;
  uint32_t messageLength = length;
  uint8_t buf[32] = {0};
  if(length > self->CCID_Handle->DataItf.InEpSize)
  {
    while(1){
      USBH_CCID_Transmit(phost, pbuff + offset, self->CCID_Handle->DataItf.OutEpSize);
      CCID_ProcessTransmission(phost);
      length -= self->CCID_Handle->DataItf.OutEpSize;
      offset += self->CCID_Handle->DataItf.OutEpSize;
      mp_hal_delay_ms(150);
      if(length < self->CCID_Handle->DataItf.OutEpSize)
      {
        memcpy(buf, pbuff + offset, length);        
        USBH_CCID_Transmit(phost, buf, length);
        CCID_ProcessTransmission(phost);
        printf("URB_Status: %d\n", URB_Status);
        break;
      }
    }
  }
  else
  {
    USBH_CCID_Transmit(phost, pbuff, length);
    CCID_ProcessTransmission(phost);
    mp_hal_delay_ms(150);
  }
}

/**
 * Send raw bytes to the USB port
 *
 * @param self       instance of CardConnection class
 * @param pbuff      buffer to transmit
 * @param length     length of the buffer
 * @param phost      instance of USB host handle structure
 */
STATIC void connection_ccid_transmit_raw(usb_connection_obj_t* self,
                                         USBH_HandleTypeDef* phost,
                                         uint8_t* pbuff, uint32_t length) {
  USBH_CCID_Transmit(phost, pbuff, length);
  CCID_ProcessTransmission(phost);
}

static bool isBufNonEmpty(uint8_t* buf)
{
  unsigned int sum = 0;
  for(int i = 0; i < sizeof(buf); i++)
  {
    sum = sum + buf[i];
  }
  if(sum > 0)
    return true;
  return false;
}

STATIC void connection_ccid_receive_extended_apdu_first_block(usb_connection_obj_t* self)
{
  uint32_t responseTimeout = 15000;
  while(1)
  {
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData, self->CCID_Handle->DataItf.InEpSize);
    mp_uint_t ticks_ms = mp_hal_ticks_ms();
    mp_uint_t elapsed = scard_ticks_diff(ticks_ms, self->prev_ticks_ms);
    self->prev_ticks_ms = ticks_ms;
    if (hUsbHostFS.rawRxData[0] == RDR_to_PC_DataBlock || hUsbHostFS.rawRxData[0] ==  RDR_to_PC_Parameters){
      break;
    }
    if (connection_timer_elapsed(&responseTimeout, elapsed)){
      raise_SmartcardException("response timeout is elapsed");
      break;
    }
  }
}

STATIC void connection_ccid_receive_extended_apdu_next_block(usb_connection_obj_t* self, uint8_t length, uint8_t offset)
{
  uint32_t responseTimeout = 15000;
  while(1)
  {
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData + offset,
                          length);
    mp_uint_t ticks_ms = mp_hal_ticks_ms();
    mp_uint_t elapsed = scard_ticks_diff(ticks_ms, self->prev_ticks_ms);
    self->prev_ticks_ms = ticks_ms;
    if (connection_timer_elapsed(&responseTimeout, elapsed)){
      break;
    }
    else {
      if (isBufNonEmpty(hUsbHostFS.rawRxData + offset)){
       break;
      }
    }
  }
}

/**
 * Receive packet from the USB port
 *
 * @param self       instance of CardConnection class
 * @param pbuff      buffer to receive
 * @param length     length of the buffer
 * @param phost      instance of USB host handle structure
 */
STATIC void connection_ccid_receive(USBH_HandleTypeDef* phost, uint8_t* pbuff,
                                    uint32_t length) {
  USBH_CCID_Receive(phost, pbuff, length);
  CCID_ProcessReception(phost);
}

/**
 * Waits for connection to complete in blocking mode
 *
 * @param self  instance of CardConnection class
 */
static inline void wait_connect_blocking(usb_connection_obj_t* self) {
  uint8_t rx_buf[254] = {0};
  while (self->state == state_connecting) {
    memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                            sizeof(hUsbHostFS.rawRxData));
    mp_hal_delay_ms(150);
    uint16_t length = USBH_LL_GetLastXferSize(&hUsbHostFS, self->CCID_Handle->DataItf.InPipe);
    if (length != 0) {
      memset(rx_buf, 0, sizeof(rx_buf));
      memcpy(rx_buf, hUsbHostFS.rawRxData + CCID_ICC_HEADER_LENGTH,
             length - CCID_ICC_HEADER_LENGTH);
      self->protocol->serial_in(self->proto_handle, rx_buf,
                                length - CCID_ICC_HEADER_LENGTH);
      timer_task(self);
      MICROPY_EVENT_POLL_HOOK
    }
  }
}

/**
 * Initializes ATR structure with default values
 * @param p_atr    pointer to ATR structure to initialize
 * @param atr_buf  buffer containing ATR
 * @param atr_len  length of ATR in bytes
 */
static inline void atr_decoded_init(t1_atr_decoded_t* p_atr,
                                    const uint8_t* atr_buf, size_t atr_len) {
  p_atr->atr = atr_buf;
  p_atr->atr_len = atr_len;
  p_atr->convention = t1_cnv_direct;
  p_atr->t0_supported = false;
  p_atr->t1_supported = false;
  p_atr->hist_bytes = NULL;
  p_atr->hist_nbytes = 0;
  for(int i = 0; i < t1_atr_intf_bytes; i++) {
    p_atr->global_bytes[i] = -1;
    p_atr->t1_bytes[i] = -1;
  }
}

/**
 * Returns number of interface bytes from given indicator Y
 * @param y  indicator, "Y"
 * @return   number of interface bytes
 */
static inline size_t atr_ibyte_num(uint8_t y) {
  return ((y >> 3) & 1) + ((y >> 2) & 1) + ((y >> 1) & 1) + (y & 1);
}

/**
 * Parses ATR message
 *
 * Limitations: only direct convention is supported, TS byte is ignored
 * @param buf    buffer containing ATR
 * @param len    length of ATR in bytes
 * @param p_atr  pointer to ATR structure filled by this function
 * @return       true if successful
 */
static bool connection_parse_atr_extended_apdu(const uint8_t *buf, size_t len, t1_atr_decoded_t* p_atr) {
  // Size of one interface record, set of TAi, TBi and TCi bytes
  const size_t intf_size = t1_atr_ta2 - t1_atr_ta1;
  // Dummy buffer to receive unneeded interface bytes, safer than NULL
  int16_t intf_null[intf_size];
  // Pointer to buffer receiving interface bytes
  int16_t* p_intf = intf_null;
  // Index of current interface block, named as "i" in standard
  size_t intf_idx = 1;
  // Index within t1_ev_prm_atr_t::global_bytes[] array
  size_t global_idx = 0;
  // Index within t1_ev_prm_atr_t::t1_bytes[] array
  size_t t1_idx = 0;
  // Expected length in bytes
  size_t exp_len = ATR_MIN_BYTES;
  // Indicator of presence of TAi...TDi bytes, named as "Y" in standard
  uint8_t indicator = 0;
  // XOR checksum
  uint8_t checksum = 0;
  // Flag indicating presence of TCK byte
  bool tck_present = false;
  // Pointer to current byte
  const uint8_t *p_byte = buf;

  // Fill "atr" and "atr_len", set other fields to defaults
  atr_decoded_init(p_atr, buf, len);

  // Parse all expected bytes of ATR but not more than contained in buffer
  for(size_t n = 0; n < len && exp_len <= len && n < exp_len; n++, p_byte++) {
    if(n == 0) { // TS, initial character
      if(*p_byte == TS_CONVENTION_INVERSE) {
        p_atr->convention = t1_cnv_inverse;
      } else if(*p_byte != TS_CONVENTION_DIRECT) {
        return false;
      }
    } else if(n == 1) { // T0, format byte
      exp_len += p_atr->hist_nbytes = *p_byte & 0x0F;
      indicator = *p_byte >> 4;
      if(indicator) {
          p_intf = &p_atr->global_bytes[global_idx];
          global_idx += intf_size;
          exp_len += atr_ibyte_num(indicator);
      }
      if(!(indicator & TD_BIT)) { // TD1 is absent, only T=0 is supported
        p_atr->t0_supported = true;
      }
    } else {
      if(indicator & TA_BIT) { // TA byte
        p_intf[t1_atr_ta1] = *p_byte;
        indicator ^= TA_BIT;
      } else if(indicator & TB_BIT) { // TB byte
        p_intf[t1_atr_tb1] = *p_byte;
        indicator ^= TB_BIT;
      } else if(indicator & TC_BIT) { // TC byte
        p_intf[t1_atr_tc1] = *p_byte;
        indicator ^= TC_BIT;
      } else if(indicator & TD_BIT) { // TD byte
        p_intf = intf_null;
        indicator = *p_byte >> 4;
        if(indicator) {
          protocol_id_t prot_id = (protocol_id_t)(*p_byte & 0x0F);
          exp_len += atr_ibyte_num(indicator);
          ++intf_idx;
          if(prot_id == atr_prot_t0) {
            p_atr->t0_supported = true;
          } else if(prot_id == atr_prot_t1) {
            p_atr->t1_supported = true;
          }
          if(prot_id == atr_prot_t1 && intf_idx > 2) {
            if(t1_idx + intf_size <= t1_atr_intf_bytes) {
              p_intf = &p_atr->t1_bytes[t1_idx];
              t1_idx += intf_size;
            }
          } else if(prot_id == atr_globals || intf_idx <= 2) {
            if(global_idx + intf_size <= t1_atr_intf_bytes) {
              p_intf = &p_atr->global_bytes[global_idx];
              global_idx += intf_size;
            }
          }
          if(prot_id != atr_prot_t0 && !tck_present) { // Is TCK present?
            tck_present = true;
            ++exp_len;
          }
        }
      } else if(p_atr->hist_bytes == NULL) { // Historical bytes
        p_atr->hist_bytes = p_byte;
      }
    }

    if(n > 0) { // Calculate checksum
      checksum ^= *p_byte;
    }
  }

  // Ensure that ATR is not truncated and TCK is correct (if exists)
  if(exp_len <= len && (!tck_present || checksum == 0) ) {
    return true;
  }
  return false;
}
static inline void connection_set_parameters_extended_apdu(usb_connection_obj_t* self) {
        uint8_t rcv[64] = {0};
        connection_ccid_transmit_set_parameters(self, &hUsbHostFS);
        connection_ccid_receive_extended_apdu_first_block(self);
        connection_ccid_receive_extended_apdu_next_block(self, 0x10, 0);
}

static inline void wait_connect_blocking_extended_apdu(usb_connection_obj_t* self) {
  uint8_t rx_buf[512] = {0};
  uint16_t rcvBytes = 0;
  uint16_t needToRcv = 0;
  unsigned int dwLength = 0; 
  self->atr_timeout_ms = 5000;
  while (self->state_ext_apdu == state_connecting) {
    mp_uint_t ticks_ms = mp_hal_ticks_ms();
    mp_uint_t elapsed = scard_ticks_diff(ticks_ms, self->prev_ticks_ms);
    self->prev_ticks_ms = ticks_ms;
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                        sizeof(hUsbHostFS.rawRxData));
    if (connection_timer_elapsed(&self->atr_timeout_ms, elapsed)){
      raise_SmartcardException("atr timeout is elapsed");
    }
    if (hUsbHostFS.rawRxData[0] == RDR_to_PC_DataBlock) {
      /// Copy all available data into buffer
      memcpy(rx_buf, hUsbHostFS.rawRxData + CCID_ICC_HEADER_LENGTH, self->CCID_Handle->DataItf.InEpSize);
      /// Calculate if we recieved all data
      rcvBytes = USBH_LL_GetLastXferPktSize(&hUsbHostFS, self->CCID_Handle->DataItf.InPipe) - CCID_ICC_HEADER_LENGTH;
      dwLength = dw2i(hUsbHostFS.rawRxData, 1);
      needToRcv = dwLength - rcvBytes;
      memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
    } 
    if(needToRcv > 0 && hUsbHostFS.rawRxData[0] != 0)
    {
      uint16_t lastTransferlength = USBH_LL_GetLastXferPktSize(&hUsbHostFS, self->CCID_Handle->DataItf.InPipe); 
      memcpy(rx_buf + rcvBytes, hUsbHostFS.rawRxData, needToRcv); 
      t1_atr_decoded_t atr_decoded;
      if(connection_parse_atr_extended_apdu(rx_buf, dwLength, &atr_decoded)){
        self->atr = mp_obj_new_bytes(atr_decoded.atr, atr_decoded.atr_len);
        self->TA_1 = atr_decoded.atr[2];
        self->state_ext_apdu = state_connected;
      } 
    }
  }
}

/**
 * @brief Terminates communication with a smart card and removes power
 *
 * .. method:: UsbCardConnection.disconnect()
 *
 *  Terminates communication with a smart card and removes power. Call this
 *  function before asking the user to remove a smart card. This function does
 *  not close connection but makes it inactive. When a new card is inserted, the
 *  connection object may be reused with the same parameters.
 *
 * @param self_in  instance of CardConnection class
 * @return         None
 */
STATIC mp_obj_t connection_disconnect(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  USBH_ChipCardDescTypeDef chipCardDesc =
      hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
  if (self->process_state != process_state_ready) {
    raise_SmartcardException("smart card reader is not connected");
  }
  // Apply power off cmd to the card
  if (!card_present(self, &hUsbHostFS)) {
    raise_NoCardException("no card inserted");
  }
  self->IccCmd[0] = 0x63; /* IccPowerOff */
  self->IccCmd[1] = self->IccCmd[2] = self->IccCmd[3] = self->IccCmd[4] =
      0;                                            /* dwLength */
  self->IccCmd[5] = chipCardDesc.bCurrentSlotIndex; /* slot number */
  self->IccCmd[6] = self->pbSeq++;
  self->IccCmd[7] = self->IccCmd[8] = self->IccCmd[9] = 0; /* RFU */
  hUsbHostFS.apdu = self->IccCmd;
  connection_ccid_transmit_raw(self, &hUsbHostFS, hUsbHostFS.apdu,
                               sizeof(self->IccCmd));
  connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                          sizeof(hUsbHostFS.rawRxData));
  memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
  // Stop USB CCID communication
  USBH_CCID_Stop(&hUsbHostFS);
  // Deinitialize timer
  if (self->timer) {
    mp_obj_t deinit_fn = mp_load_attr(self->timer, MP_QSTR_deinit);
    (void)mp_call_function_0(deinit_fn);
    self->timer = MP_OBJ_NULL;
  }
  // Detach from reader
  usbreader_deleteConnection(self->reader, MP_OBJ_FROM_PTR(self));
  self->reader = MP_OBJ_NULL;
  // Mark connection object as closed
  self->state = state_closed;
  return mp_const_none;
}
/**
 * Callback function that handles protocol events
 *
 * This protocol implementation guarantees that event handler is always called
 * just before termination of any external API function. It allows to call
 * safely any other API function(s) within user handler code.
 *
 * @param self_in  instance of user class
 * @param ev_code  event code
 * @param ev_prm   event parameter depending on event code, typically NULL
 */
static void proto_cb_handle_event(mp_obj_t self_in, proto_ev_code_t ev_code,
                                  proto_ev_prm_t prm) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  uint8_t rcv[64];
  switch (ev_code) {
    case proto_ev_atr_received:
      self->atr =
          mp_obj_new_bytes(prm.atr_received->atr, prm.atr_received->len);
      self->TA_1 = prm.atr_received->atr[2];
      break;

    case proto_ev_connect:
      if (state_connecting == self->state) {
        self->state = state_connected;
        notify_observers(self, event_connect);
      }
      break;

    case proto_ev_apdu_received:
      if (state_connected == self->state) {
        mp_obj_t response =
            make_response_list(prm.apdu_received->apdu, prm.apdu_received->len);
        notify_observers_response(self, response);
        if (self->blocking) {
          self->response = response;
        }
      }
      break;
    case proto_ev_pps_exchange_done:
      if (self->state == state_connecting) {
        connection_ccid_transmit_set_parameters(self, &hUsbHostFS);
        memset(rcv, 0, sizeof(rcv));
        connection_ccid_receive(&hUsbHostFS, rcv, sizeof(rcv));
        mp_hal_delay_ms(150);
      }
      break;
    case proto_ev_error:
      connection_disconnect(self);
      self->state = state_error;
      notify_observers_text(self, event_error, prm.error);
      if (self->raise_on_error || self->blocking) {
        self->raise_on_error = false;
        raise_SmartcardException(prm.error);
      }
      break;
  }
}

/**
 * Handles change of smart card presence state
 *
 * @param self       instance of CardConnection class
 * @param new_state  new state: true - card present, false - card absent
 */
static void handle_card_presence_change(usb_connection_obj_t* self,
                                        bool new_state) {
  const char* err_unexp_removal = "unexpected card removal";

  if (new_state != self->presence_state) {
    self->presence_state = new_state;
    if (new_state) {  // Card present
      notify_observers(self, event_insertion);
    } else {  // Card absent
      notify_observers(self, event_removal);
      // Check if the card is in use
      if (state_connecting == self->state || state_connected == self->state) {
        // Handle unexpected card removal
        connection_disconnect(self);
        self->state = state_error;
        notify_observers_text(self, event_error, err_unexp_removal);
        if (self->blocking) {
          raise_SmartcardException(err_unexp_removal);
        }
      }
    }
  }
}

/**
 * Checks if smart card is present
 *
 * @param self  an instance of CardConnection class
 * @param phost  instance of USB host handle structure
 * @return      true if card present
 */
static bool card_present(usb_connection_obj_t* self,
                         USBH_HandleTypeDef* phost) {
  // Read pin with blocking debounce algorithm if no timer is created or
  // non-blocking debounce algorithm has not come to a stable state yet.
    handle_card_presence_change(
        self, (phost->iccSlotStatus == ICC_INSERTED) ? true : false);
  return self->presence_state;
}

/**
 * The periodic task that detects presence of smart card
 *
 * This task is called each 1ms ... 50ms.
 *
 * @param self  instance of CardConnection class
 * @param phost  instance of USB host handle structure
 */
static void card_detection_task(usb_connection_obj_t* self,
                                USBH_HandleTypeDef* phost) {
  bool state = phost->iccSlotStatus == ICC_INSERTED;
  bool state_valid = false;

  // Non-blocking debounce algorithm. When the pin is active, counter is
  // incremented until it reaches threshold value; at that point pin state is
  // considered valid. When the pin is inactive the counter is set to zero and
  // pin state is valid (absent state).
  if (state) {  // Looks like the card is present
    if (self->presence_cycles >= CARD_PRESENCE_CYCLES) {
      state_valid = true;
    } else {
      ++self->presence_cycles;
    }
  } else {  // Card is absent
    self->presence_cycles = 0;
    state_valid = true;
  }

  if (state_valid) {
    handle_card_presence_change(self, state);
  }
}

/**
 * Waits for response from smart card in blocking mode
 *
 * @param self  instance of CardConnection class
 */
static inline void wait_response_blocking(usb_connection_obj_t* self) {
  while (self->response == MP_OBJ_NULL) {
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                            sizeof(hUsbHostFS.rawRxData));
    if (hUsbHostFS.rawRxData[0] == RDR_to_PC_DataBlock) {
      uint16_t dwLength = hUsbHostFS.rawRxData[1];
      if (dwLength != 0) {
        uint8_t rx_buf[CCID_MAX_RESP_LENGTH];
        memcpy(rx_buf, hUsbHostFS.rawRxData + CCID_ICC_HEADER_LENGTH, dwLength);
        self->protocol->serial_in(self->proto_handle, rx_buf, dwLength);
        memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
      } else {
        raise_SmartcardException("lenght of bulk-in message is incorrect");
      }
    }
    timer_task(self);
    MICROPY_EVENT_POLL_HOOK
  }
}

/**
 * Waits for smart card response from reader which supports extended apdu exchange 
 *
 * @param self  instance of CardConnection class
 */
static inline void wait_response_blocking_extended_apdu(usb_connection_obj_t* self) {
  uint8_t rx_buf[512] = {0};
  uint16_t rcvBytes = 0;
  int needToRcv = -1;
  unsigned int dwLength = 0;
  unsigned int bytesOffset = 0; 
  bool gotFirtsFrame = false;
  t1_apdu_t apdu_received;
  self->rsp_timeout_ms = 15000;
  while (self->response == MP_OBJ_NULL) {
    mp_uint_t ticks_ms = mp_hal_ticks_ms();
    mp_uint_t elapsed = scard_ticks_diff(ticks_ms, self->prev_ticks_ms);
    self->prev_ticks_ms = ticks_ms;
    memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                        sizeof(hUsbHostFS.rawRxData));
    if (hUsbHostFS.rawRxData[0] == RDR_to_PC_DataBlock) {
      /// Copy all available data into buffer
      memcpy(rx_buf, hUsbHostFS.rawRxData + CCID_ICC_HEADER_LENGTH, self->CCID_Handle->DataItf.InEpSize);
      /// Calculate if we recieved all data
      rcvBytes = USBH_LL_GetLastXferPktSize(&hUsbHostFS, self->CCID_Handle->DataItf.InPipe) - CCID_ICC_HEADER_LENGTH;
      dwLength = dw2i(hUsbHostFS.rawRxData, 1);
      needToRcv = dwLength - rcvBytes;
      /// If we received all data save response and exit from while loop
      if(needToRcv == 0)
      {
        mp_obj_t response = make_response_list(rx_buf, dwLength);
        notify_observers_response(self, response);
        if (self->blocking) {
          self->response = response;
        }
      }
      gotFirtsFrame = true;
    }
    /// If we have more data to receive save it to the buffer
    if(hUsbHostFS.rawRxData[0] != 0 && needToRcv > 0 && gotFirtsFrame)
    { 
      bytesOffset = bytesOffset + self->CCID_Handle->DataItf.InEpSize;
      needToRcv = needToRcv - self->CCID_Handle->DataItf.InPipe;
      if(needToRcv >= self->CCID_Handle->DataItf.InPipe)
      {
        memcpy(rx_buf + bytesOffset, hUsbHostFS.rawRxData, self->CCID_Handle->DataItf.InPipe);
      }
      else
      {
        memcpy(rx_buf + bytesOffset, hUsbHostFS.rawRxData, self->CCID_Handle->DataItf.InPipe);
      }
      if(needToRcv == 0 || needToRcv < 0)
      {
        mp_obj_t response = make_response_list(rx_buf, dwLength);
        notify_observers_response(self, response);
        if (self->blocking) {
          self->response = response;
        }
      }
    }
    if (connection_timer_elapsed(&self->rsp_timeout_ms, elapsed)){
      raise_SmartcardException("response timeout is elapsed");
    }   
  }
}


/**
 * Waits for smart card response from reader which supports extended apdu exchange 
 *
 * @param self  instance of CardConnection class
 */
static inline void wait_response_blocking_extended_apdu_2(usb_connection_obj_t* self) {
    uint8_t rx_buf[512] = {0};
    unsigned int bytesOffset = 0;
    connection_ccid_receive_extended_apdu_first_block(self);
    unsigned int dwLength = dw2i(hUsbHostFS.rawRxData, 1);
    if(dwLength < self->CCID_Handle->DataItf.InEpSize){
      memcpy(rx_buf, hUsbHostFS.rawRxData, dwLength + CCID_ICC_HEADER_LENGTH);
    }
    else {
      memcpy(rx_buf, hUsbHostFS.rawRxData, self->CCID_Handle->DataItf.InEpSize);
    }
    if(rx_buf[0] == RDR_to_PC_DataBlock){
      unsigned int pktCount = dwLength/self->CCID_Handle->DataItf.InEpSize;
      for(int i = 0; i < pktCount; i++)
      {
        memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
        connection_ccid_receive_extended_apdu_next_block(self, 0x10, 0);
        uint16_t length = USBH_LL_GetLastXferPktSize(&hUsbHostFS, self->CCID_Handle->DataItf.InPipe);   
        bytesOffset = bytesOffset + self->CCID_Handle->DataItf.InEpSize;
        memcpy(rx_buf + bytesOffset, hUsbHostFS.rawRxData, length);
      }
    }
    else
    {
     raise_SmartcardException("ccid message is corrupted"); 
    }
    t1_apdu_t apdu_received;
    mp_obj_t response = make_response_list(rx_buf + CCID_ICC_HEADER_LENGTH, dwLength);
    notify_observers_response(self, response);
    if (self->blocking) {
      self->response = response;
    }
}

/**
 * Callback function that outputs bytes to serial port
 *
 * @param self_in  instance of user class
 * @param buf      buffer containing data to transmit
 * @param len      length of data block in bytes
 */
static bool proto_cb_serial_out(mp_obj_t self_in, const uint8_t* buf,
                                size_t len) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  connection_ccid_transmit_xfr_block(self, &hUsbHostFS, (uint8_t*)buf, len);
  return true;
}
/**
 * Changes smart card protocol
 *
 * @param self           instance of CardConnection class
 * @param protocol_id    identifier of the new protocol
 * @param reset_if_same  if true resets protocol instance if it's not changed
 * @param wait_atr       if true sets protocol instance into "waiting for ATR"
 *                       state when the protocol is replaced or reset
 */
static void change_protocol(usb_connection_obj_t* self, mp_int_t protocol_id,
                            bool reset_if_same, bool wait_atr) {
  // Get implementation of the new smart card protocol
  const proto_impl_t* protocol = proto_get_implementation(protocol_id);
  if (!protocol) {
    raise_SmartcardException("protocol not supported");
  }

  // Check if protocol is already in use
  if (self->proto_handle && protocol == self->protocol) {
    // Already in use, we need to reset it if requested
    if (reset_if_same) {
      self->protocol->reset(self->proto_handle, wait_atr);
    }
  } else {
    // Release previous protocol handle if available
    if (self->protocol && self->proto_handle) {
      self->protocol->deinit(self->proto_handle);
    }

    // Allocate the new protocol instance and get handle
    self->protocol = protocol;
    self->proto_handle =
        protocol->init(proto_cb_serial_out, proto_cb_handle_event, self);
    if (!self->proto_handle) {
      self->protocol = NULL;
      raise_SmartcardException("error initializing protocol");
    }
    if (!wait_atr) {  // The caller asks to skip "waiting for ATR" state
      self->protocol->reset(self->proto_handle, false);
    }

    // Set timeouts, raise exception if failed even for non-blocking connection
    self->raise_on_error = true;
    protocol->set_timeouts(self->proto_handle, self->atr_timeout_ms,
                           self->rsp_timeout_ms, self->max_timeout_ms);
    self->raise_on_error = false;
  }
}

STATIC mp_obj_t connection_transmit(size_t n_args, const mp_obj_t* pos_args,
                                    mp_map_t* kw_args) {
  // Get self
  assert(n_args >= 1U);
  usb_connection_obj_t* self =
      (usb_connection_obj_t*)MP_OBJ_TO_PTR(pos_args[0]);

  // Parse and check arguments
  enum { ARG_bytes = 0, ARG_protocol };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_bytes, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},
      {MP_QSTR_protocol, MP_ARG_OBJ, {.u_obj = mp_const_none}}};
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1U, pos_args + 1U, kw_args,
                   MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  // Check connection state for card readers with 
  // extended APDU Level of exchange
  if ((self->dwFeatures & CCID_CLASS_SHORT_APDU) ||
    (self->dwFeatures & CCID_CLASS_EXTENDED_APDU))
  {
    if (state_connected != self->state_ext_apdu) {
      raise_SmartcardException("card not connected");
    }
  }
  else
  {
    // Check connection state
    if (state_connected != self->state) {
      raise_SmartcardException("card not connected");
    }
  }

  // Change smart card protocol if requested
  mp_int_t new_protocol = self->next_protocol;
  self->next_protocol = protocol_na;
  if (!mp_obj_is_type(args[ARG_protocol].u_obj, &mp_type_NoneType)) {
    new_protocol = mp_obj_get_int(args[ARG_protocol].u_obj);
  }
  if (new_protocol != protocol_na) {
    change_protocol(self, new_protocol, false, false);
  }
  if (!self->protocol) {
    raise_SmartcardException("no protocol selected");
  }

  // Notify observers of transmitted command
  notify_observers_command(self, args[ARG_bytes].u_obj);

  // Get data buffer of 'bytes' argument
  mp_buffer_info_t bufinfo = {.buf = NULL, .len = 0U};
  uint8_t static_buf[32] = {0};
  uint8_t* dynamic_buf = NULL;
  // Check if bytes is a list
  if (mp_obj_is_type(args[ARG_bytes].u_obj,
                     &mp_type_list)) {  // 'bytes' is list
    // Get properties of the list
    mp_obj_t* items;
    mp_obj_list_get(args[ARG_bytes].u_obj, &bufinfo.len, &items);
    // Chose static buffer or allocate dynamic buffer if needed
    if (bufinfo.len <= sizeof(static_buf)) {
      bufinfo.buf = static_buf;
    } else {
      dynamic_buf = m_new(uint8_t, bufinfo.len);
      bufinfo.buf = dynamic_buf;
    }
    // Convert list to a buffer of bytes
    if (!objects_to_buf(bufinfo.buf, items, bufinfo.len)) {
      mp_raise_ValueError("incorrect data format");
    }
  } else {  // 'bytes' is bytes array or anything supporting buffer protocol
    mp_get_buffer_raise(args[ARG_bytes].u_obj, &bufinfo, MP_BUFFER_READ);
  }

  // Transmit APDU
  self->response = MP_OBJ_NULL;
  memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
  if ((self->dwFeatures & CCID_CLASS_SHORT_APDU) ||
    (self->dwFeatures & CCID_CLASS_EXTENDED_APDU))
  {
   // connection_ccid_transmit_xfr_block_extended_apdu(self, &hUsbHostFS, bufinfo.buf,
   //                                 bufinfo.len);
   connection_ccid_process_exchange_extended_apdu(self, &hUsbHostFS, bufinfo.buf, bufinfo.len);
  }
  else
  {
    self->protocol->transmit_apdu(self->proto_handle, bufinfo.buf, bufinfo.len);
  }
  // Let's free dynamic buffer manually to help the GC
  if (dynamic_buf) {
    m_del(uint8_t, dynamic_buf, bufinfo.len);
    dynamic_buf = NULL;
  }
  if (self->blocking) {
    if ((self->dwFeatures & CCID_CLASS_SHORT_APDU) || (self->dwFeatures & CCID_CLASS_EXTENDED_APDU))
    {
      mp_hal_delay_ms(150);
    }
    else
    {
      wait_response_blocking(self);
    }
    // Do not keep the reference to allow GC to remove response later
    mp_obj_t response = self->response;
    self->response = MP_OBJ_NULL;
    return response;
  }

  return mp_const_none;
}
#ifdef USB_DEBUG
STATIC mp_obj_t connection_poweron(size_t n_args, const mp_obj_t* pos_args,
                                   mp_map_t* kw_args) {
  // Get self
  assert(n_args >= 1U);
  usb_connection_obj_t* self =
      (usb_connection_obj_t*)MP_OBJ_TO_PTR(pos_args[0]);

  // Parse and check arguments
  enum { ARG_protocol = 0 };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_protocol, MP_ARG_INT, {.u_int = protocol_na}},
  };
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1U, pos_args + 1U, kw_args,
                   MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  // Change smart card protocol if requested
  mp_int_t new_protocol = (protocol_na != args[ARG_protocol].u_int)
                              ? args[ARG_protocol].u_int
                              : self->next_protocol;
  self->next_protocol = protocol_na;
  // If protocol not defined, use default protocol
  new_protocol = (protocol_na == new_protocol) ? protocol_any : new_protocol;
  change_protocol(self, new_protocol, true, true);

  self->chipCardDesc = hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
  self->dwFeatures = self->chipCardDesc.dwFeatures;
  self->CCID_Handle = hUsbHostFS.pActiveClass->pData;
  // Set IFSD=0x32 because we can receive more than 64 bytes per attempt
  self->protocol->set_usb_features(self->proto_handle,
                                   self->chipCardDesc.dwFeatures, 0x08);

  self->CCID_Handle = hUsbHostFS.pActiveClass->pData;
  if (self->process_state == process_state_ready) {
    notify_observers(self, event_insertion);
    USBH_ChipCardDescTypeDef chipCardDesc =
        hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
    hUsbHostFS.apduLen = CCID_ICC_HEADER_LENGTH;
    self->IccCmd[0] = 0x62;
    self->IccCmd[1] = 0x00;
    self->IccCmd[2] = 0x00;
    self->IccCmd[3] = 0x00;
    self->IccCmd[4] = 0x00;
    self->IccCmd[5] = chipCardDesc.bCurrentSlotIndex;
    self->IccCmd[6] = self->pbSeq++;
    self->IccCmd[7] = 0x01;
    self->IccCmd[8] = 0x00;
    self->IccCmd[9] = 0x00;
    hUsbHostFS.apdu = self->IccCmd;
    connection_ccid_transmit_raw(self, &hUsbHostFS, hUsbHostFS.apdu,
                              hUsbHostFS.apduLen);
    self->state_ext_apdu = state_connecting;
    wait_connect_blocking_extended_apdu(self);
    memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
    /// Select
    // uint8_t apdu_select[] = {0x00, 0xA4, 0x04, 0x00, 0x06, 0xB0, 0x0B, 0x51, 0x11, 0xCA, 0x01};
    uint8_t apdu_select_p1[] = {0x00, 0xA4, 0x04, 0x00, 0x06, 0xB0};
    uint8_t apdu_select_p2[] = {0x0B, 0x51, 0x11, 0xCA, 0x01};
    uint8_t cmd[CCID_ICC_HEADER_LENGTH + CCID_MAX_DATA_LENGTH] = {0};
    // Part 1
    connection_prepare_xfrblock(self, apdu_select_p1, sizeof(apdu_select_p1), cmd, 0x01, 0);
    connection_ccid_transmit_raw_extended_apdu(self, &hUsbHostFS, cmd, sizeof(apdu_select_p1) + CCID_ICC_HEADER_LENGTH);
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                            sizeof(hUsbHostFS.rawRxData));
    mp_hal_delay_ms(150);
    for(int i = 0; i < 32; i++)
    {
      printf("0x%X ", hUsbHostFS.rawRxData[i]);
    }
    printf("\n");
    // Part 2
    connection_prepare_xfrblock(self, apdu_select_p2, sizeof(apdu_select_p2), cmd, 0x02, 0);
    connection_ccid_transmit_raw_extended_apdu(self, &hUsbHostFS, cmd, sizeof(apdu_select_p2) + CCID_ICC_HEADER_LENGTH); 
    // Recv
    connection_ccid_receive(&hUsbHostFS, hUsbHostFS.rawRxData,
                            sizeof(hUsbHostFS.rawRxData));
    mp_hal_delay_ms(150);
    for(int i = 0; i < 32; i++)
    {
      printf("0x%X ", hUsbHostFS.rawRxData[i]);
    }
    printf("\n");
    /// Get data
    uint8_t apdu_get[] = {0xB0, 0xA1, 0x00, 0x00, 0x00};
    connection_ccid_transmit_xfr_block(self, &hUsbHostFS, (uint8_t*)apdu_get,
                                       sizeof(apdu_get));
    wait_response_blocking_extended_apdu_2(self);
    mp_hal_delay_ms(150);
    for(int i = 0; i < 32; i++)
    {
      printf("0x%X ", hUsbHostFS.rawRxData[i]);
    }
    printf("\n");
  } else {
    raise_SmartcardException("smart card reader is not connected");
  }
  return mp_const_none;
}
#endif
/**
 * @brief Connects to a smart card
 *
 * .. method:: UsbCardConnection.connect()
 *
 *  Connects to a smart card.
 *
 * @param self      instance of UsbCardConnection class
 * @return          None
 */
STATIC mp_obj_t connection_connect(size_t n_args, const mp_obj_t* pos_args,
                                   mp_map_t* kw_args) {
  // Get self
  assert(n_args >= 1U);
  usb_connection_obj_t* self =
      (usb_connection_obj_t*)MP_OBJ_TO_PTR(pos_args[0]);

  // Parse and check arguments
  enum { ARG_protocol = 0 };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_protocol, MP_ARG_INT, {.u_int = protocol_na}},
  };
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1U, pos_args + 1U, kw_args,
                   MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  // Change smart card protocol if requested
  mp_int_t new_protocol = (protocol_na != args[ARG_protocol].u_int)
                              ? args[ARG_protocol].u_int
                              : self->next_protocol;
  self->next_protocol = protocol_na;
  // If protocol not defined, use default protocol
  new_protocol = (protocol_na == new_protocol) ? protocol_any : new_protocol;
  change_protocol(self, new_protocol, true, true);
  self->chipCardDesc = hUsbHostFS.device.CfgDesc.Itf_Desc[0].CCD_Desc;
  self->dwFeatures = self->chipCardDesc.dwFeatures;
  self->CCID_Handle = hUsbHostFS.pActiveClass->pData;
  // Set IFSD=0x20 because we can receive more than 64 bytes per attempt
  self->protocol->set_usb_features(self->proto_handle,
                                   self->chipCardDesc.dwFeatures, 0x20);
  if (self->process_state != process_state_ready) {
    raise_SmartcardException("smart card reader is not connected");
  }
  // Apply power and reset the card
  if (!card_present(self, &hUsbHostFS)) {
      raise_NoCardException("no card inserted");
  }
  hUsbHostFS.apduLen = CCID_ICC_HEADER_LENGTH;
  self->IccCmd[0] = 0x62;
  self->IccCmd[1] = 0x00;
  self->IccCmd[2] = 0x00;
  self->IccCmd[3] = 0x00;
  self->IccCmd[4] = 0x00;
  self->IccCmd[5] = self->chipCardDesc.bCurrentSlotIndex;
  self->IccCmd[6] = self->pbSeq++;
  self->IccCmd[7] = getVoltageSupport(&self->chipCardDesc);
  self->IccCmd[8] = 0x00;
  self->IccCmd[9] = 0x00;
  hUsbHostFS.apdu = self->IccCmd;
  notify_observers_command(self, self->IccCmd);
  connection_ccid_transmit_raw(self, &hUsbHostFS, hUsbHostFS.apdu,
                               hUsbHostFS.apduLen);
  // Update state and wait for connection established
  if ((self->dwFeatures & CCID_CLASS_SHORT_APDU) ||
    (self->dwFeatures & CCID_CLASS_EXTENDED_APDU))
  {
    self->state_ext_apdu = state_connecting;
    wait_connect_blocking_extended_apdu(self);
    connection_set_parameters_extended_apdu(self);
  }
  else
  {
    self->state = state_connecting;
    wait_connect_blocking(self);
  }
  memset(hUsbHostFS.rawRxData, 0, sizeof(hUsbHostFS.rawRxData));
  return mp_const_none;
}

/**
 * @brief Checks if smart card is inserted
 *
 * .. method:: UsbCardConnection.isCardInserted()
 *
 * @param self_in   instance of CardConnection class
 * @return          True if smart card is inserted
 */
/**
 * @brief Checks if smart card is inserted
 *
 * .. method:: CardConnection.isCardInserted()
 *
 * @param self_in   instance of CardConnection class
 * @return          True if smart card is inserted
 */
STATIC mp_obj_t connection_isCardInserted(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  return card_present(self, &hUsbHostFS) ? mp_const_true : mp_const_false;
}

/**
 * @brief Return card connection reader
 *
 * .. method:: UsbCardConnection.getReader()
 *
 * @param self_in  instance of UsbCardConnection class
 * @return         reader object
 */
STATIC mp_obj_t connection_getReader(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  if (self->reader) {
    return MP_OBJ_FROM_PTR(self->reader);
  }
  return mp_const_none;
}

/**
 * @brief Checks if connection is active
 *
 * .. method:: UsbCardConnection.isActive()
 *
 *  Checks connection state, returning True if card is inserted and powered
 *
 * @param self_in  instance of UsbCardConnection class
 * @return         connection state: True - active, False - not active
 */
STATIC mp_obj_t connection_isActive(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  if ((self->dwFeatures & CCID_CLASS_SHORT_APDU) ||
    (self->dwFeatures & CCID_CLASS_EXTENDED_APDU))
  {
    switch (self->state_ext_apdu) {
      case state_connecting:
      case state_connected:
        return mp_const_true;

      default:
        return mp_const_false;
    }
  }
  else
  {
    switch (self->state) {
      case state_connecting:
      case state_connected:
        return mp_const_true;

      default:
        return mp_const_false;
    }
  }
}

/**
 * @brief Checks if smart card reader is ready for connection
 *
 * .. method:: UsbCardConnection.isReady()
 *
 *  Checks connection state, returning True if card is inserted and powered
 *
 * @param self_in  instance of UsbCardConnection class
 * @return         connection state: True - active, False - not active
 */
STATIC mp_obj_t connection_isReady(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  switch (self->process_state) {
    case process_state_ready:
      return mp_const_true;
    default:
      return mp_const_false;
  }
}

/**
 * @brief Returns APDU sentence received from the smart card
 *
 * .. method:: UsbCardConnection.getAPDU()
 *
 * @param self_in  instance of CardConnection class
 * @return         APDU sentence as array bytes or None if not available
 */
STATIC mp_obj_t connection_getAPDU(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  if (state_closed != self->state && self->apdu) {
    return self->apdu;
  }
  return mp_const_none;
}

/**
 * @brief Returns ATR sentence received from the smart card
 *
 * .. method:: UsbCardConnection.getATR()
 *
 * @param self_in  instance of CardConnection class
 * @return         ATR sentence as array bytes or None if not available
 */
STATIC mp_obj_t connection_getATR(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  if (state_closed != self->state && self->atr) {
    return self->atr;
  }
  else if (state_closed != self->state_ext_apdu && self->atr){
    return self->atr;
  }
  else{
    return mp_const_none;
  }
}
/**
 * @brief Returns connection state
 *
 * .. method:: UsbCardConnection.getState()
 *
 *  Returns connection state, one of the following QSTR strings:
 *
 *    - *'closed'* - connection is closed
 *    - *'disconnected'* - smart card is disconnected
 *    - *'connecting'* - smart card connection is in progress
 *    - *'connected'* - smart card is connected and ready to use
 *    - *'error'* - smart card error
 *
 * @param self_in  instance of CardConnection class
 * @return         connection state, one of uscard.STATE_* constants
 */
STATIC mp_obj_t connection_getState(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  return MP_OBJ_NEW_QSTR((qstr)self->state);
}

/**
 * @brief Adds observer
 *
 * .. method:: UsbCardConnection.addObserver(observer)
 *
 *  Adds observer receiving notifications on events of connection. The
 *  *observer* should be a callable object, a function or a method having the
 *  following signature:
 *
 *  def my_function(usbcardconnection, type, args=None)
 *  def my_method(self, usbcardconnection, type, args=None)
 *
 *  Observer arguments:
 *
 *    - *usbscardconnection* smart card connection
 *    - *type* event type: 'connect', 'disconnect', 'command', 'response',
 *                         'insertion', 'removal', 'error'
 *    - *args* event arguments, a list
 *
 * @param self_in   instance of CardConnection class
 * @param observer  observer, a callable object
 * @return          None
 */
STATIC mp_obj_t connection_addObserver(mp_obj_t self_in, mp_obj_t observer) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;

  if (!mp_obj_is_callable(observer)) {
    mp_raise_TypeError("observer must be callable");
  }
  (void)mp_obj_list_append(self->observers, observer);

  return mp_const_none;
}

/**
 * @brief Deletes observer
 *
 * .. method:: UsbCardConnection.deleteObserver(observer)
 *
 *  Deletes observer. The *observer* should be a callable object.
 *
 * @param self_in   instance of UsbCardConnection class
 * @param observer  observer, a callable object
 * @return          None
 */
STATIC mp_obj_t connection_deleteObserver(mp_obj_t self_in, mp_obj_t observer) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;

  if (!mp_obj_is_callable(observer)) {
    mp_raise_TypeError("observer must be callable");
  }
  (void)mp_obj_list_remove(self->observers, observer);

  return mp_const_none;
}

/**
 * @brief Deletes all observers
 *
 * .. method:: UsbCardConnection.deleteObservers()
 *
 *  Deletes all observers.
 *
 * @param self_in  instance of UsbCardConnection class
 * @return         None
 */
STATIC mp_obj_t connection_deleteObservers(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;

  // Remove references to each observer to help the GC
  size_t n_observers;
  mp_obj_t* observers;
  mp_obj_list_get(self->observers, &n_observers, &observers);
  for (size_t i = 0U; i < n_observers; ++i) {
    observers[i] = MP_OBJ_NULL;
  }

  // Replace with a new empty list
  self->observers = mp_obj_new_list(0, NULL);

  return mp_const_none;
}

/**
 * @brief Returns number of registered observers
 *
 * .. method:: UsbCardConnection.countObservers()
 *
 *  Returns number of registered observers.
 *
 * @param self_in  instance of UsbCardConnection class
 * @return         number of registered observers
 */
STATIC mp_obj_t connection_countObservers(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  return MP_OBJ_NEW_SMALL_INT(list_get_len(self->observers));
}

/**
 * Scheduled task notifying all observers about all occurred events
 *
 * Not intended to be called by user.
 *
 * @param self    instance of CardConnection class
 * @param unused  unused
 * @return        None
 */
STATIC mp_obj_t connection_notifyAll(mp_obj_t self_in, mp_obj_t unused) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  size_t n_events = self->event_idx;
  self->event_idx = 0U;  // Clear event buffer

  // Extract the list of observers
  size_t n_observers;
  mp_obj_t* observers;
  mp_obj_list_get(self->observers, &n_observers, &observers);

  // Loop over all events in the buffer
  const event_t* p_event = self->event_buf;
  while (p_event != self->event_buf + n_events) {
    // Notify all observers
    for (size_t i = 0U; i < n_observers; ++i) {
      (void)mp_call_function_n_kw(observers[i], EVENT_POS_ARGS, p_event->n_kw,
                                  p_event->args);
    }
    ++p_event;
  }
}

/**
 * @brief Sets timeouts for blocking and non-blocking operations
 *
 * .. method:: CardConnection.setTimeouts( atrTimeout=None,
 *                                         responseTimeout=None,
 *                                         maxTimeout=None )
 *
 *  Configures protocol timeouts, all arguments are optional:
 *
 *    - *atrTimeout* ATR timeout in ms, -1 = default
 *    - *responseTimeout* response timeout in ms, -1 = default
 *    - *maxTimeout* maximal response timeout in ms used when smart card
 *      (repeatedly) requesting timeout extension, -1 = default
 *
 * @param n_args    number of arguments
 * @param pos_args  positional arguments
 * @param kw_args   keyword arguments
 * @return          None
 */
STATIC mp_obj_t connection_setTimeouts(size_t n_args, const mp_obj_t* pos_args,
                                       mp_map_t* kw_args) {
  // Get self
  assert(n_args >= 1U);
  usb_connection_obj_t* self =
      (usb_connection_obj_t*)MP_OBJ_TO_PTR(pos_args[0]);

  // Parse and check arguments
  enum { ARG_atrTimeout = 0, ARG_responseTimeout, ARG_maxTimeout };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_atrTimeout, MP_ARG_OBJ, {.u_obj = mp_const_none}},
      {MP_QSTR_responseTimeout, MP_ARG_OBJ, {.u_obj = mp_const_none}},
      {MP_QSTR_maxTimeout, MP_ARG_OBJ, {.u_obj = mp_const_none}}};
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1U, pos_args + 1U, kw_args,
                   MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  // Check if connection is closed
  if (state_closed == self->state) {
    raise_CardConnectionException("connection is closed");
  }

  // Save parameters
  self->atr_timeout_ms =
      mp_obj_is_type(args[ARG_atrTimeout].u_obj, &mp_type_NoneType)
          ? proto_prm_unchanged
          : mp_obj_get_int(args[ARG_atrTimeout].u_obj);

  self->rsp_timeout_ms =
      mp_obj_is_type(args[ARG_responseTimeout].u_obj, &mp_type_NoneType)
          ? proto_prm_unchanged
          : mp_obj_get_int(args[ARG_responseTimeout].u_obj);

  self->max_timeout_ms =
      mp_obj_is_type(args[ARG_maxTimeout].u_obj, &mp_type_NoneType)
          ? proto_prm_unchanged
          : mp_obj_get_int(args[ARG_maxTimeout].u_obj);

  // Configure protocol if it is initialized
  if (self->protocol) {
    self->protocol->set_timeouts(self->proto_handle, self->atr_timeout_ms,
                                 self->rsp_timeout_ms, self->max_timeout_ms);
  }

  return mp_const_none;
}

/**
 * @brief Closes connection releasing hardware resources
 *
 * .. method:: UsbCardConnection.close()
 *
 *  Closes connection releasing hardware resources
 *
 * @param self_in  instance of CardConnection class
 * @return         None
 */
STATIC mp_obj_t connection_close(mp_obj_t self_in) {
  usb_connection_obj_t* self = (usb_connection_obj_t*)self_in;
  if (self->state != state_closed) {
    connection_disconnect(self_in);
    connection_deleteObservers(self_in);

    // Deinitialize timer
    if (self->timer) {
      mp_obj_t deinit_fn = mp_load_attr(self->timer, MP_QSTR_deinit);
      (void)mp_call_function_0(deinit_fn);
      self->timer = MP_OBJ_NULL;
    }
    // Detach from reader
    usbreader_deleteConnection(self->reader, MP_OBJ_FROM_PTR(self));
    self->reader = MP_OBJ_NULL;

    // Mark connection object as closed
    self->state = state_closed;
  }
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_disconnect_obj, connection_disconnect);
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(connection_connect_obj, 1, connection_connect);
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(connection_transmit_obj, 1, connection_transmit);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_getATR_obj, connection_getATR);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_getAPDU_obj, connection_getAPDU);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_isCardInserted_obj, connection_isCardInserted);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_getReader_obj, connection_getReader);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_getState_obj, connection_getState);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_isActive_obj, connection_isActive);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_isReady_obj, connection_isReady);
STATIC MP_DEFINE_CONST_FUN_OBJ_2(connection_addObserver_obj, connection_addObserver);
STATIC MP_DEFINE_CONST_FUN_OBJ_2(connection_deleteObserver_obj, connection_deleteObserver);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_deleteObservers_obj, connection_deleteObservers);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_countObservers_obj, connection_countObservers);
STATIC MP_DEFINE_CONST_FUN_OBJ_2(connection_notifyAll_obj, connection_notifyAll);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_close_obj, connection_close);
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(connection_setTimeouts_obj, 1, connection_setTimeouts);
#ifdef USB_DEBUG
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(connection_poweron_obj, 1, connection_poweron);
#endif
STATIC const mp_rom_map_elem_t connection_locals_dict_table[] = {
  // Instance methods
  { MP_ROM_QSTR(MP_QSTR___del__),         MP_ROM_PTR(&connection_close_obj)             },
  { MP_ROM_QSTR(MP_QSTR___enter__),       MP_ROM_PTR(&mp_identity_obj)                  },
  { MP_ROM_QSTR(MP_QSTR___exit__),        MP_ROM_PTR(&connection_close_obj)             },
  { MP_ROM_QSTR(MP_QSTR_connect),         MP_ROM_PTR(&connection_connect_obj)           },
#ifdef USB_DEBUG
  { MP_ROM_QSTR(MP_QSTR_poweron),         MP_ROM_PTR(&connection_poweron_obj)           },
#endif
  { MP_ROM_QSTR(MP_QSTR_disconnect),      MP_ROM_PTR(&connection_disconnect_obj)        },
  { MP_ROM_QSTR(MP_QSTR_transmit),        MP_ROM_PTR(&connection_transmit_obj)          },
  { MP_ROM_QSTR(MP_QSTR_getATR),          MP_ROM_PTR(&connection_getATR_obj)            },
  { MP_ROM_QSTR(MP_QSTR_getAPDU),         MP_ROM_PTR(&connection_getAPDU_obj)           },
  { MP_ROM_QSTR(MP_QSTR_isCardInserted),  MP_ROM_PTR(&connection_isCardInserted_obj)    },
  { MP_ROM_QSTR(MP_QSTR_getReader),       MP_ROM_PTR(&connection_getReader_obj)         },
  { MP_ROM_QSTR(MP_QSTR_getState),        MP_ROM_PTR(&connection_getState_obj)          },
  { MP_ROM_QSTR(MP_QSTR_isActive),        MP_ROM_PTR(&connection_isActive_obj)          },
  { MP_ROM_QSTR(MP_QSTR_isReady),         MP_ROM_PTR(&connection_isReady_obj)           },
  { MP_ROM_QSTR(MP_QSTR_addObserver),     MP_ROM_PTR(&connection_addObserver_obj)       },
  { MP_ROM_QSTR(MP_QSTR_deleteObserver),  MP_ROM_PTR(&connection_deleteObserver_obj)    },
  { MP_ROM_QSTR(MP_QSTR_deleteObservers), MP_ROM_PTR(&connection_deleteObservers_obj)   },
  { MP_ROM_QSTR(MP_QSTR_countObservers),  MP_ROM_PTR(&connection_countObservers_obj)    },
  { MP_ROM_QSTR(MP_QSTR_close),           MP_ROM_PTR(&connection_close_obj)             },
  { MP_ROM_QSTR(MP_QSTR__notifyAll),      MP_ROM_PTR(&connection_notifyAll_obj)         },
  { MP_ROM_QSTR(MP_QSTR_setTimeouts),     MP_ROM_PTR(&connection_setTimeouts_obj)       },
};

STATIC MP_DEFINE_CONST_DICT(connection_locals_dict, connection_locals_dict_table);
const mp_obj_type_t scard_UsbCardConnection_type = {
  { &mp_type_type },
  .name = MP_QSTR_CardConnection,
  .make_new = connection_make_new,
  .call = connection_call,
  .locals_dict = (void*)&connection_locals_dict,
};