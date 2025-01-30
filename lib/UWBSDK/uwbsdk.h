#ifndef __UWBSDK_H__
#define __UWBSDK_H__

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>

#define UWB_DW3000

#ifdef UWB_DW1000
    #include <DW1000.h>
#endif

#ifdef UWB_DW3000
    #include <dw3000.h>
#endif

namespace uwbsdk {
    
    #ifdef UWB_DW3000
        extern dwt_config_t     dw3000_config;
        extern dwt_txconfig_t   dw3000_txconfig;
        extern SPISettings      _fastSPI;
    #endif

    struct UWBFrame_t {
        bool    is_ok = false;
        uint8_t frame_control[2],
                seq_num,
                pan_id[2],
                dst_addr[2],
                src_addr[2],
                func_code,
                payload[115],
                checksum[2] = {0x00, 0x00};
    };

    class UWBController {

        public:
            void        begin(uint8_t irq_pin, uint8_t rst_pin, uint8_t ss_pin, uint16_t tx_antenna_delay=16385, uint16_t rx_antenna_delay=16385);
            void        uwbTx(UWBFrame_t data);
            UWBFrame_t  uwbRx(uint32_t timeout_us=0);
            UWBFrame_t  uwbTxThenRx(UWBFrame_t data, uint32_t rx_delay_us, uint32_t timeout_us=0);
            bool        uwbDelayedTx(UWBFrame_t data, uint32_t tx_delay_us);

        private:
            size_t      frame_size = 127*sizeof(uint8_t);
            uint8_t     rx_buffer[127];
            uint16_t    tx_antenna_delay,
                        rx_antenna_delay;
            uint32_t    status_reg;
            Preferences prefs;

            uint8_t*    frameToBytes(UWBFrame_t *data);
            UWBFrame_t  bytesToFrame(uint8_t *data);
    };
}

#endif