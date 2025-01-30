#ifndef __DW3000_ANCHOR_H__
#define __DW3000_ANCHOR_H__

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include "dw3000.h"

#define DW3000_ANCHOR_SERIAL_BAUDRATE   115200
#define DW3000_ANCHOR_PIN_RST           27
#define DW3000_ANCHOR_PIN_IRQ           34
#define DW3000_ANCHOR_PIN_SS            4
#define DW3000_ANCHOR_TX_ANTENNA_DELAY  16385
#define DW3000_ANCHOR_RX_ANTENNA_DELAY  16385
#define DW3000_ANCHOR_RX2TX_DELAY_UUS   450
#define DW3000_ANCHOR_TX2RX_DELAY_UUS   240
#define DW3000_ANCHOR_MAX_FRAME_LEN     128

#define DW3000_ANCHOR_NETWORK_SCAN_TIMEOUT  10000000
#define DW3000_ANCHOR_NETWORK_SCAN_TRIAL    40

#define DW3000_ANCHOR_RX_TIMEOUT_UUS        400

namespace uwbips {
    
    extern SPISettings _fastSPI;
    extern dwt_config_t dw3000_config;
    extern dwt_txconfig_t dw3000_txconfig;

    class DW3000Anchor {
        
        public:

            void configure(uint8_t pan_id_1, uint8_t pan_id_2, uint8_t device_addr_1, uint8_t device_addr_2);
            void spin();

        private:
        
            enum system_state_t {
                STATE_HIERARCHY_CHECK,
                STATE_BROADCAST,
                STATE_CLOCK_CALIB,
                STATE_CLOCK_SYNC,
                STATE_TAG_CALL
            };

            enum hierarchy_t {
                HIERARCHY_MASTER,
                HIERARCHY_SLAVE
            };

            struct phy_dataframe_t {
                uint8_t frame_control[2],
                        sequence_number,
                        pan_id[2],
                        dest_addr[2],
                        src_addr[2],
                        function_code,
                        payload[116],
                        checksum[2] = {0x00, 0x00};
            };

            system_state_t  system_state;

            hierarchy_t     hierarchy;
            
            uint8_t         pan_id[2],
                            device_addr[2];

            uint32_t        status_reg;
            
            Preferences     filesystem;

            void beginSystem();
            void beginDW3000();
            void beginDisplay();

            void stActNetworkScan();
            void stActBroadcast();
            void stActClockCalib();
            void stActClockSync();
            void stActTagCall();

            void        uwbTx(uint8_t *phy_dataframe);
            uint8_t*    uwbRx(uint32_t timeout_us, bool get_broadcast, uint8_t *source_addr = nullptr);
            uint8_t*    uwbTxThenRx(uint8_t *phy_dataframe, uint32_t timeout_us);

            phy_dataframe_t getPHYDataframeTemplate();
            phy_dataframe_t getPHYDataframeFromBytes(const uint8_t *data);
            uint8_t*        getBytesFromPHYDataframe(const phy_dataframe_t *data);
    };
}

#endif