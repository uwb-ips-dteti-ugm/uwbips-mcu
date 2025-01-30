#include "dw3000_anchor.h"



SPISettings uwbips::_fastSPI;
dwt_config_t uwbips::dw3000_config = {
    5,
    DWT_PLEN_128,
    DWT_PAC8,
    9,
    9,
    0,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};
dwt_txconfig_t uwbips::dw3000_txconfig;



void uwbips::DW3000Anchor::configure(uint8_t pan_id_1, uint8_t pan_id_2, uint8_t device_addr_1, uint8_t device_addr_2) {

    this->pan_id[0]         = pan_id_1;
    this->pan_id[1]         = pan_id_2;
    this->device_addr[0]    = device_addr_1;
    this->device_addr[1]    = device_addr_2;

    this->system_state = uwbips::DW3000Anchor::STATE_HIERARCHY_CHECK;

    this->beginSystem();
    this->beginDW3000();
    this->beginDisplay();

    Serial.println("[INFO] Configuration completed successfully.");
}



void uwbips::DW3000Anchor::spin() {

    switch(this->system_state) {

        case uwbips::DW3000Anchor::STATE_HIERARCHY_CHECK:
            this->stActNetworkScan();
            break;

        
        case uwbips::DW3000Anchor::STATE_BROADCAST:
            this->stActBroadcast();
            break;


        case uwbips::DW3000Anchor::STATE_CLOCK_CALIB:
            this->stActClockCalib();
            break;


        case uwbips::DW3000Anchor::STATE_CLOCK_SYNC:
            this->stActClockSync();
            break;


        case uwbips::DW3000Anchor::STATE_TAG_CALL:
            this->stActTagCall();
            break;
    }
}



void uwbips::DW3000Anchor::beginSystem() {

    Serial.begin(DW3000_ANCHOR_SERIAL_BAUDRATE);
    Serial.println("[INFO] Serial communication started.");

    this->filesystem.begin("UWBIPS_DW3000_ANCHOR");
}



void uwbips::DW3000Anchor::beginDW3000() {
    
    uwbips::_fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
    spiBegin(DW3000_ANCHOR_PIN_IRQ, DW3000_ANCHOR_PIN_RST);
    spiSelect(DW3000_ANCHOR_PIN_SS);
    delay(2);

    if(!dwt_checkidlerc()) {
        Serial.println("[ERROR]\tIdle return code error.");
        Serial.println("[INFO]\tHalted.");
        while(1);
    }
    if(dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        Serial.println("[ERROR]\tFailed to initialise DW3000.");
        Serial.println("[INFO]\tHalted.");
        while(1);
    }
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    if(dwt_configure(&uwbips::dw3000_config)) {
        Serial.println("[ERROR]\tFailed to configure DW3000.");
        Serial.println("[INFO]\tHalted.");
        while(1);
    }
    dwt_configuretxrf(&uwbips::dw3000_txconfig);
    dwt_setrxantennadelay(DW3000_ANCHOR_RX_ANTENNA_DELAY);
    dwt_settxantennadelay(DW3000_ANCHOR_TX_ANTENNA_DELAY);
    dwt_setrxaftertxdelay(DW3000_ANCHOR_TX2RX_DELAY_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}



void uwbips::DW3000Anchor::beginDisplay() {

}



void uwbips::DW3000Anchor::stActNetworkScan() {

}



void uwbips::DW3000Anchor::stActBroadcast() {

}



void uwbips::DW3000Anchor::stActClockCalib() {

}



void uwbips::DW3000Anchor::stActClockSync() {

}



void uwbips::DW3000Anchor::stActTagCall() {

}



void uwbips::DW3000Anchor::uwbTx(uint8_t *phy_dataframe) {
    
    size_t data_size = DW3000_ANCHOR_MAX_FRAME_LEN * sizeof(uint8_t);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(data_size, phy_dataframe, 0);
    dwt_writetxfctrl(data_size, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
}



uint8_t* uwbips::DW3000Anchor::uwbRx(uint32_t timeout_us, bool get_broadcast, uint8_t *source_addr = nullptr) {

    uint8_t     broadcast_addr[2]   =  {0xFF, 0xFF};
    uint64_t    start_us            = micros(),
                diff_us;

    while((diff_us = micros() - start_us) < timeout_us) {

        dwt_setrxtimeout(timeout_us - (uint32_t)diff_us);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while(!(
            (this->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
            (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
        ));

        if(this->status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

            uint8_t     *rx_buffer = new uint8_t[DW3000_ANCHOR_MAX_FRAME_LEN];
            uint32_t    frame_len;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            dwt_readrxdata(rx_buffer, frame_len, 0);

            if(get_broadcast) {
                if(memcmp(rx_buffer + 5, broadcast_addr, 2) == 0) return rx_buffer;
            }
            else if(memcmp(rx_buffer + 5, this->device_addr, 2) == 0) {

                if(source_addr == nullptr || memcmp(rx_buffer + 7, source_addr, 2) == 0) return rx_buffer;
                else {
                    delete[] rx_buffer;
                    continue;
                }
            }
            else {
                delete[] rx_buffer;
                continue;
            }
        }
        else {

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            Serial.print("[ERROR] UWB RX timeout triggered: ");
            Serial.print(timeout_us);
            Serial.println(" uus.");
            return nullptr;
        }
    }

    Serial.print("[ERROR] RX timeout: ");
    Serial.print(timeout_us);
    Serial.println(" uus.");
    return nullptr;
}



uint8_t* uwbips::DW3000Anchor::uwbTxThenRx(uint8_t *phy_dataframe, uint32_t timeout_us) {
    
    size_t      data_size       = DW3000_ANCHOR_MAX_FRAME_LEN * sizeof(uint8_t);
    uint8_t     *target_addr    = phy_dataframe + 5;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(data_size, phy_dataframe, 0);
    dwt_writetxfctrl(data_size, 0, 0);

    dwt_setrxtimeout(timeout_us);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    while (!((this->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    if(this->status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        
    }
    else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}



uwbips::DW3000Anchor::phy_dataframe_t uwbips::DW3000Anchor::getPHYDataframeTemplate() {

    uwbips::DW3000Anchor::phy_dataframe_t dataframe;
    dataframe.frame_control[0]  = 0x41;
    dataframe.frame_control[1]  = 0x88;
    memcpy(dataframe.pan_id, this->pan_id, sizeof(this->pan_id));
    memcpy(dataframe.src_addr, this->device_addr, sizeof(this->device_addr));

    return dataframe;
}



uint8_t* uwbips::DW3000Anchor::getBytesFromPHYDataframe(const uwbips::DW3000Anchor::phy_dataframe_t *data) {
    
    uint8_t *dataframe = new uint8_t[DW3000_ANCHOR_MAX_FRAME_LEN];

    dataframe[2] = data->sequence_number;
    dataframe[9] = data->function_code;
    memcpy(dataframe + 0, data->frame_control, sizeof(data->frame_control));
    memcpy(dataframe + 3, data->pan_id, sizeof(data->pan_id));
    memcpy(dataframe + 5, data->dest_addr, sizeof(data->dest_addr));
    memcpy(dataframe + 7, data->src_addr, sizeof(data->src_addr));
    memcpy(dataframe + 10, data->payload, sizeof(data->payload));

    return dataframe;
}