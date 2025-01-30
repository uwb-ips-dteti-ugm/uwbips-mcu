#include "uwbsdk.h"


#ifdef UWB_DW3000
    dwt_config_t uwbsdk::dw3000_config = {
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
    dwt_txconfig_t  uwbsdk::dw3000_txconfig;
    SPISettings     uwbsdk::_fastSPI;
#endif



void uwbsdk::UWBController::begin(uint8_t irq_pin, uint8_t rst_pin, uint8_t ss_pin, uint16_t tx_antenna_delay=16385, uint16_t rx_antenna_delay=16385) { 
    
    Serial.begin(115200);
    this->prefs.begin("UWBSDK", false);

    #ifdef UWB_DW3000

        this->tx_antenna_delay = tx_antenna_delay;
        this->rx_antenna_delay = rx_antenna_delay;

        uwbsdk::_fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
        spiBegin(irq_pin, rst_pin);
        spiSelect(ss_pin);
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
        if(dwt_configure(&uwbsdk::dw3000_config)) {
            Serial.println("[ERROR]\tFailed to configure DW3000.");
            Serial.println("[INFO]\tHalted.");
            while(1);
        }
        dwt_configuretxrf(&uwbsdk::dw3000_txconfig);
        dwt_settxantennadelay(this->tx_antenna_delay);
        dwt_setrxantennadelay(this->rx_antenna_delay);
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    
    #endif
}



void uwbsdk::UWBController::uwbTx(uwbsdk::UWBFrame_t data) {

    #ifdef UWB_DW3000

        uint8_t *databytes = this->frameToBytes(&data);

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(this->frame_size, databytes, 0);
        dwt_writetxfctrl(this->frame_size, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        delete[] databytes;

    #endif
}



uwbsdk::UWBFrame_t uwbsdk::UWBController::uwbRx(uint32_t timeout_us=0) {

    #ifdef UWB_DW3000

        memset(this->rx_buffer, 0, sizeof(this->rx_buffer));
        dwt_setrxtimeout(timeout_us);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        if (timeout_us == 0) {
            while (!(
                (this->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)
            ));
        }
        else {
            while (!(
                (this->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
            ));
        }

        if (this->status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

            if (frame_len <= sizeof(this->rx_buffer)) {
                dwt_readrxdata(this->rx_buffer, frame_len, 0);
                return this->bytesToFrame(this->rx_buffer);
            }
            else return {};
        }
        
        else {
            if (timeout_us == 0)
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            else
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            return {};
        }
    
    #endif
}



uwbsdk::UWBFrame_t uwbsdk::UWBController::uwbTxThenRx(uwbsdk::UWBFrame_t data, uint32_t rx_delay_us, uint32_t timeout_us=0) {

    #ifdef UWB_DW3000

        uint8_t *databytes = this->frameToBytes(&data);

        memset(this->rx_buffer, 0, sizeof(this->rx_buffer));
        dwt_setrxaftertxdelay(rx_delay_us);
        dwt_setrxtimeout(timeout_us);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(this->frame_size, databytes, 0);
        dwt_writetxfctrl(this->frame_size, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        delete[] databytes;

        if (timeout_us == 0) {
            while (!(
                (this->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)
            ));
        }
        else {
            while (!(
                (this->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
            ));
        }

        if (this->status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

            if (frame_len <= sizeof(this->rx_buffer)) {
                dwt_readrxdata(this->rx_buffer, frame_len, 0);
                return this->bytesToFrame(this->rx_buffer);
            }
            else return {};
        }
        
        else {
            if (timeout_us == 0)
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            else
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            return {};
        }

    #endif
}



bool uwbsdk::UWBController::uwbDelayedTx(uwbsdk::UWBFrame_t data, uint32_t tx_delay_us) {

    #ifdef UWB_DW3000

        uint8_t *databytes = this->frameToBytes(&data);

        dwt_setdelayedtrxtime(tx_delay_us);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(this->frame_size, databytes, 0);
        dwt_writetxfctrl(this->frame_size, 0, 0);
        
        int ret = dwt_starttx(DWT_START_TX_DELAYED);
        delete[] databytes;

        if (ret == DWT_SUCCESS) {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            return true;
        }
        else return false;

    #endif
}



uint8_t* uwbsdk::UWBController::frameToBytes(uwbsdk::UWBFrame_t *data) {

    uint8_t *ret_bytes = new uint8_t[127];
    *(ret_bytes + 125) = 0x00;
    *(ret_bytes + 126) = 0x00;

    memcpy(ret_bytes, data->frame_control, 2);
    memcpy(ret_bytes + 2, &data->seq_num, 1);
    memcpy(ret_bytes + 3, data->pan_id, 2);
    memcpy(ret_bytes + 5, data->dst_addr, 2);
    memcpy(ret_bytes + 7, data->src_addr, 2);
    memcpy(ret_bytes + 9, &data->func_code, 1);
    memcpy(ret_bytes + 10, data->payload, 115);

    return ret_bytes;
}



uwbsdk::UWBFrame_t uwbsdk::UWBController::bytesToFrame(uint8_t* data) {

    uwbsdk::UWBFrame_t ret_frame;

    memcpy(ret_frame.frame_control, data, 2);
    memcpy(&ret_frame.seq_num, data + 2, 1);
    memcpy(ret_frame.pan_id, data + 3, 2);
    memcpy(ret_frame.dst_addr, data + 5, 2);
    memcpy(ret_frame.src_addr, data + 7, 2);
    memcpy(&ret_frame.func_code, data + 9, 1);
    memcpy(ret_frame.payload, data + 10, 115);

    ret_frame.is_ok = true;
    return ret_frame;
}