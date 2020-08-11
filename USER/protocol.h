//
// Created by Frank Young on 2020/3/27.
//

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
    
    bool            C__ID_hash_table_init();
    uint8_t         C__IC_rec_buf_write(uint8_t Res, uint16_t USART_RX_STA);
    uint8_t         C__PC_rec_buf_write(uint8_t Res, uint16_t USART_RX_STA);
    uint8_t         C__PC_send_buf_write(const uint8_t& data);
    bool            C__set_IC_parse_flag(const int& stat);
    bool            C__set_PC_parse_flag(const int& stat);
    bool            C__get_IC_parse_flag();
    bool            C__get_PC_parse_flag();
    float           C__parse_IC_msg();
    bool            C__parse_PC_msg();
    void            C__init();

#ifdef __cplusplus
}
#endif






#endif //WEEDER_CONTROLLER_PROTOCOL_H