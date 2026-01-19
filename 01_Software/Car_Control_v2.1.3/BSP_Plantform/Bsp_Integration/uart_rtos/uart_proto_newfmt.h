#ifndef __UART_PROTO_NEWFMT_H__
#define __UART_PROTO_NEWFMT_H__

#include <stdint.h>
#include <stdbool.h>

#ifndef H1
#define H1 0xA0
#endif
#ifndef H2
#define H2 0x0A
#endif

#define UART_PROTO_VER         0x01
#define UART_MAX_PAYLOAD       128u
#define UART_MAX_FRAME         (7u + UART_MAX_PAYLOAD + 2u)  /* hdr7 + payload + crc2 */

typedef struct {
    uint8_t ver;
    uint8_t seq;
    uint8_t type;
    uint16_t len;
    const uint8_t *payload;  /* 指向 parser 内部缓存（当场处理） */
} uart_frame_t;

typedef struct {
    uint8_t  buf[UART_MAX_FRAME];
    uint16_t pos;
    uint16_t need;
    uint16_t payload_len;
    enum {
        ST_FIND_H1 = 0,
        ST_FIND_H2,
        ST_READ_HDR,        /* VER SEQ TYPE LEN_L LEN_H */
        ST_READ_PAYLOAD_CRC
    } st;
} uart_parser_t;

typedef struct {
    uint8_t *out;
    uint16_t max;
    uint16_t w;
    uint16_t payload_start;
} uart_builder_t;

uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

/* Parser */
void uart_parser_init(uart_parser_t *p);
bool uart_parser_feed(uart_parser_t *p, uint8_t byte, uart_frame_t *out_frame);

/* Builder */
void uart_builder_begin(uart_builder_t *b, uint8_t *out, uint16_t max,
                        uint8_t seq, uint8_t type);
bool uart_builder_add_tlv(uart_builder_t *b, uint8_t tag, const void *v, uint8_t vlen);
bool uart_builder_add_u8 (uart_builder_t *b, uint8_t tag, uint8_t  v);
bool uart_builder_add_u16(uart_builder_t *b, uint8_t tag, uint16_t v);
bool uart_builder_add_s32(uart_builder_t *b, uint8_t tag, int32_t  v);
bool uart_builder_end(uart_builder_t *b, uint16_t *out_len);

#endif
