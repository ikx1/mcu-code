#include "host_protocol.h"

#include <string.h>

static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void wr_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void uart_parser_init(uart_parser_t *p)
{
    if (p == NULL) {
        return;
    }

    p->pos = 0;
    p->need = 0;
    p->payload_len = 0;
    p->st = ST_FIND_H1;
}

bool uart_parser_feed(uart_parser_t *p, uint8_t byte, uart_frame_t *out_frame)
{
    if ((p == NULL) || (out_frame == NULL)) {
        return false;
    }

    switch (p->st)
    {
    case ST_FIND_H1:
        if (byte == H1) {
            p->buf[0] = byte;
            p->pos = 1;
            p->st = ST_FIND_H2;
        }
        break;

    case ST_FIND_H2:
        if (byte == H2) {
            p->buf[p->pos++] = byte;
            p->need = 2 + 5;
            p->st = ST_READ_HDR;
        } else if (byte == H1) {
            p->buf[0] = byte;
            p->pos = 1;
            p->st = ST_FIND_H2;
        } else {
            p->st = ST_FIND_H1;
            p->pos = 0;
        }
        break;

    case ST_READ_HDR:
        p->buf[p->pos++] = byte;
        if (p->pos >= p->need)
        {
            uint8_t ver = p->buf[2];
            uint16_t len = rd_le16(&p->buf[5]);

            if (ver != UART_PROTO_VER || len > UART_MAX_PAYLOAD) {
                uart_parser_init(p);
                break;
            }

            p->payload_len = len;
            p->need = (uint16_t)(2 + 5 + len + 2);
            p->st = ST_READ_PAYLOAD_CRC;
        }
        break;

    case ST_READ_PAYLOAD_CRC:
        p->buf[p->pos++] = byte;
        if (p->pos >= p->need)
        {
            uint16_t frame_len = p->need;
            uint16_t crc_rx = rd_le16(&p->buf[frame_len - 2]);
            uint16_t crc_calc = crc16_modbus(&p->buf[2], (uint16_t)(5 + p->payload_len));

            if (crc_calc == crc_rx)
            {
                out_frame->ver = p->buf[2];
                out_frame->seq = p->buf[3];
                out_frame->type = p->buf[4];
                out_frame->len = p->payload_len;
                out_frame->payload = &p->buf[7];
                uart_parser_init(p);
                return true;
            }

            uart_parser_init(p);
        }
        break;

    default:
        uart_parser_init(p);
        break;
    }

    return false;
}

void uart_builder_begin(uart_builder_t *b, uint8_t *out, uint16_t max,
                        uint8_t seq, uint8_t type)
{
    if (b == NULL) {
        return;
    }

    b->out = out;
    b->max = max;
    b->w = 0;
    if (!out || max < 7u) {
        b->payload_start = 0;
        return;
    }

    out[b->w++] = H1;
    out[b->w++] = H2;
    out[b->w++] = UART_PROTO_VER;
    out[b->w++] = seq;
    out[b->w++] = type;
    out[b->w++] = 0;
    out[b->w++] = 0;

    b->payload_start = b->w;
}

bool uart_builder_add_tlv(uart_builder_t *b, uint8_t tag, const void *v, uint8_t vlen)
{
    if ((b == NULL) || (b->out == NULL)) {
        return false;
    }
    if ((v == NULL) && (vlen != 0u)) {
        return false;
    }

    if ((uint32_t)b->w + 2u + (uint32_t)vlen + 2u > b->max) {
        return false;
    }

    b->out[b->w++] = tag;
    b->out[b->w++] = vlen;
    memcpy(&b->out[b->w], v, vlen);
    b->w += vlen;
    return true;
}

bool uart_builder_add_u8(uart_builder_t *b, uint8_t tag, uint8_t v)
{
    return uart_builder_add_tlv(b, tag, &v, 1);
}

bool uart_builder_add_u16(uart_builder_t *b, uint8_t tag, uint16_t v)
{
    uint8_t t[2];

    t[0] = (uint8_t)(v & 0xFF);
    t[1] = (uint8_t)(v >> 8);
    return uart_builder_add_tlv(b, tag, t, 2);
}

bool uart_builder_add_s32(uart_builder_t *b, uint8_t tag, int32_t v)
{
    uint8_t t[4];

    t[0] = (uint8_t)(v & 0xFF);
    t[1] = (uint8_t)((v >> 8) & 0xFF);
    t[2] = (uint8_t)((v >> 16) & 0xFF);
    t[3] = (uint8_t)((v >> 24) & 0xFF);
    return uart_builder_add_tlv(b, tag, t, 4);
}

bool uart_builder_end(uart_builder_t *b, uint16_t *out_len)
{
    uint16_t payload_len;
    uint16_t crc;

    if (!b || !b->out || !out_len || b->max < 7u) {
        return false;
    }
    if (b->payload_start < 7u || b->payload_start > b->w) {
        return false;
    }
    if (b->w > b->max) {
        return false;
    }

    payload_len = (uint16_t)(b->w - b->payload_start);
    if (payload_len > UART_MAX_PAYLOAD) {
        return false;
    }

    wr_le16(&b->out[5], payload_len);
    crc = crc16_modbus(&b->out[2], (uint16_t)(5 + payload_len));

    if (b->w + 2 > b->max) {
        return false;
    }

    b->out[b->w++] = (uint8_t)(crc & 0xFF);
    b->out[b->w++] = (uint8_t)(crc >> 8);

    *out_len = b->w;
    return true;
}
