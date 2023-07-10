#ifndef  __LINK_PROTOCOL_H
#define  __LINK_PROTOCOL_H
#include "common.h"
#include "crc.h"
#define UTILS_LEN_MAX       (720)
typedef struct
{
    uint8_t start;
    uint8_t version;
    uint8_t controlbits;
    uint8_t reserved;
    uint8_t dest_addr;
    uint8_t src_addr;
    uint8_t message_id;
    uint8_t response_id;
    uint8_t len;
} protocol_v10_t;

typedef struct
{
    uint8_t start;
    uint8_t version;
    uint8_t controlbits;
    uint8_t reserved;
    uint8_t dest_addr;
    uint8_t src_addr;
    uint8_t message_id;
    uint8_t response_id;
    uint16_t len;
} protocol_v20_t;

typedef struct
{
    uint8_t start;
    uint8_t version;
    uint8_t controlbits;
    uint8_t reserved;
    uint16_t dest_addr;
    uint16_t src_addr;
    uint8_t message_id;
    uint8_t response_id;
    uint16_t len;
} protocol_v30_t;

typedef struct
{
    uint8_t start;
    uint8_t version;
    uint8_t controlbits;
    uint8_t reserved;
    uint32_t dest_addr;
    uint32_t src_addr;
    uint16_t message_id;
    uint16_t response_id;
    uint32_t len;
} protocol_v40_t;

typedef struct
{
    uint8_t start;
    uint8_t version;
    uint8_t controlbits;
    uint8_t reserved;
    uint64_t dest_addr;
    uint64_t src_addr;
    uint32_t message_id;
    uint32_t response_id;
    uint32_t len;
} protocol_v50_t;


typedef enum
{
    parse_state_none,
    parse_ESC,
    parse_other,
    parse_data,
    parse_finish,
} parse_state_t;

typedef struct
{
    uint8_t version;
    uint8_t controlbits;
    uint64_t dest_addr;
    uint64_t src_addr;
    uint32_t response_id;
    uint8_t *payload;
    uint32_t payload_len;
    uint8_t *frame_buf;
    uint32_t message_id;
    crc16_t pack_crc;
    uint32_t frame_len;
} frame_config_t;

typedef struct
{
    union
    {
        uint8_t data[UTILS_LEN_MAX];
        protocol_v10_t frame_v10;
        protocol_v20_t frame_v20;
        protocol_v30_t frame_v30;
        protocol_v40_t frame_v40;
        protocol_v50_t frame_v50;
    } data_frame_u;
    crc16_t crc16;
    uint8_t stop;
    uint8_t *payload;
    uint8_t payload_len;
    uint8_t version;
    uint8_t controlbits;
    parse_state_t parse_state;
    uint32_t message_id;
    uint8_t index;
} frame_t;
void pack(frame_config_t *frame);
void parse(char c,frame_t *fram);
#endif
