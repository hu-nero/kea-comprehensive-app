#ifndef _HIL_PROTOCOL_H
#define _HIL_PROTOCOL_H

#include <stdint.h>

typedef enum
{
    HIL_ID_KPE_1 = 0x1001,
    HIL_ID_KPE_2 = 0x1002,
    HIL_ID_KPE_3 = 0x1003,
    HIL_ID_KPL_1 = 0x2001,
    HIL_ID_KPL_1_2 = 0x2101,
    HIL_ID_KPL_2 = 0x2002,
    HIL_ID_KPL_2_2 = 0x2102,
    HIL_ID_BAL_1 = 0x3001
} HIL_PRO_ID_enum;

void HIL_protocol_handle(const uint32_t can_id, uint8_t *can_data);


#endif


