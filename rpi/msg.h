#ifndef MSG_H
#define MSG_H

#include <stdint.h>

struct s_msg {
        uint8_t t; //type
        int16_t v; //value
};
#endif
