#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum { ENCODER_LEFT, ENCODER_RIGHT } encoder_side_t;

void encoder_init(void);
int32_t encoder_get_count(encoder_side_t side);
void encoder_reset(encoder_side_t side);

#endif  // ENCODER_H
