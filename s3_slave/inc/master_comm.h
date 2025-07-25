#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

typedef enum {
    SLAVE_NOT_READY = 0UL,
    SLAVE_READY = 1UL
} slave_ready_t;

extern volatile bool master_ready_status;

void master_comm_init(void);
void slave_set_ready(slave_ready_t ready);

#endif
