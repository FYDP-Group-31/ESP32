#ifndef _SLAVE_COMM_H_
#define _SLAVE_COMM_H_

typedef enum {
    MASTER_NOT_READY = 0UL,
    MASTER_READY = 1UL
} master_ready_t;

extern volatile bool slave_ready_status;

void slave_comm_init(void);
void master_set_ready(master_ready_t ready);

#endif
