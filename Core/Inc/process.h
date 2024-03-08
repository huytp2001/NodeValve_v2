#ifndef PROCESS_H
#define PROCESS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "frame.h"

#define WAITING_CTR_TIMEOUT_SEC 5
#define WAITING_ACK_TIMEOUT_SEC 4

void v_process_ctr_frame(stru_frame_t *stru_frame_in);
void v_process_rep_frame(stru_frame_t *stru_frame_in);
void v_process_wait_ctr(void);
void v_process_wait_rep(void);
void v_process_wait_ack(stru_frame_t *stru_frame_in, bool b_direction);

#ifdef __cplusplus
}
#endif
#endif



