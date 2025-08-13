#ifndef WS22_CTRL_SM_H
#define WS22_CTRL_SM_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Cruise Control FSM (cc_sm legacy API) ---------- */
typedef enum {
    CC_STATE_DISABLED = 0,
    CC_STATE_STANDBY,
    CC_STATE_ACTIVE,
    CC_STATE_FAULT
} CC_State;

typedef enum {
    CC_EVENT_INIT = 0,
    CC_EVENT_ENABLE_CMD,
    CC_EVENT_DISABLE_CMD,
    CC_EVENT_BRAKE_PEDAL,
    CC_EVENT_NEUTRAL,
    CC_EVENT_ERROR
} CC_Event;

/* Initialize / drive the cruise‑control state machine */
void        CC_SM_Init(void);
void        CC_SM_HandleEvent(CC_Event ev);
CC_State    CC_SM_GetState(void);

/* ---------- WS22 top‑level torque vs cruise selection ------ */
typedef enum {
    WS22_CTRL_TORQUE = 0,
    WS22_CTRL_CRUISE
} WS22_CtrlMode_t;

/* Exposed current mode variable (read‑only to other modules) */
extern volatile WS22_CtrlMode_t ws22_ctrl_mode;

/* Update function called once per control loop iteration */
void WS22_FSM_Update(void);


#ifdef __cplusplus
}
#endif

#endif /* WS22_CTRL_SM_H */