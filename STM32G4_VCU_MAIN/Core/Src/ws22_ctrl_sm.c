#include "ws22_ctrl_sm.h"
#include "vcu_state.h"
#include "vcu_hal.h"

/* ---------- Torque vs Cruise mode selector ---------------------
   Cruise engage/disengage decision relies *only* on:
     - cruise_control_on  (set by cc.c)
     - gear state         (drive_direction / drive_neutral)
   Throttle input **does NOT** cancel cruise anymore.
-----------------------------------------------------------------*/

volatile WS22_CtrlMode_t ws22_ctrl_mode = WS22_CTRL_TORQUE;

void WS22_FSM_Update(void)
{
    bool gear_fwd  = drive_direction;
    bool gear_safe = (!drive_neutral && gear_fwd);

    switch (ws22_ctrl_mode)
    {
        case WS22_CTRL_TORQUE:
            /* Enter cruise when CC armed+active and forward gear */
            if (cruise_control_on && gear_safe)
                ws22_ctrl_mode = WS22_CTRL_CRUISE;
            break;

        case WS22_CTRL_CRUISE:
            /* Exit cruise if CC disabled by upper FSM or gear no longer safe */
            if (!cruise_control_on || !gear_safe)
                ws22_ctrl_mode = WS22_CTRL_TORQUE;
            break;

        default:
            ws22_ctrl_mode = WS22_CTRL_TORQUE;
            break;
    }

    /* Redundant safety: never stay in CRUISE with unsafe gear */
    if (!gear_safe)
        ws22_ctrl_mode = WS22_CTRL_TORQUE;

}