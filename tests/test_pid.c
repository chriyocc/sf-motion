/* Hardware-free regression test:
 * cc -O3 -ffast-math -Ilib/PID tests/test_pid.c lib/PID/pid_utils.c -o test_pid
 * ./test_pid
 */
#include "pid_utils.h"
#include <assert.h>
#include <string.h>

static PID_Controller_t controller(void) {
    PID_Controller_t pid = {0};
    assert(pid_set_ts(&pid, 0.01f) == 0);
    assert(pid_set_kp(&pid, 2.0f) == 0);
    assert(pid_set_ki(&pid, 1.0f) == 0);
    assert(pid_set_out_constraint(&pid, 1.0f, -1.0f) == 0);
    assert(pid_set_d_filter_fc(&pid, 37.0f) == 0);
    assert(pid_set_max_d(&pid, 100.0f) == 0);
    return pid;
}

static void check_anti_windup(float (*control)(PID_Controller_t *, float)) {
    for (int sign = -1; sign <= 1; sign += 2) {
        PID_Controller_t pid = controller();
        for (int i = 0; i < 1000; ++i) {
            assert(control(&pid, sign * 10.0f) == sign * 1.0f);
        }
        assert(pid.integral == 0.0f);
        assert(control(&pid, 0.0f) == 0.0f);
        /* Opposing error must still unwind an already saturated integral. */
        pid.integral = sign * 5.0f;
        control(&pid, sign * -0.1f);
        assert(sign * pid.integral < 5.0f);
    }
    PID_Controller_t pid = controller();
    control(&pid, 0.1f);
    assert(pid.integral > 0.0f);
}

int main(void) {
    check_anti_windup(pi_control);
    check_anti_windup(pid_control);

    /* D alone can saturate PID even with a small proportional term. */
    PID_Controller_t pid = controller();
    pid.kp = 0;
    pid.kd = 1;
    pid.d_alpha_filter = 1;
    assert(pid_control(&pid, 0.1f) == 1.0f);
    assert(pid.integral == 0.0f);

    pid = controller();
    PID_Controller_t before = pid;
    assert(pid_set_kp(&pid, -1) != 0);
    assert(pid_set_ki(&pid, -1) != 0);
    assert(pid_set_kd(&pid, -1) != 0);
    assert(pid_set_out_constraint(&pid, -2, 2) != 0);
    assert(pid_set_deadband(&pid, -1) != 0);
    assert(pid_set_d_filter_fc(&pid, 0) != 0);
    assert(pid_set_d_filter_fc(&pid, -1) != 0);
    assert(memcmp(&pid, &before, sizeof(pid)) == 0);

    /* Reject NaN and infinity even under -ffast-math. */
    const uint32_t invalid_bits[] = {0x7fc00000u, 0x7f800000u, 0xff800000u};
    for (unsigned i = 0; i < sizeof(invalid_bits) / sizeof(invalid_bits[0]); ++i) {
        float invalid;
        memcpy(&invalid, &invalid_bits[i], sizeof(invalid));
        assert(pid_set_kp(&pid, invalid) != 0);
        assert(pid_set_ki(&pid, invalid) != 0);
        assert(pid_set_kd(&pid, invalid) != 0);
        assert(pid_set_out_constraint(&pid, invalid, -1) != 0);
        assert(pid_set_d_filter_fc(&pid, invalid) != 0);
        assert(memcmp(&pid, &before, sizeof(pid)) == 0);
    }

    /* Match startup: load a saved cutoff, then initialize the time step. */
    pid.ts = 0;
    assert(pid_set_d_filter_fc(&pid, 37) == 0);
    assert(pid_set_ts(&pid, 0.001f) == 0);
    assert(pid_set_d_filter_fc(&pid, pid.d_fc_lpf) == 0);
    assert(pid.d_fc_lpf == 37);
    assert(pid.d_alpha_filter > 0.18f && pid.d_alpha_filter < 0.20f);
    pid.d_filtered = 10;
    pid_reset(&pid);
    assert(pid.d_filtered == 0);
    return 0;
}
