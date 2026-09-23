# Hardware-free control checks

Run the Python GUI/protocol tests from the repository root:

```sh
python -B -m unittest discover -s communication/tests -v
```

They use mocked serial connections and offscreen Qt. No board is needed.

With a native C compiler, run the PID saturation, validation, and filter tests:

```sh
cc -O3 -ffast-math -Ilib/PID tests/test_pid.c lib/PID/pid_utils.c -o test_pid
./test_pid
```

The fast-math option matches the firmware's floating-point optimization assumptions.
Do not define `NDEBUG`: these tests use assertions.

Build the firmware without uploading:

```sh
pio run -e genericSTM32F405RG
```

## Communication recovery

A timeout, transport exception, or malformed/mismatched response invalidates the
client session. Reconnect before making further requests, then read all parameters.
The protocol has no transaction IDs, so a late reply to the same register cannot
be distinguished from a new reply. A disable write is still allowed in an invalid
session, but is reported as unconfirmed rather than treating a stale reply as success.

PID groups remain separate register writes. All values are validated before the
first write, but a communication failure can still leave a partial update. Such
failures are reported explicitly; Apply All requires a fresh successful Read All
after a failed apply, failed refresh, or reconnect. No automatic rollback is attempted
over a connection whose state is uncertain.

Relative position commands read current feedback directly before calculating the
target; paused plotting no longer determines the command's starting angle.
