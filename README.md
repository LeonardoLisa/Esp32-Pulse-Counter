# Esp32-Pulse-Counter
Use Esp32 timer (set to max speed 40MHz, pre-scaler 2) to count between the rising edge of start signal and rising edge of stop signal. Rising edges are detected using GPIO interrupt.

### Specs
* Min. time delta (stop - start) ~ 14μs
* Resolution ~ 50ns
