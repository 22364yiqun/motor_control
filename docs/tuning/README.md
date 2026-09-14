# Calibration and Tuning

## Recommended Sequence

1. With power removed, check phase wiring, supply rails, and sensor power.
2. Apply power through a current-limited supply without enabling the power stage.
3. Calibrate the three phase-current offsets and confirm sensor polarity.
4. Verify complementary PWM, dead time, and the absence of shoot-through.
5. Verify encoder continuity, direction, air gap, and zero position.
6. At low voltage/current, confirm pole pairs, phase order, and electrical-angle alignment.
7. Tune the current-loop PI controller from conservative limits.
8. Tune the MIT outer loop from low `Kp`, low `Kd`, and zero feed-forward torque.
9. After adding the second encoder, verify the reduction ratio, backlash, and output zero.
10. Increase load gradually while recording temperature, current, noise, and tracking error.

Before the first run, provide a current-limited supply and emergency stop; verify `APP_ALLOW_OPEN_LOOP_RUN`, pole pairs, phase order, encoder direction, current-sense polarity, reduction ratio, `Kt`, and every safety limit. Also confirm that internal-flash sector 31 does not overlap the program image.

> [!CAUTION]
> Multi-turn position is currently accumulated from the motor-side single-turn encoder and stored in flash. Movement while power is off can make the restored position unreliable. When the second output encoder is integrated, add an independent consistency check and homing/recovery strategy.
