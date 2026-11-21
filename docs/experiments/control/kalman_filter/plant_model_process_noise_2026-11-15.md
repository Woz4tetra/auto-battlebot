# Plant model process noise and Jacobian notes

## Process noise mapping

Growth-law mechanisms from fit_process_noise.py, mapped per predict step. The white-noise acceleration PSDs use the standard discrete block (position picks up the h^3/3 growth through the Jacobian as the EKF accumulates F P F^T + Q). The scale-factor and delay-jitter mechanisms grow h^2 and h^0, which additive per-step Q cannot reproduce, so they are injected at a rate matched at the 400 ms design horizon: exact at a full coast, conservative below it. q = 3c assumes undamped velocity-error integration; the plant's lag damps it, so the h^3 terms come out somewhat narrow over long coasts. Cross-track has no velocity state, so its whole budget is horizon-matched, and heading noise also reaches cross-track through the Jacobian: a known double count in the wide direction.
