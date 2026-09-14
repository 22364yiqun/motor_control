# Project Media

All photographs, CAD renderings, overview graphics, and demonstration videos are collected here so that static project assets do not become scattered across documentation or experiment directories.

## Images

### Project overview

![OpenEAI-FOC overview](images/openeai-foc-overview.png)

### Physical prototype

![OpenEAI-FOC actuator prototype](images/actuator-prototype.jpg)

### Exploded assembly

![OpenEAI-FOC actuator exploded assembly](images/actuator-exploded-view.png)

Additional CAD exports:

- [Actuator appearance](images/4310-appearance.png)
- [Exploded view](images/4310-exploded-view.png)
- [Exploded view, revision 2](images/4310-exploded-view-v2.png)

## Demonstration Videos

| Demonstration | Video | Interpretation |
| --- | --- | --- |
| MIT control mode | [Watch video](videos/mit-mode-demo.mp4) | General MIT-mode operation on the prototype |
| MIT position command | [Watch video](videos/mit-position-command.mp4) | Position-target command response |
| Velocity command | [Watch video](videos/velocity-command-1.2-rad-s.mp4) | Demonstration with a 1.2 rad/s velocity target |
| Torque command | [Watch video](videos/torque-command-5-ncm.mp4) | Demonstration with a 5 N·cm torque target |
| Fault protection | [Watch video](videos/fault-protection-demo.mp4) | Demonstration of the current protection behavior |

The filenames record the commanded setpoints shown by the supplied demonstrations. They are not a substitute for calibrated measurements. Future quantitative results will be published under [`../results/experiments/`](../results/experiments/) with test conditions and raw data.

## Contribution Rules

- Use short, descriptive English filenames.
- Put images in `images/` and videos in `videos/`.
- Record the hardware revision, firmware commit, date, supply voltage/current limit, motor/load, and test purpose in the accompanying documentation.
- Remove private information, serial numbers, and unrelated background content before publication.
- Use Git LFS for binary media.
