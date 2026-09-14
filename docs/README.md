# Documentation

| Document | Contents | Status |
| --- | --- | --- |
| [Development setup](development-setup.md) | Required hardware, vendor packages, IDE, flashing, and repository policy | Available |
| [MIT control](control/mit-control.md) | Control chain, equations, limits, and test procedure | Available |
| [CAN protocol](protocol/can.md) | Commands, ACK, heartbeat, and units | Available |
| [UART protocol](protocol/uart.md) | Text-command format and examples | Available |
| [Assembly guide](assembly/README.md) | Mechanical/electrical assembly and checklists | To be completed |
| [Calibration and tuning](tuning/README.md) | Power-up, encoder, current loop, and MIT parameters | Initial outline |

The `docs/` tree contains human-readable engineering documentation. It is intentionally separate from `tools/`, which contains executable utilities, debugger support files, and tool-specific setup material. Store project photographs, renderings, and videos centrally in [`results/media/`](../results/media/); store raw logs, measurement tables, and reproducible experiment outputs in [`results/experiments/`](../results/experiments/).
