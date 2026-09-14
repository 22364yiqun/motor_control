# Host Tools

The current Python utilities remain in the repository root to preserve existing invocation paths:

- `can_trans.py`: sends MIT commands through a Damiao USB-CAN adapter, waits for ACKs, and listens for frames;
- `can_code_control_test.py`: sends a sequence of output-shaft angles;
- `vla_can_control_example.py`: demonstrates how to connect VLA output to the CAN target.

As the toolset grows, utilities may move into this directory while retaining backward-compatible entry points.
