# 上位机工具

当前 Python 工具位于仓库根目录，以保持已有调用方式不变：

- `can_trans.py`：达妙 USB-CAN 命令发送、ACK 等待和帧监听；
- `can_code_control_test.py`：输出轴角度序列测试；
- `vla_can_control_example.py`：VLA 输出接入 CAN 目标的示例。

后续工具增多时可迁移到本目录，并保留向后兼容入口。
