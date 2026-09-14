# UART Debugging

The firmware uses USART1 with these settings:

```text
Baud rate:   115200
Data bits:   8
Parity:      none
Stop bits:   1
Flow control:none
Line ending: CR, LF, or CRLF
```

ATK-XCOM can be used on Windows. Select the USB-UART adapter's COM port, choose `115200 8N1`, enable “send new line,” and send commands such as `m0`, `m10`, or `m90,0,5`. The firmware accepts integer fields only.

ATK-XCOM is third-party software from ALIENTEK. Its executable is not mirrored in this repository because an explicit redistribution license could not be verified. Obtain it through [ALIENTEK's official documentation/support channels](https://wiki.alientek.com/docs/Boards/STM32/DNN647/basic-examples/serial/). Any equivalent terminal that can send CR/LF text at 115200 8N1 will work.

No special UART application is required. The USB-UART bridge may require its own vendor driver, depending on whether the board uses CH340, CP210x, FTDI, or another bridge IC. Document the actual bridge part in the electronics BOM before linking a driver.
