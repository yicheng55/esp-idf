# ESP32 模組腳位對照表

## DM9051A 介面腳位對照

| Function   | ESP32-S3-WROOM<br>Pin Name/Number | ESP32-WROOM<br>Pin Name/Number | ESP32-WROVER<br>Pin Name/Number | ESP32-C6-WROOM<br>Pin Name/Number | ESP32-C5-WROOM<br>Pin Name/Number |
|------------|:---------------------------------:|:------------------------------:|:-------------------------------:|:---------------------------------:|:---------------------------------:|
| I2C SDA    | IO20                              | IO12                           | IO22                            |                                   |                                   |
| I2C CLK    | IO9                               | IO13                           | IO21                            |                                   |                                   |
| SPI CS     | IO15                              | IO32                           | IO32                            | IO0                               | IO0                               |
| SPI CK     | IO16                              | IO33                           | IO33                            | IO1                               | IO1                               |
| SPI MOSI   | IO17                              | IO25                           | IO25                            | IO8                               | IO6                               |
| SPI MISO   | IO18                              | IO26                           | IO26                            | IO10                              | IO7                               |
| SPI INT    | IO8                               | IO27                           | IO27                            | IO11                              | IO8                               |
| RST        | IO3                               | IO23                           | IO23                            | IO2                               | IO25                              |

> **備註：**
> - ESP32-C6-WROOM 欄位以紅框標示（原圖）
> - I2C SDA / I2C CLK 欄位：ESP32-C6-WROOM 與 ESP32-C5-WROOM 無資料
> - 所有腳位編號皆以 `IO` 前綴表示 GPIO 編號
