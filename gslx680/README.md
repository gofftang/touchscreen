# GSLX680
## 硬件接法
IOVCC；I2C加上拉；INT和SHUTDOWN不要加上拉（加上有200uA+漏电）。

## 软件
此系列IC没有内部flash，需要上电后通过I2C写到内部memory中。IC没有硬件防抖(比较那个啥。。。)，软件防抖宏CONFIG_TOUCHSCREEN_GSLX680_ANTI_SHAKE。
降低功耗：官方说sleep电流<30uA，实测200uA左右（开发说正常是这样。。。），最好关闭IOVCC，SHUTDOWN和INT输出低。
