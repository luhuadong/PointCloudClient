# PointCloudClient

PointCloudClient 是一个点云数据输出程序，支持 UDP 和 TCP 两种连接方式，用于模拟 LiDAR 扫描获得的点云数据，方便与点云可视化软件（上位机）配合调试。虽然这个项目叫 Point Cloud Client，但其实站在 TCP/IP 的角度来看，它属于 Server 端。在正常工作模式下，上位机软件需要主动与 PointCloudClient 建立连接，才能获取点云数据。



## 依赖

| 软件库 | 版本   | 功能               |
| ------ | ------ | ------------------ |
| cJSON  | 1.7.15 | 解析 JSON 格式数据 |



## 用法

