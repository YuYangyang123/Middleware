# 探测数据转换中间件（最终精简版）

本版本用于实验室在线部署：

```text
上游仿真系统 TCP JSON → 输入解析 TargetState → 雷达/光电/无线电模型 → 输出协议编码 → TCP/UDP 下游转发
```

代码只依赖 Python 标准库，建议 Python 3.10+。

## 1. 目录说明

```text
middleware_final_project/
├── config/default_config.json        # 设备指标、监听端口、下游转发地址
├── examples/sample_input.json        # 本地自检用样例输入
├── middleware/
│   ├── app.py                        # 主入口：在线监听 / 离线自检
│   ├── config.py                     # 配置读取
│   ├── domain.py                     # TargetState / DetectionReport / OutputPacket
│   ├── parser.py                     # JSON 校验、单位转换、WGS84→ENU
│   ├── models.py                     # 雷达、光电、无线电模型和状态机
│   ├── network.py                    # TCP JSON 实时监听
│   ├── output.py                     # 雷达/光电/无线电输出编码和网络转发
│   └── utils.py                      # 坐标、CRC、时间、ID 等工具
└── tools/send_sample_frame.py         # 本地模拟上游按 1Hz 发送 JSON
```

## 2. 在线运行

在工程根目录执行：

```bash
python -m middleware.app --config config/default_config.json
```

默认监听：

```text
0.0.0.0:1102
```

上游仿真系统通过 TCP 连接到该地址，推荐发送格式为“一行一个 JSON”：

```text
{"header": {...}, "body": {...}}\n
```

服务端也兼容多个 JSON 对象连续发送的 TCP 流。若后续上游改成“长度头 + JSON”的二进制帧，只需要改 `middleware/network.py` 的分帧逻辑，后面的模型和输出端不用改。

## 3. 下游转发配置

所有下游地址都在 `config/default_config.json` 的 `network.outputs` 中配置：

```json
"radar_track_upload_1001.bin": {
  "enabled": true,
  "protocol": "tcp",
  "host": "127.0.0.1",
  "port": 1001
}
```

当前默认严格对应设计文档的 8 类输出：

| 输出报文 | 默认协议 | 默认端口 | 说明 |
|---|---:|---:|---|
| `radar_track_upload_1001.bin` | TCP | 1001 | 雷达点迹数据上传 |
| `radar_status_upload_1002.bin` | TCP | 1002 | 雷达状态数据上传 |
| `eo_ivpReport_1004.udp` | UDP | 1004 | 光电探测数据回传 ivpReport |
| `eo_PTZStatusReport_1005.udp` | UDP | 1005 | 光电状态数据回传 PTZStatusReport |
| `radio_device_1010.bin` | TCP | 1010 | 无线电侦测设备数据 |
| `radio_target_1011.bin` | TCP | 1011 | 无线电侦测目标数据 |
| `message_device_1012.json` | TCP | 1012 | 报文解析设备数据，默认关闭 |
| `message_parse_1013.json` | TCP | 1013 | 报文解析数据，默认关闭 |

调试时可设置：

```json
"mirror_output_dir": "out_mirror"
```

这样每次转发时会额外把报文写到本地目录，方便核对二进制/JSON 内容。

## 4. 离线自检

不连接真实上下游时，先跑：

```bash
python -m middleware.app --config config/default_config.json --demo examples/sample_input.json --out-dir out
```

成功后会生成：

```text
out/radar_track_upload_1001.bin
out/radar_status_upload_1002.bin
out/eo_ivpReport_1004.udp
out/eo_PTZStatusReport_1005.udp
out/radio_device_1010.bin
out/radio_target_1011.bin
out/message_device_1012.json
out/message_parse_1013.json
```

## 5. 本地联调

开两个终端。

终端 1：启动中间件监听。

```bash
python -m middleware.app --config config/default_config.json
```

终端 2：模拟上游仿真系统每秒发一帧。

```bash
python tools/send_sample_frame.py --host 127.0.0.1 --port 1102 --count 6
```

如果本机没有启动下游 TCP 服务，日志里可能出现 `send failed`，这是正常的；中间件不会退出。接入真实下游或把对应输出 `enabled` 改成 `false` 即可。

## 6. 8 类输出对应关系

本工程每处理一帧上游 JSON，都会构造 8 类输出包；是否真正向下游发送由 `config/default_config.json/network.outputs` 里的 `enabled` 控制。

```text
1001 radar_track_upload_1001.bin      雷达点迹数据上传
1002 radar_status_upload_1002.bin     雷达状态数据上传
1004 eo_ivpReport_1004.udp            光电探测数据回传
1005 eo_PTZStatusReport_1005.udp      光电状态数据回传
1010 radio_device_1010.bin            无线电侦测设备数据
1011 radio_target_1011.bin            无线电侦测目标数据
1012 message_device_1012.json         报文设备数据
1013 message_parse_1013.json          报文解析数据
```

说明：设计文档对 1001、1004、1011、1012、1013 的字段描述更细；对 1002 雷达状态和 1010 无线电设备数据只明确了输出类别与用途。因此代码里先实现最小可运行状态帧，字段集中在设备 ID、时间戳、工作状态、站点经纬高等，后续拿到厂家完整接口表时只需要替换 `middleware/output.py` 里的两个 builder。

## 7. 模型指标对应

### 雷达

默认参数在 `config.default_config.json/radar`：

- 距离：50 m ~ 3000 m
- 方位：0° ~ 360°
- 俯仰：-5° ~ 40°
- 最低可探测速度：2 m/s
- 探测概率：Pd=0.95，支持随距离衰减
- 误差：距离 5 m，方位 0.4°，俯仰 0.4°
- RCS：0.01 m²
- 连续 3 帧确认，连续 5 帧丢失

### 光电

默认参数在 `config.default_config.json/electro_optical`：

- 距离：40 m ~ 2000 m
- 云台水平：360°
- 俯仰：-40° ~ 85°
- FOV：2° ~ 60°，默认 60°
- 置信度阈值：0.6
- 输出脱靶量 offsetH/offsetV，单位 mrad
- 图像坐标：640×512
- 连续 3 帧确认，连续 8 帧丢失

### 无线电

默认参数在 `config.default_config.json/radio`：

- 探测距离：3 km
- 俯仰：-5° ~ 55°
- 频段：300 MHz ~ 6000 MHz
- 并发：30 架
- 首次截获延迟：3 s
- 测向误差：≤3°
- 置信度：0~100，默认阈值 60
- 连续 3 帧确认，连续 8 帧丢失

## 8. 输入 JSON 字段

核心字段仍沿用设计文档：

```json
{
  "header": {
    "MessageId": "AA123456789",
    "MessageType": "MannedAircraft",
    "Timestamp": 1671354231500,
    "Source": "MAV001",
    "Version": "1.0"
  },
  "body": {
    "FlightID": "FL12345",
    "Callsign": "ABC123",
    "Reg": "N12345",
    "ACCode": "7500",
    "Latitude": 31.125456,
    "Longitude": 120.128456,
    "Altitude": 1200.0,
    "Heading": 45.0,
    "Pitch": 0.0,
    "Roll": 0.0,
    "GS": 180.0,
    "TAS": 100.0,
    "NextPoint": "ZLYC",
    "FrequencyMHz": 2400.0
  }
}
```

`FrequencyMHz` 是给无线电模型预留的扩展字段；如果没有该字段，默认使用配置里的 `default_target_frequency_mhz=2400.0`。
