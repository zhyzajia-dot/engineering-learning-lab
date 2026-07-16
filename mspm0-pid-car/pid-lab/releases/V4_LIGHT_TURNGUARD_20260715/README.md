# V4 LIGHT / V4-TurnGuard 冻结归档

这是不带重云台时已经验证可用的轻载 V4 归档，不包含 GIMBAL 参数实验。

完整冻结工程位于相邻文件 `V4_LIGHT_TURNGUARD_PROJECT_20260715.zip`：

- 大小：201,581 B
- SHA-256：`33F8D90520A3CA89A686D871E32120E8A1BCFE5A19B1CA64F1F6C1DF83D12BAD`
- 内容：MSPM0 源码、上位机、ESP8266 桥源码、构建脚本、两个回退 HEX 和校验清单。

## 推荐回退固件

- `pid_lab_mspm0_v4_turnguard.hex`
- 大小：87,405 B
- SHA-256：`E8EA86F3A9CD9BA089644B90D8C7E6922221B08E38C6136FD7068A4648A36A93`
- 控制：V4 直线循迹原样，只增加最小 50° TurnGuard。
- 实车证据：轻载 300 mm/s 完成方框；普通直线循迹 PD 未改变。

## 精确 V4 回退固件

- `pid_lab_mspm0_v4_exact.hex`
- 大小：87,158 B
- SHA-256：`39AB0AF11D68C6541B7C159AD4590EF7AFE632C9B79B4F9DA43B3ED5CF7B5AA1`
- 控制：截图版本的精确 V4，不包含 50° TurnGuard。
- 实车证据：轻载 300 / 340 / 380 mm/s 各完成过方框。

## 可复现源码构建

完整源码保留在本工程中。`LAB_ENABLE_DUAL_PROFILE=0` 会排除后续的
LIGHT/GIMBAL Flash V3 基础设施，恢复原 V4-TurnGuard 的 Flash V2、命令和
PARAM 格式。执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_v4_light_hex.ps1
```

脚本只有在生成文件的 SHA-256 与上述 V4-TurnGuard 完全一致时才成功。
当前上位机 `restore-v4` 同时兼容该归档的无档位协议和当前 Flash V3 协议。

烧录旧 V4 后仍要回读参数，不能假设 HEX 会覆盖芯片里已有的 Flash 参数：

```powershell
python .\HOST\pid_lab_cli.py restore-v4 --port COM12
```

归档文件的唯一身份依据是 `SHA256SUMS.txt`，不要根据文件名猜版本。
