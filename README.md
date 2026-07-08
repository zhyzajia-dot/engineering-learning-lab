# Engineering Learning Lab

这个仓库用来统一管理学习和工程代码，方便回退版本、同步到 GitHub、和朋友协作。

## 目录

- `esp32-radar/`: ESP32-S3 雷达生命体征固件，包含当前微信小程序/自有服务器版、涂鸦版、旧 MQTT + InfluxDB 版。
- `k230-pid-gimbal/`: K230 二维云台视觉跟踪和 PID 自动调参工程，包含 PC 上位机。
- `mspm0-pid-car/pid-lab/`: MSPM0G3507 PID 实验工程，包含上位机和 ESP8266 无线串口桥。
- `mspm0-pid-car/working-car/`: MSPM0G3507 小车可工作 baseline 工程。

## 第一次下载

朋友第一次使用时执行：

```powershell
cd F:\
git clone https://github.com/zhyzajia-dot/engineering-learning-lab.git
cd F:\engineering-learning-lab
```

你现在本地已经有：

```powershell
F:\engineering-learning-lab
```

以后主要改这个目录里的代码。

## 每天怎么用 Git

开始写代码前先同步：

```powershell
cd F:\engineering-learning-lab
git pull
```

查看当前改了什么：

```powershell
git status
```

改完后提交：

```powershell
git add .
git commit -m "说明这次改了什么"
git push
```

提交信息例子：

```powershell
git commit -m "fix esp32 radar upload interval"
git commit -m "add k230 pid tuning notes"
git commit -m "update mspm0 square tracking parameters"
```

## 怎么回退

查看历史版本：

```powershell
git log --oneline
```

如果某个文件改坏了，还没有提交：

```powershell
git restore 文件名
```

如果已经提交了，想安全撤销某次提交：

```powershell
git revert 提交ID
```

不熟悉时优先用 `git revert`，它比较安全。

## 多人协作规则

1. 写代码前先 `git pull`。
2. 一次提交只做一类修改，不要把 ESP32、K230、MSPM0 无关改动混在一起。
3. 提交信息写清楚，不要只写 `update`。
4. 如果 `git push` 失败，先执行 `git pull`，解决冲突后再 push。
5. 编译生成目录、日志、压缩包不要上传。

## ESP32 快速使用

当前主要开发目录：

```powershell
F:\engineering-learning-lab\esp32-radar\current-server-wechat
```

编译自有服务器 + 微信小程序版：

```powershell
cd F:\engineering-learning-lab\esp32-radar\current-server-wechat
platformio run -e freenove_esp32_s3_wroom
```

编译涂鸦版：

```powershell
platformio run -e tuya_freenove_esp32_s3_wroom
```

编译旧 MQTT + InfluxDB 版：

```powershell
platformio run -e legacy_freenove_esp32_s3_wroom
```

烧录时在对应环境后加 `-t upload`：

```powershell
platformio run -e freenove_esp32_s3_wroom -t upload
```

## K230 快速使用

主程序：

```text
k230-pid-gimbal/auto_pid_tracker.py
```

PC 上位机：

```powershell
cd F:\engineering-learning-lab\k230-pid-gimbal\host
.\start_gui.bat
```

## MSPM0 快速使用

PID 实验工程：

```text
mspm0-pid-car/pid-lab/
```

工作小车 baseline：

```text
mspm0-pid-car/working-car/
```

PID 上位机：

```powershell
cd F:\engineering-learning-lab\mspm0-pid-car\pid-lab\HOST
.\start_autotune.bat
```

## 不要上传的东西

这些已经在 `.gitignore` 中排除了：

```text
.pio/
.platformio-home/
Debug/
build/
logs/
__pycache__/
*.zip
*.rar
*.bin
*.elf
*.map
*.o
*.d
```

如果发现 `git status` 里出现这些文件，先不要提交，应该检查 `.gitignore`。
