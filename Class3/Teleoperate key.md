## 一、系統架構與操作目標說明
TurtleBot3 的基本操作採用 **Client–Server 架構**：
- **TurtleBot3 端（SBC）**  
  負責啟動機器人本體的驅動程式、感測器與運動控制節點。
- **電腦端（Remote PC）**  
  負責透過鍵盤發送控制指令，並監控機器人狀態。

在進行任何控制之前，**必須先在 TurtleBot3 端完成 Bringup**，確保 ROS 節點與通訊正常運作。

---
## 二、TurtleBot3 端（機器人本體）操作
### 1. 啟動 TurtleBot3 Bringup
在 TurtleBot3 的單板電腦（SBC）終端機中執行以下指令：
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```
此步驟會完成以下工作：
- 啟動馬達控制與 OpenCR 通訊
- 啟動 LiDAR 感測器
- 發佈機器人狀態相關的 ROS topics（如 `/odom`、`/scan`）
- 等待外部裝置（電腦端）送出移動指令 `/cmd_vel`

Bringup 成功後，TurtleBot3 即可接收來自電腦端的控制命令。

---
## 三、電腦端（Remote PC）操作
### 1. 鍵盤遙控 TurtleBot3
在已安裝 ROS / ROS2 並完成網路設定的電腦端，開啟終端機並執行：
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```
啟動後，終端機會顯示鍵盤操作說明，可透過以下按鍵控制 TurtleBot3 移動：

| 按鍵            | 功能   |
| ------------- | ---- |
| `w`           | 向前移動 |
| `x`           | 向後移動 |
| `a`           | 左轉   |
| `d`           | 右轉   |
| `s` / `space` | 立即停止 |
| `Ctrl + C`    | 結束程式 |

此程式會持續向 ROS topic `/cmd_vel` 發送速度指令，使 TurtleBot3 依照鍵盤輸入進行移動。

---
## 四、操作流程總結
完整的鍵盤操作流程如下：
1. **在 TurtleBot3 端**
   - 設定機器人型號
   - 啟動 `turtlebot3_bringup`

2. **在電腦端**
   - 啟動鍵盤遙控程式 `teleop_keyboard`
   - 使用鍵盤控制 TurtleBot3 前進、後退與轉向

3. **ROS 通訊**
   - 電腦端透過 `/cmd_vel` 發送控制指令
   - TurtleBot3 回傳感測器與狀態資訊供系統使用
---
## 五、注意事項
- 務必確認 **Bringup 已成功執行**，否則鍵盤指令將無法控制機器人。
- 操作時請確保機器人周圍環境安全，避免撞擊或跌落。
- 建議在低速下進行初次測試，以降低風險。