## 1. Interactive Marker 概念介紹
**Interactive Marker** 是 ROS 中由 **RViz** 所支援的一種互動式控制工具，允許使用者直接在 3D 視覺化介面中，透過滑鼠拖曳標記（Marker）來控制機器人的移動與旋轉。

在 TurtleBot3 的範例中，Interactive Marker 會被用來產生速度控制指令，並透過 ROS Topic 發佈至機器人的控制節點，使機器人即時依照使用者在 RViz 中的操作進行運動，而不需要額外撰寫鍵盤控制或手動輸入指令。

---
## 2. 系統架構與運作流程
使用 Interactive Marker 控制 TurtleBot3 時，整體 ROS 系統的運作流程如下：
1. RViz 顯示 Interactive Marker  
2. 使用者拖曳或旋轉 Marker  
3. Interactive Marker 節點處理輸入  
4. 發佈 `/cmd_vel` 速度指令  
5. TurtleBot3 接收並執行移動  
### 主要 ROS 元件說明
| 元件                            | 說明                       |
| ----------------------------- | ------------------------ |
| turtlebot3_interactive_marker | 建立並管理 Interactive Marker |
| turtlebot3_node               | 接收 `/cmd_vel` 並控制機器人     |
| diff_drive_controller         | 差速驅動控制                   |
| robot_state_publisher         | 提供機器人模型給 RViz            |

---
## 3. 啟動 TurtleBot3 系統
### 3.1 連線至 TurtleBot3
```bash
ssh ubuntu@<TURTLEBOT3_IP>
```
### 3.2 設定 TurtleBot3 機型
```bash
export TURTLEBOT3_MODEL=burger
```
### 3.3 啟動 Bringup
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
---
## 4. 啟動 Interactive Marker 範例節點
```bash
ros2 run turtlebot3_example turtlebot3_interactive_marker
```
---
## 5. RViz 設定
```bash
rviz2
```
新增：
- RobotModel（/robot_description）
- InteractiveMarkers（By Topic）
---
## 6. 操作方式
### 線性移動
拖曳箭頭控制前進與後退（linear.x）
### 旋轉控制
拖曳圓形控制環進行旋轉（angular.z）

---
## 7. Topic 觀察
```bash
ros2 topic echo /cmd_vel
```

---
## 8. 注意事項
- 請在安全環境操作
- 若無反應請確認 `/cmd_vel` 是否被佔用
- Marker 未顯示請確認節點是否啟動
---
## 9. 小結

Interactive Marker 提供直觀的 RViz 控制方式，有助於理解 ROS 控制流程與機器人運動行為。