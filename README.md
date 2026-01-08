## 課程目標  
1. 操作 Ubuntu 與 Terminal 進行基本系統管理
2. 閱讀並修改基礎 Python ROS 2 程式
3. 理解 ROS 2 中 node 與 topic 的實際用途
4. 操作 TurtleBot3 進行手動與自動移動
5. 使用光達資料完成基本避障行為
6. 進行 SLAM 建圖並儲存地圖
7. 使用 Navigation2 套件完成自主導航
8. 整合感測、定位與導航，完成行動機器人自動駕駛實作

## 課程進度與內容規劃（共 8 週）

### 第 1 週：Ubuntu安裝、Python 與 ROS 2 基礎操作
- Ubuntu 基本操作與 Terminal 指令
- Python 基礎語法（僅限閱讀與簡易修改）
- ROS 2 基本概念（node、topic 直覺式說明）
- turtlesim 環境測試  
**實作：**
- ROS Hello world

---

### 第 2 週：ROS 2 通訊與機器人控制基礎
- Publisher / Subscriber 概念
- `cmd_vel` 與速度控制
- ROS 2 CLI 操作  
**實作：**
- 使用 topic 指令控制 turtlesim 移動

---

### 第 3 週：TurtleBot3 系統安裝與環境建置
- TurtleBot3 系統架構介紹
- ROS 2 與 TurtleBot3 套件安裝
- 網路與裝置設定（實機或模擬）
- 啟動測試與常見問題排除  
**實作：**
- 成功啟動 TurtleBot3 並確認感測資料正常

---

### 第 4 週：TurtleBot3 手動移動與感測資料觀察
- Teleoperation 操作
- 光達（LaserScan）與里程計（Odometry）資料介紹
- 使用 RViz 觀察感測資料  
**實作：**
- 手動操控 TurtleBot3 並分析感測數據變化

---

### 第 5 週：光達避障實作
- LaserScan 資料結構（ranges）
- 簡易避障邏輯設計
- 範例程式修改與參數調整  
**實作：**
- TurtleBot3 光達自動避障實作

---

### 第 6 週：SLAM 建圖實作
- SLAM 基本概念（系統流程導向）
- 啟動 SLAM Toolbox / Cartographer
- 地圖建構與儲存  
**實作：**
- 操作 TurtleBot3 建立並儲存環境地圖

---

### 第 7 週：Navigation2 自主導航
- Navigation2 系統流程概述
- 定位、路徑規劃與避障整合
- RViz 目標點設定  
**實作：**
- TurtleBot3 自主導航至指定位置
- 基本 costmap 參數調整

---

### 第 8 週：期末實作展示－行動機器人自動駕駛
- 系統整合與啟動流程展示
- 使用光達與地圖完成自動導航
- 成果展示與問題說明  
**評量重點：**
- 系統是否可正常運作
- 自主導航完成度
- 對系統流程的理解程度
