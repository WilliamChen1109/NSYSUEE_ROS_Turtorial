## 一、為什麼機器人需要 Map？
在自主移動機器人（Autonomous Mobile Robot, AMR）中，**Map（地圖）是機器人理解環境的核心資料結構**，用來回答三個基本問題：
1. **我在哪裡？（Localization）**
2. **環境長怎樣？（Mapping）**
3. **我要怎麼走到目的地？（Planning）**

在 ROS 2 的 Navigation2（Nav2）架構中，為了同時兼顧「全局路徑規劃」與「即時避障」，地圖被劃分為 **Global Map** 與 **Local Map** 兩個層級。

---
## 二、Global Map（全域地圖）
### 1. 定義與用途
**Global Map** 是一張**靜態或低頻更新**的地圖，描述整個環境的結構，例如：
- 牆壁
- 固定障礙物
- 房間、走廊配置
主要用途：
- **全域路徑規劃（Global Planning）**
- 提供機器人對「整體環境」的認知

在 ROS 2 中，Global Map 通常由 **SLAM 或事先建圖後載入**。

---
### 2. 地圖表示方式（Occupancy Grid）
Global Map 通常使用 **Occupancy Grid Map（佔據格地圖）**：
- 將空間離散化成固定大小的格子（cell）
- 每個 cell 表示佔據狀態：
  - `0`：自由空間（Free）
  - `100`：障礙物（Occupied）
  - `-1`：未知區域（Unknown）
地圖解析度：
```
Resolution = 每個 cell 對應的實際距離（例如 0.05 m / cell）
```

---
### 3. 建立方式（SLAM）
Global Map 可透過 SLAM 建立，例如：
- **slam_toolbox（ROS 2 常用）**
- Cartographer

SLAM 的核心概念：
1. 使用 LiDAR / Depth Camera 感知環境
2. 透過里程計（Odometry）預測位置
3. 不斷修正地圖與自身位置（閉環修正）

---
### 4. 在 Nav2 中的角色
- 提供給 **Global Costmap**
- 用於 A*、Dijkstra 等 **全域路徑規劃演算法**
- 更新頻率低（甚至完全不更新）
---
## 三、Local Map（區域地圖）
### 1. 定義與用途
**Local Map** 是以機器人為中心、**即時更新的動態地圖**，主要用於：
- 即時避障
- 動態障礙物處理（人、移動物體）
- 局部路徑修正

Local Map 通常是：
- 小範圍（例如 5m × 5m）
- 高更新頻率（10–30 Hz）
---
### 2. Local Map 的特性
| 特性   | Global Map | Local Map        |
| ---- | ---------- | ---------------- |
| 範圍   | 大          | 小                |
| 更新頻率 | 低          | 高                |
| 障礙物  | 靜態         | 動態 + 靜態          |
| 參考座標 | map frame  | odom / base_link |
| 用途   | 路徑規劃       | 即時避障             |

---
### 3. 感測器融合
Local Map 主要來自即時感測器資料：
- LiDAR（LaserScan）
- Depth Camera（PointCloud2）
- 超音波（較少見）
流程概念：
```
Sensor Data → 障礙物投影 → Local Costmap
```
---
### 4. Costmap（代價地圖）
Local Map 在 Nav2 中以 **Local Costmap** 形式存在，除了「是否有障礙」，還引入 **代價（Cost）** 概念：
- 障礙物中心：最高代價
- 障礙物周圍：逐漸遞減（Inflation Layer）
- 自由空間：低代價

這樣可以讓機器人：
- 不只「不撞到」
- 而是「保持安全距離」
---
## 四、Global Map 與 Local Map 的協同運作
### 1. 系統整體流程
```
Global Map
   ↓
Global Planner（A*, Dijkstra）
   ↓
全域路徑
  ↓
Local Map（即時更新）
   ↓
Local Planner（DWB、TEB）
   ↓
控制指令（cmd_vel）
```

---
### 2. 實際行為範例
假設：
- Global Map 中走廊是空的
- 但現場突然出現一個人

系統行為：
1. Global Planner 仍維持原路徑
2. Local Map 偵測到動態障礙物
3. Local Planner 即時繞開人
4. 障礙物消失後回到原路徑

👉 **不需要重建 Global Map**

---
## 五、ROS 2 中的實際對應元件（Nav2）
| 功能             | ROS 2 套件 / 節點     |
| -------------- | ----------------- |
| Global Map     | map_server        |
| SLAM           | slam_toolbox      |
| Global Costmap | global_costmap    |
| Local Costmap  | local_costmap     |
| 全域規劃           | planner_server    |
| 區域規劃           | controller_server |

---
## 六、總結
- **Global Map**  
  - 提供環境全貌  
  - 用於長距離路徑規劃  
  - 偏向「策略層」
- **Local Map**  
  - 即時反映周圍環境  
  - 用於避障與微調  
  - 偏向「反應層」