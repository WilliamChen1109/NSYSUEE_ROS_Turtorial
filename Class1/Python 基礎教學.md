## 1. 什麼是 Python？
Python 是一種：
- 高階語言
- 語法簡單、可讀性高
- 常用於資料分析、AI、網頁、自動化
---
## 2. 第一個 Python 程式
```python
print("Hello, Python!")
```
說明：
- `print()` 用來輸出文字
- 字串要用 `""` 或 `''` 包起來
---
## 3. 變數（Variable）
### 宣告變數
```python
x = 10
name = "Ryan"
```
特點：
- 不需要指定型別
- `=` 是指定，不是等號比較
---
## 4. 基本資料型態

```python
a = 10        # int
b = 3.14      # float
c = "Hello"   # str
d = True      # bool
```
---
## 5. 基本運算
```python
a = 10
b = 3

print(a + b)   # 加法
print(a - b)   # 減法
print(a * b)   # 乘法
print(a / b)   # 除法
print(a % b)   # 餘數
```
---
## 6. 輸入與輸出
### 使用者輸入
```python
name = input("請輸入名字：")
print("你好", name)
```
注意：
- `input()` 讀入的是 **字串**
---
## 7. 型別轉換
```python
age = int(input("請輸入年齡："))
print(age + 1)
```
常見轉換：
```python
int()
float()
str()
```
---
## 8. 比較運算子
```python
a = 10
b = 5
a == b   # 等於
a != b   # 不等於
a > b
a < b
a >= b
a <= b
```
---
## 9. if 條件判斷
### 基本 if
```python
age = 18

if age >= 18:
    print("可以投票")
```
注意：
- Python **用縮排（空白）表示區塊**
- 通常使用 4 個空白
---
## 10. if / else
```python
age = 16

if age >= 18:
    print("成年人")
else:
    print("未成年人")

```
---
## 11. if / elif / else
```python
score = 85

if score >= 90:
    print("A")
elif score >= 80:
    print("B")
else:
    print("C")
```
---
## 12. while
```python
count = 0

while count < 10:
    print(count)
    count = count + 1
```
基本上跟if是一樣的概念，但當while後面的條件成立時，裡面的code會一直不斷執行

---
## 13. 常見新手錯誤
❌ 忘記縮排  
❌ 把 `=` 寫成 `==` 或反過來  
❌ 忘記 `:`  
