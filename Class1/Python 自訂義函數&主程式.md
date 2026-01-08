## 一、自訂義函數（Function）
### 1. 什麼是函數？
函數是一段**可以重複使用的程式碼**，用來：
- 減少重複程式
- 讓程式更清楚、好維護
---
### 2. 基本函數寫法
```python
def 函數名稱():
    程式內容
```
範例：
```python
def hello():
    print("Hello, Python!")
```
---
### 3. 呼叫函數
```python
hello()
```
輸出：
```text
Hello, Python!
```
---
### 4. 有參數的函數
```python
def greet(name):
    print("Hello", name)
```
呼叫方式：
```python
greet("Ryan")
```
---
### 5. 有回傳值的函數（return）
```python
def add(a, b):
    return a + b
```
使用方式：
```python
result = add(3, 5)
print(result)
```
---
## 二、`if __name__ == "__main__"` 說明
### 1. `__name__` 是什麼？
- `__name__` 是 Python 內建變數
- 用來表示「目前程式檔案的身分」

| 執行方式     | `__name__` 的值 |
| -------- | ------------- |
| 直接執行     | `"__main__"`  |
| 被 import | 檔案名稱          |

---
### 2. 基本寫法
```python
if __name__ == "__main__":
    main()
```
---
### 3. 為什麼要使用？

防止程式在被其他檔案 `import` 時自動執行

---
### 4. 完整範例
```python
def add(a, b):
    return a + b
 
def main():
    x = 3
    y = 5
    print(add(x, y))

if __name__ == "__main__":
    main()
```
說明：
- 直接執行此檔案 → `main()` 會執行
- 被其他檔案 `import` → 只匯入函數，不執行 `main()`
---
## 三、重點整理
- `def` 用來定義函數
- `return` 用來回傳結果
- `main()` 常作為程式進入點
- `if __name__ == "__main__"` 是 Python 的標準寫法
---
## 四、總結
> **函數是工具，main 是入口**

```python
if __name__ == "__main__":
```
用來決定「程式要不要現在執行」