import matplotlib.pyplot as plt
import pandas as pd

# 读取数据（假设数据存储在 'data.txt' 文件中）
data = pd.read_csv('output.txt', header=None, names=['Index', 'Value'])

# 绘制图形
plt.plot(data['Index'], data['Value'], label='Data Points')

# 添加标题和标签
plt.title('Plot of Values vs Index')
plt.xlabel('Index')
plt.ylabel('Value')

# 显示图例
plt.legend()

# 显示图形
plt.show()
