import numpy as np
import matplotlib.pyplot as plt

# 定义输入范围
x = np.linspace(-5, 5, 1000)

# 定义 sign 函数
def sign(x):
    return np.where(x > 0, 1, np.where(x < 0, -1, 0))

# 定义多个 tanh(kx) 近似函数
k_values = [1,10,50]
tanh_curves = [np.tanh(k * x) for k in k_values]

# 画图
plt.figure(figsize=(10, 6))
plt.plot(x, sign(x), label="sign(x)", color='black', linestyle='--', linewidth=2)

# 画出 tanh(kx) 曲线
for k, curve in zip(k_values, tanh_curves):
    plt.plot(x, curve, label=f"tanh({k}x)")

# 图例与标签
plt.title("Tanh Approximation to Sign Function")
plt.xlabel("x")
plt.ylabel("Output")
plt.grid(True)
plt.legend()
plt.axhline(0, color='gray', linestyle=':')
plt.axvline(0, color='gray', linestyle=':')
plt.tight_layout()
plt.show()
