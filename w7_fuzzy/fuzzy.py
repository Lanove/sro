#%%
import numpy as np
import matplotlib.pyplot as plt

#%%
def triangular_membership(x, a, b, c, d, e):
    if x < a:
        return d
    elif a <= x <= b:
        return (x - a) / (b - a)
    elif b <= x <= c:
        return (c - x) / (c - b)
    else:  # x > c
        return e

def temperature_membership(temp):
  dingin = triangular_membership(temp, 15, 15, 25, 1, 0)
  sedang = triangular_membership(temp, 15, 25, 35, 0 ,0)
  panas = triangular_membership(temp, 25, 35, 35, 0 , 1)
  
  return dingin, sedang, panas

temp_eval_scalar = 32.5
dingin, sedang, panas = temperature_membership(temp_eval_scalar)
print(f"Pada temperatur {temp_eval_scalar}°C, nilai keanggotaan dingin: {dingin}, sedang: {sedang}, panas: {panas}")

temp_eval = np.linspace(10, 40, 1000)

dingin_vals = []
sedang_vals = []
panas_vals = []

for temp in temp_eval:
    dingin, sedang, panas = temperature_membership(temp)
    dingin_vals.append(dingin)
    sedang_vals.append(sedang)
    panas_vals.append(panas)

plt.figure(figsize=(10, 6))
plt.plot(temp_eval, dingin_vals, 'b-', label='Dingin')
plt.plot(temp_eval, sedang_vals, 'g-', label='Sedang')
plt.plot(temp_eval, panas_vals, 'r-', label='Panas')

plt.title('Temperature Membership Functions')
plt.xlabel('Temperature (°C)')
plt.ylabel('Membership Value')
plt.grid(True)
plt.legend()
plt.xlim([10, 40])
plt.ylim([0, 1.1])

plt.fill_between(temp_eval, dingin_vals, alpha=0.2, color='blue')
plt.fill_between(temp_eval, sedang_vals, alpha=0.2, color='green')
plt.fill_between(temp_eval, panas_vals, alpha=0.2, color='red')

plt.show()

# %%
