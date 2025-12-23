#!/usr/bin/env python3
"""
Візуалізація результатів розв'язання рівняння (1)
методом Рунге-Кутта 4-го порядку
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Читання даних з правильною обробкою line endings
df = pd.read_csv('simulation_results.csv', sep=';', encoding='utf-8', engine='python')

# Видалення можливих пробілів у назвах колонок
df.columns = df.columns.str.strip()

print("Доступні колонки:", df.columns.tolist())
print(f"Перші рядки:\n{df.head()}")

# Створення фігури з підграфіками
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('Розв\'язання рівняння (1) САР авіаційного двигуна\nМетод Рунге-Кутта 4-го порядку',
             fontsize=14, fontweight='bold')

# Графік 1: x(t) - обороти двигуна
axes[0, 0].plot(df['t'], df['x'], 'b-', linewidth=1.5)
axes[0, 0].set_xlabel('Час t (с)', fontsize=10)
axes[0, 0].set_ylabel('x(t) - обороти', fontsize=10)
axes[0, 0].set_title('Кількість обертів двигуна')
axes[0, 0].grid(True, alpha=0.3)

# Графік 2: x'(t) - швидкість зміни обертів
axes[0, 1].plot(df['t'], df['x_d'], 'g-', linewidth=1.5)
axes[0, 1].set_xlabel('Час t (с)', fontsize=10)
axes[0, 1].set_ylabel('x\'(t)', fontsize=10)
axes[0, 1].set_title('Перша похідна (швидкість зміни)')
axes[0, 1].grid(True, alpha=0.3)

# Графік 3: x''(t) - прискорення
axes[1, 0].plot(df['t'], df['x_dd'], 'r-', linewidth=1.5)
axes[1, 0].set_xlabel('Час t (с)', fontsize=10)
axes[1, 0].set_ylabel('x\'\'(t)', fontsize=10)
axes[1, 0].set_title('Друга похідна (прискорення)')
axes[1, 0].grid(True, alpha=0.3)

# Графік 4: x'''(t) - третя похідна
axes[1, 1].plot(df['t'], df['x_ddd'], 'm-', linewidth=1.5)
axes[1, 1].set_xlabel('Час t (с)', fontsize=10)
axes[1, 1].set_ylabel('x\'\'\'(t)', fontsize=10)
axes[1, 1].set_title('Третя похідна')
axes[1, 1].grid(True, alpha=0.3)

# Графік 5: F(t) - збурення
axes[2, 0].plot(df['t'], df['F'], 'k-', linewidth=1.5)
axes[2, 0].set_xlabel('Час t (с)', fontsize=10)
axes[2, 0].set_ylabel('F(t)', fontsize=10)
axes[2, 0].set_title('Зовнішнє збурення F(t)')
axes[2, 0].grid(True, alpha=0.3)

# Графік 6: x(t) та F(t) разом
ax1 = axes[2, 1]
ax2 = ax1.twinx()

line1 = ax1.plot(df['t'], df['x'], 'b-', linewidth=1.5, label='x(t) - обороти')
line2 = ax2.plot(df['t'], df['F'], 'k--', linewidth=1.5, label='F(t) - збурення')

ax1.set_xlabel('Час t (с)', fontsize=10)
ax1.set_ylabel('x(t)', fontsize=10, color='b')
ax2.set_ylabel('F(t)', fontsize=10, color='k')
ax1.tick_params(axis='y', labelcolor='b')
ax2.tick_params(axis='y', labelcolor='k')
axes[2, 1].set_title('Відгук системи на збурення')
ax1.grid(True, alpha=0.3)

# Легенда
lines = line1 + line2
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper right', fontsize=9)

plt.tight_layout()

# Визначаємо шлях для збереження
import os
output_path = 'simulation_results.png'
if not os.access('.', os.W_OK):
    # Якщо немає прав запису в поточній директорії, зберігаємо у /tmp
    output_path = '/tmp/simulation_results.png'

plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"\n✓ Графіки збережено у файл: {output_path}")

# Статистика
print("\n=== СТАТИСТИКА РОЗВ'ЯЗАННЯ ===")
print(f"Кількість точок: {len(df)}")
print(f"Інтервал часу: [{df['t'].min():.2f}, {df['t'].max():.2f}]")
print(f"\nЗначення x(t):")
print(f"  Мінімум: {df['x'].min():.6f}")
print(f"  Максимум: {df['x'].max():.6f}")
print(f"  Кінцеве значення: {df['x'].iloc[-1]:.6f}")
print(f"\nЗначення F(t):")
print(f"  Початкове: {df['F'].iloc[0]:.6f}")
print(f"  Кінцеве: {df['F'].iloc[-1]:.6f}")

# plt.show() - закоментовано, оскільки може не працювати без дисплея