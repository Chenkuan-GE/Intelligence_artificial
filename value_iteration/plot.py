import itertools
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd

define_param = ["t_success_prob", "game_over_cost", "seed"]
#
t_success_prob = [0.5, 0.7, 0.9]
game_over_cost = [-15, -20, -25, -30]
#
# df = pd.DataFrame([], columns=define_param)
# np.random.seed(20180316)
# x = np.random.randn(4, 4)
#
# f, (ax1, ax2) = plt.subplots(figsize=(6, 6), nrows=2)
#
# sns.heatmap(x, annot=True, ax=ax1)

# sns.heatmap(x, annot=True, ax=ax2, annot_kws={'size': 9, 'weight': 'bold', 'color': 'blue'})
# f, (ax1, ax2) = plt.subplots(figsize=(6,6),nrows=2)

f,(ax1,ax2) = plt.subplots(figsize=(6,6),nrows=2)
x = np.array([[1,1,1],[2,2,1],[2,2,1],[2,2,2]])

sns.heatmap(x, annot=True, ax=ax1, fmt="d")
# ax = sns.heatmap(uniform_data,cmap = 'RdBu', center=0,cbar = True, square = False,xticklabels =['12','22'])#字符串命名
ax1.set_title("heatmap")
ax1.set_xlabel("t_success_prob "+str(t_success_prob)+"\n"+"The value of 1 means risky road\n" +"The value of 2 means the safe path")
ax1.set_ylabel("game_over_cost "+str(game_over_cost))
# sns.heatmap(x, mask=x < 1, ax=ax2, annot=True, annot_kws={"weight": "bold"})   #把小于1的区域覆盖掉
plt.show()