#!/usr/bin/env python
# a bar plot with errorbars
import numpy as np
import matplotlib.pyplot as plt

N = 7
menMeans = (20, 20, 20, 20, 20, 18, 17)
menStd = (1, 1, 1, 1, 1,1)

ind = np.arange(N)  # the x locations for the groups
width = 0.35       # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind, menMeans, width, color='b')

womenMeans = (20, 18, 12, 11, 6, 5, 4)
womenStd = (1, 1, 1, 1, 1,1)
rects2 = ax.bar(ind + width, womenMeans, width, color='c')

# add some text for labels, title and axes ticks
ax.set_ylabel('Number of success')
ax.set_xlabel('Error')
ax.set_title('Grasp Success Rate')
ax.set_xticks(ind + width)
ax.set_xticklabels(('0.01', '0.02', '0.03', '0.04', '0.05', '0.06'))

ax.legend((rects1[0], rects2[0]), ('Controller', 'No Controller'))


def autolabel(rects):
    # attach some text labels
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                '%d' % int(height),
                ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)
plt.show()
