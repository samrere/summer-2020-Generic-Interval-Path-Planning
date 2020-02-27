import collections
from os import mkdir

import matplotlib.cm as cmx
import matplotlib.colors as colors
import matplotlib.pyplot as plt
from matplotlib.legend_handler import HandlerPatch
from matplotlib.patches import FancyArrow, Circle, Ellipse
from numpy import Inf

from adt import PriorityQueue


def build_map(file_name):
    with open(file_name) as f:
        lines = f.read().splitlines()
    assert lines[1][:6] == 'height'
    assert lines[2][:5] == 'width'
    height = int(lines[1][6:])
    width = int(lines[2][5:])

    with open(file_name + '.ecbs', 'w') as f:
        f.write('%d,%d\n' % (height + 2, width + 2))
        f.write(','.join(['1' for _ in range(width + 2)]) + '\n')
        for line in lines[4:]:
            f.write('1,' + line.replace('T', '1,').replace('@', '1,').replace('.', '0,') + '1\n')
        f.write(','.join(['1' for _ in range(width + 2)]) + '\n')

    with open(file_name + '.ecbs') as f:
        lines = f.read().splitlines()
    i = 0
    for line in lines[1:]:
        l = line.split(',')
        lines[i] = [int(_) for _ in l]
        i += 1
    map_ = lines[:-1]
    return map_


def create_timeline(wait_cost, transition_cost, start, goal):
    class Node:
        def __init__(self, interval: tuple):
            self.interval = list(interval)
            self.pqLoc = None

        def __str__(self):
            return str(self.interval)

        # example: (2,2)<(3,5); (3,3)<(3,5)
        def __lt__(self, other):
            return self.interval[1] < other.interval[1] if self.interval[0] == other.interval[0] \
                else self.interval[0] < other.interval[0]

        def __gt__(self, other):
            return self.interval[1] > other.interval[1] if self.interval[0] == other.interval[0] \
                else self.interval[0] > other.interval[0]

        def __eq__(self, other):
            return self.interval == other.interval

    new_wait_cost = collections.defaultdict(dict)  # create a new wait cost for each time step
    # put everything inside a list (can be unordered)
    loc_intervals = dict()
    for loc in wait_cost:
        intervals = []
        loc_intervals[loc] = intervals
        for intvl in wait_cost[loc]:
            intervals.append(Node(intvl))
            new_wait_cost[loc][intvl] = wait_cost[loc][intvl]
            for t in range(intvl[0], intvl[1] + 1):
                new_wait_cost[loc][t, t] = wait_cost[loc][intvl]
    for loc in transition_cost:
        if loc not in loc_intervals:
            intervals = []
            loc_intervals[loc] = intervals
        intervals = loc_intervals[loc]
        for t in transition_cost[loc]:
            intervals.append(Node((t, t)))

    # reconstruct
    timeline = dict()
    for loc in loc_intervals:
        timeline[loc] = [[0, Inf]]
        intervals = timeline[loc]
        minHeap = PriorityQueue()
        minHeap.buildHeap(loc_intervals[loc])
        if loc == (1, 1):
            a = minHeap
        while not minHeap.isEmpty():
            minNode = minHeap.delMin()
            # if the current min is equal to the previous one, ignore it.(duplicate element in the heap)
            if minNode is None:
                continue
            else:
                intvl = minNode.interval
            if intvl[0] < intervals[-1][0]:
                if intvl[1] < intervals[-1][0]:  # [[0,2],[3,5],[6,Inf]] + [4,4]/[5,5]
                    tmp_intvl = intervals.pop()
                    tmp_num = intervals[-1][1]
                    intervals[-1][1] = intvl[0] - 1
                    if intervals[-1][1] < intervals[-1][0]: intervals.pop()
                    intervals.append(intvl)
                    if intvl[1] < tmp_num:  # +[4,4]
                        intervals.append([intvl[1] + 1, tmp_num])
                    intervals.append(tmp_intvl)
                else:  # [[0,2],[3,3],[4,Inf]] + [3,4]/[3,5]
                    tmp_intvl = intervals.pop()
                    intervals.append([tmp_intvl[0], intvl[1]])
                    intervals.append([intvl[1] + 1, Inf])
            else:
                if intervals[-1][0] == intvl[0]:
                    intervals[-1][1] = intvl[1]
                else:
                    intervals[-1][1] = intvl[0] - 1
                    intervals.append(intvl)
                intervals.append([intvl[1] + 1, Inf])

    # check timeline is correct
    for loc in timeline:
        intervals = timeline[loc]
        i = 1
        while i < len(intervals):
            intvl = intervals[i]
            assert intervals[i - 1][1] < intvl[0] <= intvl[1], 'Timeline error'
            i += 1
    # trim: get rid of the first few intervals at the goal location, because they won't be reached.
    min_time = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    if goal in timeline:
        intervals = timeline[goal]
        for i in range(len(intervals)):
            if intervals[i][1] >= min_time:
                break
        timeline[goal] = intervals[i:]
    return timeline, new_wait_cost


def reconstruct_path(s):
    print(f'cost to goal: {s.f}\n')
    stack = [(s.loc, s.t)]
    while s.parent is not None:
        s = s.parent
        stack.append((s.loc, s.t))
    path = []
    while stack:
        path.append(stack.pop())
    print('Path:', path)
    return path


def make_legend_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
    p = FancyArrow(0, 0.5 * height, width, 0, length_includes_head=True, head_width=0.75 * height)
    return p


class HandlerEllipse(HandlerPatch):
    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize, trans):
        center = 0.5 * width - 0.5 * xdescent, 0.5 * height - 0.5 * ydescent
        p = Ellipse(xy=center, width=height + xdescent,
                    height=height + ydescent)
        self.update_prop(p, orig_handle, legend)
        p.set_transform(trans)
        return [p]


def drawPath(path, map_, wait_cost, transition_cost, vertex_wait_cost):
    mkdir('path')
    max_len = 0
    for i in map_:
        if len(i) > max_len:
            max_len = len(i)
    for i in map_:
        i.extend([1] * (max_len - len(i)))
    # robot location
    robot_loc = []
    for i in range(1, len(path)):
        add_times = path[i][1] - path[i - 1][1]
        for _ in range(add_times):
            robot_loc.append(path[i - 1][0])
    robot_loc.append(path[-1][0])
    ##
    max_cost = None
    min_cost = None
    square_list = [[] for _ in range(len(robot_loc))]
    for loc in wait_cost:
        for (from_t, to_t) in wait_cost[loc]:
            for t in range(from_t, to_t + 1):
                if t < len(robot_loc):
                    cost = wait_cost[loc][(from_t, to_t)]
                    if cost != Inf and (max_cost is None or cost > max_cost):
                        max_cost = cost
                    if cost != Inf and (min_cost is None or cost < min_cost):
                        min_cost = cost
                    if vertex_wait_cost:
                        square_list[t].append((loc, cost))
                    else:
                        square_list[t - 1].append((loc, cost))
                else:
                    break

    arrow_list = [[] for _ in range(len(robot_loc))]

    for to_loc in transition_cost:
        for t in transition_cost[to_loc]:
            if t >= len(robot_loc):
                continue
            for from_loc in transition_cost[to_loc][t]:
                cost = transition_cost[to_loc][t][from_loc]
                if cost != Inf and (max_cost is None or cost > max_cost):
                    max_cost = cost
                if cost != Inf and (min_cost is None or cost < min_cost):
                    min_cost = cost

                arrow_list[t - 1].append((from_loc, to_loc, cost))
    # start plotting
    ## colormap
    if max_cost is not None:
        if min_cost is None:
            min_cost = 0
        cmap = colors.LinearSegmentedColormap.from_list('my_colormap', ['green', 'red'], 256)
        cNorm = colors.Normalize(vmin=min_cost, vmax=max_cost)
        scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cmap)

    for t in range(len(robot_loc)):
        fig, ax = plt.subplots()
        obj, label = [], []
        # draw color bar
        if min_cost is not None and max_cost is not None:
            zvals = [[min_cost, max_cost]]
            cmap = colors.LinearSegmentedColormap.from_list('my_colormap', ['green', 'red'], 256)
            img = plt.imshow(zvals, interpolation='Nearest', cmap=cmap)
            plt.colorbar(img, cmap=cmap)

        # draw map
        plt.imshow(map_, interpolation='None', cmap='Greys')

        ax.xaxis.tick_top()

        ax.set_yticks([_ + 0.5 for _ in range(len(map_) - 1)], minor=True)
        ax.yaxis.grid(True, which='minor')
        ax.set_xticks([_ + 0.5 for _ in range(len(map_[0]) - 1)], minor=True)
        ax.xaxis.grid(True, which='minor')

        ## plot square
        if square_list[t]:
            for (loc, cost) in square_list[t]:
                if cost == Inf:
                    # square = plt.Rectangle((loc[0] - 0.5, loc[1] - 0.5), 1, 1, color='k')
                    square = plt.Circle(loc, 0.25, color='k')
                else:
                    # square = plt.Rectangle((loc[0] - 0.5, loc[1] - 0.5), 1, 1, color=scalarMap.to_rgba(cost))
                    square = plt.Circle(loc, 0.25, color=scalarMap.to_rgba(cost))
                ax.add_artist(square)
        obj.append(Circle((0.5, 0.5), facecolor='k', edgecolor='None'))
        if vertex_wait_cost:
            label.append('vertex cost')
        else:
            label.append('wait cost')

        ## plot arrow
        if arrow_list[t]:
            for (from_loc, to_loc, cost) in arrow_list[t]:
                x, y, dx, dy = from_loc[0], from_loc[1], (to_loc[0] - from_loc[0]) * .4, (
                        to_loc[1] - from_loc[1]) * .4
                if cost == Inf:
                    arrow = plt.arrow(x, y, dx, dy, color='k', head_width=0.05, head_length=0.03, linewidth=2,
                                      length_includes_head=True)
                else:
                    arrow = plt.arrow(x, y, dx, dy, color=scalarMap.to_rgba(cost), head_width=0.05,
                                      head_length=0.03, linewidth=2, length_includes_head=True)
                ax.add_artist(arrow)
        obj.append(plt.arrow(0, 0, 0.5, 0.6, color='k'))
        label.append('transition cost')

        ## plot robot
        robot = plt.Circle(robot_loc[t], 0.15, color='y')
        ax.add_artist(robot)
        obj.append(Circle((0.5, 0.5), facecolor='y', edgecolor='None'))
        label.append('robot')

        plt.title(f'vertex wait cost={vertex_wait_cost}, t={t}')
        plt.legend(obj, label, loc='upper left',
                   handler_map={FancyArrow: HandlerPatch(patch_func=make_legend_arrow), Circle: HandlerEllipse()})

        plt.savefig(f'path/t={t}.png')
