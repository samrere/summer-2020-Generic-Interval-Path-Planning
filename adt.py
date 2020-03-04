from math import sqrt

from numpy import Inf

'''
Abstract Data Type: State and Priority queue implemented using min heap
Please be noted that the delMin method in priority queue is a bit different,
which only delete different element.

Author: Yu Hou
Email: yhou0015@student.monash.edu
Last edit: 04/03/2020
'''


class State:
    def __init__(self, loc, i, goal, t=None, tempT=None):
        self.loc = loc
        self.i = i
        self.t = t
        self.tempT = tempT  # temporary time, replace self.t when condition applies
        self.parent = None
        self.g = Inf
        self.f = Inf
        self.h = sqrt((loc[0] - goal[0]) ** 2 + (loc[1] - goal[1]) ** 2)
        self.pqLoc = None  # location in priority queue

    def __str__(self):
        parentLoc = self.parent.loc if self.parent is not None else None
        parentInterval = self.parent.i if self.parent is not None else None
        return f'loc={self.loc},i={self.i},t={self.t},tempT={self.tempT},par={parentLoc, parentInterval},g={self.g},f={self.f},h={self.h}'

    def __gt__(self, other):
        return self.f > other.f

    def __lt__(self, other):
        return self.f < other.f


class PriorityQueue:
    def __init__(self):
        self.heapList = [0]
        self.currentSize = 0
        self.lastDeletedMin = None

    def isEmpty(self):
        return self.currentSize == 0

    def percUp(self, i):
        tmp = self.heapList[i]
        while i // 2 != 0 and tmp < self.heapList[i // 2]:
            self.heapList[i] = self.heapList[i // 2]
            self.heapList[i // 2].pqLoc = i
            i //= 2
        self.heapList[i] = tmp
        tmp.pqLoc = i

    def add(self, k):
        if k.pqLoc is None:
            self.heapList.append(k)
            self.currentSize += 1
            k.pqLoc = self.currentSize
        self.percUp(k.pqLoc)

    def delMin(self):
        assert len(self.heapList) > 1, "can't pop from empty heap"
        # replace with the last item
        retval = self.heapList[1]
        retval.pqLoc = None
        self.heapList[1] = self.heapList[self.currentSize]
        self.currentSize -= 1
        self.heapList.pop()
        self.percDown(1)
        if (self.lastDeletedMin is not None) and (
                self.lastDeletedMin == retval):  # only delete different min. i.e. there may be equal nodes in the heap
            return
        self.lastDeletedMin = retval
        return retval

    def percDown(self, i):
        if not self.isEmpty():
            tmp = self.heapList[i]
            while 2 * i <= self.currentSize:
                mc = self.minChild(i)
                if tmp > self.heapList[mc]:
                    self.heapList[i] = self.heapList[mc]
                    self.heapList[i].pqLoc = i
                else:
                    break
                i = mc
                # mc=self.minChild(i)
            self.heapList[i] = tmp
            tmp.pqLoc = i

    def minChild(self, i):
        if 2 * i + 1 > self.currentSize:
            return 2 * i
        else:
            if self.heapList[2 * i] < self.heapList[2 * i + 1]:
                return 2 * i
            else:
                return 2 * i + 1

    def buildHeap(self, alist):
        i = len(alist) // 2
        self.currentSize = len(alist)
        self.heapList = [0] + alist[:]
        while i > 0:
            self.percDown(i)
            i = i - 1
