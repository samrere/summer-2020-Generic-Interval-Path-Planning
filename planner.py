from shutil import rmtree

from adt import *
from fcn import *


class Planner:
    def __init__(self, file_name, wait_cost, transition_cost, start: tuple, goal: tuple, save_path, vertex_wait_cost):
        self.map = build_map(file_name)  # access graph via graph[x,y] notation. x is horizontal, y is vertical
        self.start = start  # start location expressed as a tuple (x,y)
        self.goal = goal  # goal location expressed as a tuple (x,y)
        self.dist_diff = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        self.transition_cost = transition_cost
        self.original_wait_cost = wait_cost
        self.timeline, self.wait_cost = create_timeline(wait_cost, transition_cost, start, goal)
        self.goal_num_intervals = len(self.timeline[goal]) if goal in self.timeline else 1
        # maintain a dictionary of discovered states
        # key is the combination of cfg and i; value is the state object
        self.discovered_states = dict()
        self.save_path = save_path
        self.vertex_wait_cost = vertex_wait_cost

    def intervals(self, loc):
        # return safe intervals for a configuration
        return self.timeline[loc] if loc in self.timeline else [[0, Inf]]

    def state_interval(self, s):
        return tuple(self.timeline[s.loc][s.i]) if s.loc in self.timeline else (0, Inf)

    def M(self, s):
        # return possible neighbours for a given loc
        x, y = s.loc[0], s.loc[1]
        neighbours = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        motion = [(x, y) for x, y in neighbours if self.map[y][x] == 0]
        # add self cfg to next available motion if there are more intervals after.
        if s.i + 1 < len(self.intervals(s.loc)):
            motion.append(s.loc)
        return motion

    def earliestArrival(self, start_t, intvl):
        if intvl[0] <= start_t:
            t = start_t
        else:
            t = intvl[0]
        return t

    def getSuccessor(self, s: State):
        successors = []
        for cfg in self.M(s):
            if cfg == s.loc:
                end_t = self.state_interval(s)[1]
                i = 0
                for intvl in self.intervals(cfg):
                    start_t = intvl[0]
                    if start_t <= end_t:
                        i += 1
                        continue
                    # ignore all intervals after it if the current interval's wait cost is Inf
                    try:
                        if self.wait_cost[cfg][start_t, start_t] == Inf:
                            break
                    except KeyError:
                        pass
                    if (cfg, i) in self.discovered_states:
                        # retrieve the neighbour state object if it was discovered before
                        nbrState = self.discovered_states[cfg, i]
                        nbrState.tempT = intvl[0]
                    else:
                        nbrState = State(loc=cfg, i=i, tempT=intvl[0], goal=self.goal)
                        self.discovered_states[cfg, i] = nbrState
                    successors.append(nbrState)
                    i += 1
            else:
                m_time = 1
                start_t = s.t + m_time
                end_t = self.state_interval(s)[1] + m_time
                i = 0
                for intvl in self.intervals(cfg):
                    if intvl[0] > end_t or intvl[1] < start_t:
                        i += 1
                        continue
                    # ignore the state if transition cost to it is Inf
                    try:
                        if (intvl[0] == intvl[1]) and self.transition_cost[cfg][intvl[0]][s.loc] == Inf:
                            i += 1
                            continue
                    except KeyError:
                        pass
                    arrivalTime = self.earliestArrival(start_t, intvl)
                    if (cfg, i) in self.discovered_states:
                        # retrieve the neighbour state object if it was discovered before
                        nbrState = self.discovered_states[cfg, i]
                        nbrState.tempT = arrivalTime
                    else:
                        nbrState = State(loc=cfg, i=i, tempT=arrivalTime, goal=self.goal)
                        self.discovered_states[cfg, i] = nbrState
                    successors.append(nbrState)
                    i += 1
        return successors

    def c(self, s: State, n: State):
        cost = 0
        if s.loc == n.loc:
            wait_time = n.tempT - s.t
            # cost to transition
            try:
                end_t=self.state_interval(s)[1]
                time_diff = end_t - s.t + 1 if self.vertex_wait_cost else end_t - s.t
                wait_time += time_diff * self.wait_cost[s.loc][s.t, s.t]
            except KeyError:
                pass
            # wait cost of all states between s and n
            for intvl in self.intervals(n.loc)[s.i + 1:n.i]:
                from_t = intvl[0]
                if (from_t, from_t) in self.wait_cost[n.loc]:
                    wait_time += (intvl[1] - intvl[0] + 1) * self.wait_cost[n.loc][from_t, from_t]
            # wait cost at n
            try:
                from_t = self.state_interval(n)[0]
                wait_time += self.wait_cost[n.loc][from_t, from_t]
            except KeyError:
                pass
            cost += wait_time
        else:
            m_time = 1
            transition_start_time = n.tempT - m_time
            cost_to_transition = transition_start_time - s.t
            try:
                cost_to_transition += self.wait_cost[s.loc][s.t, s.t] * (transition_start_time - s.t)
            except KeyError:
                pass
            transition_cost = 1
            try:
                # relative transition cost i.e. cost above 1
                transition_cost += self.transition_cost[n.loc][n.tempT][s.loc]
            except KeyError:
                pass
            target_vertex_cost = 0
            if self.vertex_wait_cost:
                try:
                    target_vertex_cost = self.wait_cost[n.loc][n.tempT, n.tempT]
                except KeyError:
                    pass
            cost += cost_to_transition + transition_cost + target_vertex_cost
        return cost

    def Astar(self):
        # create starting state
        start = State(loc=self.start, i=0, t=0, goal=self.goal)
        start.g = 0
        start.f = start.h
        # add new State into discovered state
        self.discovered_states[start.loc, start.i] = start
        # add into priority queue
        pq = PriorityQueue()
        pq.add(start)
        final_state = None
        counter = 0
        while not pq.isEmpty() and self.goal_num_intervals > 0:
            s = pq.delMin()
            counter += 1
            if s.loc == self.goal:
                self.goal_num_intervals -= 1
                if final_state is None:
                    final_state = s
                else:
                    if final_state.g > s.g:
                        final_state = s

            neighbours = self.getSuccessor(s)
            nbr_loc = collections.defaultdict(dict)  # loc:g:State
            nbr_loc[s.loc][s.g] = s
            for nbr in neighbours:
                if s.loc == (1, 2) and s.i == 0 and nbr.loc == (1, 2) and nbr.i == 1:
                    a = self.c(s, nbr)

                if nbr.g > s.g + self.c(s, nbr):
                    nbr.g = s.g + self.c(s, nbr)
                    nbr.t = nbr.tempT
                    nbr.parent = s
                    nbr.f = nbr.g + nbr.h
                    # if nbr already in pq, perform reduce key operation
                    pq.add(nbr)
        print('Calculation Complete.\n')
        rmtree('path', ignore_errors=True)  # delete directory if it already exists
        print('Summary:')
        print('poped states:', counter)
        print('priority queue is empty:', pq.isEmpty())
        if final_state is None:
            print('\npath not found')
        else:
            path = reconstruct_path(final_state)
            if self.save_path:
                print('\nSaving Path...')
                draw_path(path, self.map, self.original_wait_cost, self.transition_cost, self.vertex_wait_cost)
                print('Saved')
                return path
