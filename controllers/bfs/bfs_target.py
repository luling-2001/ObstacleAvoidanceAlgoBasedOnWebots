class BFS():
    def __init__(self, my_map):
        MAP_SIZE = len(my_map)
        self.step_move = [
            [1, 0],
            [-1, 0],
            [0, 1],
            [0, -1]
        ]
        self.route_type = ['D', 'U', 'R', 'L']
        self.gps_move = [
            [-0.125, 0],
            [0.125, 0],
            [0, 0.125],
            [0, -0.125]
        ]
        self.target = []
        self.robot = {
            'x': -1.4375,
            'y': -1.4375
        }
        self.step_now = {
            'x': 23,
            'y': 0,
            't': 0,
            'way': -1
        }
        self.step_temp = dict()
        self.now = {
            'pos': self.step_now.copy(),
            'record': [self.step_now]
        }
        self.is_visited = [[False for i in range(MAP_SIZE)] for j in range(MAP_SIZE)]

    def bfs(self, my_map):
        MAP_SIZE = len(my_map)
        q = []
        q.append(self.now)

        while len(q):
            temp = dict()
            temp['pos'] = q[0]['pos'].copy()
            temp['record'] = list(q[0]['record'])
            del q[0]

            if my_map[temp['pos']['x']][temp['pos']['y']] == 'E':
                return temp

            for i in range(4):
                new_leaf = dict()
                new_step = dict()
                dx = temp['pos']['x'] + self.step_move[i][0]
                dy = temp['pos']['y'] + self.step_move[i][1]

                if dx >= 0 and dx < MAP_SIZE and dy >= 0 and dy < MAP_SIZE and my_map[dx][dy] != '1' and self.is_visited[dx][dy] is False:
                    new_step['x'] = dx
                    new_step['y'] = dy
                    new_step['t'] = temp['pos']['t'] + 1
                    new_step['way'] = i

                    self.is_visited[new_step['x']][new_step['y']] = True

                    new_leaf['pos'] = new_step.copy()
                    new_leaf['record'] = list(temp['record'])
                    new_leaf['record'].append(new_step)
                    q.append(new_leaf)

    def getTarget(self, result):
        self.target.append([self.robot['x'], self.robot['y']])
        for i in range(1, len(result['record'])):
            self.robot['x'] = self.robot['x'] + self.gps_move[result['record'][i]['way']][0]
            self.robot['y'] = self.robot['y'] + self.gps_move[result['record'][i]['way']][1]
            self.target.append([self.robot['x'], self.robot['y']])
        return self.target

