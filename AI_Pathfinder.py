import pygame
import heapq
import sys
from collections import deque

SCREEN_W, SCREEN_H = 950, 650
MAP_AREA = 600
GRID_DIM = 10  
TILE_SIZE = MAP_AREA // GRID_DIM
FPS_DELAY = 15

BG_WHITE = (255, 255, 255)
UI_GRAY = (220, 220, 220)
TEXT_BLACK = (0, 0, 0)
WALL_RED = (255, 0, 0)
START_GRN = (0, 150, 0)
GOAL_BLU = (0, 0, 255)
SEARCHED_GRAY = (169, 169, 169)
EDGE_YELLOW = (255, 255, 0)
PATH_ORANGE = (255, 165, 0)

WALK_DIR = [(-1, 0), (0, 1), (1, 0), (1, 1), (0, -1), (-1, -1)]

class GridNavigator:
    def __init__(self):
        pygame.init()
        self.surface = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("AI Pathing Engine")
        self.font_main = pygame.font.SysFont('Arial', 22, bold=True)
        self.font_data = pygame.font.SysFont('Arial', 18, bold=True)
        self.reset_state()

    def reset_state(self):
        """Clears the pathfinding data and resets the grid."""
        self.matrix = [[0 for _ in range(GRID_DIM)] for _ in range(GRID_DIM)]
        self.start_pos, self.end_pos = (6, 7), (6, 1)
        self.explored = set()
        self.border_nodes = set()
        self.final_route = []
        self.flow_map = {}
        for r in range(1, 8): 
            self.matrix[r][5] = -1

    def draw_world(self):
        self.surface.fill(BG_WHITE)
        for r in range(GRID_DIM):
            for c in range(GRID_DIM):
                box = (c * TILE_SIZE, r * TILE_SIZE, TILE_SIZE, TILE_SIZE)
                color = BG_WHITE
                
                if self.matrix[r][c] == -1: color = WALL_RED
                elif (r, c) == self.start_pos: color = START_GRN
                elif (r, c) == self.end_pos: color = GOAL_BLU
                elif (r, c) in self.final_route: color = PATH_ORANGE
                elif (r, c) in self.border_nodes: color = EDGE_YELLOW
                elif (r, c) in self.explored: color = SEARCHED_GRAY
                
                pygame.draw.rect(self.surface, color, box)
                pygame.draw.rect(self.surface, TEXT_BLACK, box, 1)
                
                if (r, c) == self.start_pos:
                    val = "s"
                elif (r, c) == self.end_pos:
                    val = "t"
                elif self.matrix[r][c] == -1:
                    val = "-1"
                elif hasattr(self, 'node_costs') and (r, c) in self.node_costs:
                    val = str(self.node_costs[(r, c)])
                else:
                    val = "0"

                txt = self.font_data.render(val, True, TEXT_BLACK)
                self.surface.blit(txt, (c * TILE_SIZE + 18, r * TILE_SIZE + 20))

        pygame.draw.rect(self.surface, UI_GRAY, (600, 0, 350, SCREEN_H))
        self.draw_legend()
        pygame.display.update()
    def draw_legend(self):
        labels = ["1: BFS", "2: DFS", "3: UCS", "4: DLS", "5: IDDFS", "6: Bidirectional", "R: Reset"]
        for i, text in enumerate(labels):
            img = self.font_main.render(text, True, TEXT_BLACK)
            self.surface.blit(img, (630, 60 + i * 65))

    def refresh_ui(self, frontier):
        self.border_nodes = set(frontier)
        self.draw_world()
        pygame.time.delay(FPS_DELAY)
        for e in pygame.event.get():
            if e.type == pygame.QUIT: pygame.quit(); sys.exit()

    def get_valid_steps(self, p):
        return [(p[0] + dr, p[1] + dc) for dr, dc in WALK_DIR 
                if 0 <= p[0] + dr < GRID_DIM and 0 <= p[1] + dc < GRID_DIM 
                and self.matrix[p[0] + dr][p[1] + dc] != -1]

    def build_path(self, target, parent_dict):
        self.final_route = []
        curr = target
        while curr in parent_dict and curr is not None:
            self.final_route.append(curr)
            curr = parent_dict[curr]
            self.draw_world() 
            pygame.time.delay(35)


    def run_bfs(self):
        q = deque([self.start_pos])
        self.flow_map = {self.start_pos: None}
        self.explored = {self.start_pos}
        while q:
            curr = q.popleft()
            if curr == self.end_pos: return self.build_path(curr, self.flow_map)
            for nxt in self.get_valid_steps(curr):
                if nxt not in self.explored:
                    self.explored.add(nxt)
                    self.flow_map[nxt] = curr
                    q.append(nxt)
                    self.refresh_ui(q)

    def run_dfs(self):
        stack = [self.start_pos]
        self.flow_map = {self.start_pos: None}
        self.explored = set()
        while stack:
            curr = stack.pop()
            if curr == self.end_pos: return self.build_path(curr, self.flow_map)
            if curr not in self.explored:
                self.explored.add(curr)
                for nxt in reversed(self.get_valid_steps(curr)):
                    if nxt not in self.explored:
                        self.flow_map[nxt] = curr
                        stack.append(nxt)
                        self.refresh_ui(stack)

    def run_ucs(self):
        self.node_costs = {self.start_pos: 0} 
        pq = [(0, self.start_pos)]
        self.flow_map = {self.start_pos: None}
        self.explored = set()

        while pq:
            c, curr = heapq.heappop(pq)
            if curr == self.end_pos: return self.build_path(curr, self.flow_map)
            
            if curr in self.explored: continue
            self.explored.add(curr)

            for nxt in self.get_valid_steps(curr):
                new_c = c + 1  # Standard step cost
                if nxt not in self.node_costs or new_c < self.node_costs[nxt]:
                    self.node_costs[nxt] = new_c
                    self.flow_map[nxt] = curr
                    heapq.heappush(pq, (new_c, nxt))
                    self.refresh_ui([x[1] for x in pq])

    def run_dls(self, limit):
        stack = [(self.start_pos, 0)]
        local_map = {self.start_pos: None}
        self.explored = set()
        while stack:
            curr, depth = stack.pop()
            if curr == self.end_pos:
                self.flow_map = local_map
                return True
            if depth < limit:
                if curr not in self.explored:
                    self.explored.add(curr)
                    for nxt in reversed(self.get_valid_steps(curr)):
                        if nxt not in self.explored:
                            local_map[nxt] = curr
                            stack.append((nxt, depth + 1))
                            self.refresh_ui([node for node, d in stack])
        return False

    def run_iddfs(self):
        for d in range(GRID_DIM * GRID_DIM):
            if self.run_dls(d):
                return self.build_path(self.end_pos, self.flow_map)

    def run_bidir(self):
        fwd_q, rev_q = deque([self.start_pos]), deque([self.end_pos])
        fwd_map, rev_map = {self.start_pos: None}, {self.end_pos: None}
        while fwd_q and rev_q:
            c_f = fwd_q.popleft()
            self.explored.add(c_f)
            for n in self.get_valid_steps(c_f):
                if n not in fwd_map:
                    fwd_map[n] = c_f
                    fwd_q.append(n)
                    if n in rev_map: return self.stitch_bidir(n, fwd_map, rev_map)
            c_r = rev_q.popleft()
            self.explored.add(c_r)
            for n in self.get_valid_steps(c_r):
                if n not in rev_map:
                    rev_map[n] = c_r
                    rev_q.append(n)
                    if n in fwd_map: return self.stitch_bidir(n, fwd_map, rev_map)
            self.refresh_ui(list(fwd_q) + list(rev_q))

    def stitch_bidir(self, meet, m1, m2):
        self.final_route = []
        c = meet
        while c is not None: self.final_route.append(c); c = m1.get(c)
        c = m2.get(meet)
        while c is not None: self.final_route.insert(0, c); c = m2.get(c)
        self.draw_world()

    def main_loop(self):
        while True:
            self.draw_world()
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit(); sys.exit()
                if event.type == pygame.KEYDOWN:
                    self.explored, self.border_nodes, self.final_route = set(), set(), []
                    self.node_costs = {}
                    if event.key == pygame.K_1: self.run_bfs()
                    if event.key == pygame.K_2: self.run_dfs()
                    if event.key == pygame.K_3: self.run_ucs()
                    if event.key == pygame.K_4: 
                        if self.run_dls(20): self.build_path(self.end_pos, self.flow_map)
                    if event.key == pygame.K_5: self.run_iddfs()
                    if event.key == pygame.K_6: self.run_bidir()
                    if event.key == pygame.K_r: self.reset_state()

if __name__ == "__main__":
    GridNavigator().main_loop()