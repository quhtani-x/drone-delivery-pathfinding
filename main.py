import heapq
import random
import sys
import pygame

# DISCLAIMER , most of the comments has been added by Ai as my code didnt have much comments and i told the Ai to explain the code , also remove dead commented code

# AUTONOMOUS DRONE DELIVERY.
# a drone has to fly from the depot to a delivery house across a city grid.
# there are buildings / no-fly zones it must avoid. it plans the shortest safe
# route with A* and then flies to it it. to test , click anywhere to drop a new delivery point
# and watch it replan live.

COLS, ROWS = 40, 26
CELL = 24
W, H = COLS * CELL, ROWS * CELL + 60

pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("autonomous drone delivery - A* path planning")
font = pygame.font.SysFont("consolas", 18)
clock = pygame.time.Clock()

# grid: 0 = open air, 1 = building / no-fly zone
grid = [[0] * COLS for _ in range(ROWS)]


def make_buildings():
    # put some rectangular "buildings" the drone cant fly through
    for _ in range(18):
        bx = random.randint(2, COLS - 6)
        by = random.randint(2, ROWS - 5)
        bw = random.randint(2, 4)
        bh = random.randint(2, 4)
        for y in range(by, min(by + bh, ROWS)):
            for x in range(bx, min(bx + bw, COLS)):
                grid[y][x] = 1


make_buildings()
depot = (1, ROWS // 2)
target = (COLS - 2, ROWS // 2)
grid[depot[1]][depot[0]] = 0
grid[target[1]][target[0]] = 0


def neighbors(node):
    x, y = node
    # 8 directions so it can fly diagonally
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < COLS and 0 <= ny < ROWS and grid[ny][nx] == 0:
            yield (nx, ny), (1.0 if dx == 0 or dy == 0 else 1.41)


def heuristic(a, b):
    # diagonal distance estimate
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def astar(start, goal):
    
    frontier = [(0, start)]
    came = {start: None}
    cost = {start: 0}
    while frontier:
        _, cur = heapq.heappop(frontier)
        if cur == goal:
            break
        for nxt, step in neighbors(cur):
            new = cost[cur] + step
            if nxt not in cost or new < cost[nxt]:
                cost[nxt] = new
                heapq.heappush(frontier, (new + heuristic(nxt, goal), nxt))
                came[nxt] = cur
    if goal not in came:
        return []
    
    path = []
    node = goal
    while node:
        path.append(node)
        node = came[node]
    return path[::-1]


path = astar(depot, target)
drone_step = 0.0

running = True
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.MOUSEBUTTONDOWN:
            # click to set a new delivery target and replan
            mx, my = e.pos
            gx, gy = mx // CELL, my // CELL
            if 0 <= gx < COLS and 0 <= gy < ROWS and grid[gy][gx] == 0:
                target = (gx, gy)
                path = astar(depot, target)
                drone_step = 0.0

    screen.fill((18, 22, 30))

    # draw the grid
    for y in range(ROWS):
        for x in range(COLS):
            if grid[y][x] == 1:
                pygame.draw.rect(screen, (70, 80, 100),
                                 (x * CELL, y * CELL, CELL - 1, CELL - 1))

    # draw the planned route
    if path:
        for (x, y) in path:
            pygame.draw.circle(screen, (60, 120, 90),
                               (x * CELL + CELL // 2, y * CELL + CELL // 2), 4)

    # depot + target markers
    pygame.draw.rect(screen, (90, 200, 255),
                     (depot[0] * CELL, depot[1] * CELL, CELL, CELL))
    pygame.draw.rect(screen, (255, 170, 60),
                     (target[0] * CELL, target[1] * CELL, CELL, CELL))

    # move the drone along the path
    if path:
        drone_step = min(drone_step + 0.25, len(path) - 1)
        dx, dy = path[int(drone_step)]
        px, py = dx * CELL + CELL // 2, dy * CELL + CELL // 2
        pygame.draw.circle(screen, (240, 240, 60), (px, py), 8)
        pygame.draw.circle(screen, (240, 240, 60), (px, py), 14, 1)

    status = f"route: {len(path)} cells" if path else "NO SAFE ROUTE - blocked!"
    screen.blit(font.render(status, True, (220, 220, 220)), (12, ROWS * CELL + 12))
    screen.blit(font.render("click to set a new delivery point", True, (150, 160, 180)),
                (W - 380, ROWS * CELL + 12))

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sys.exit()
