"""Fast grid-navigation agent.

Use the goal directly whenever the currently observed walls do not block that
line.  When a wall blocks the direct route, use A* on an inflated grid.
Unknown cells are allowed in planning because the robot only sees a local
window; the short look-ahead means the next command stays near the sensor.
"""

import heapq
import math
from collections import deque


class Agent:
  def __init__(self, cfg: dict):
    self.res = float(cfg["resolution"])
    self.w = round(cfg["width_m"] / self.res)
    self.h = round(cfg["height_m"] / self.res)
    self.v_max = float(cfg["v_max"])
    self.goal_tol = float(cfg["goal_tol"])
    self.goal = tuple(cfg["goal"])

    # 0.30 m / 0.10 m = 3 cells.  One extra cell gives a little margin for
    # pose noise without making narrow passages automatically impossible.
    self.inflate = math.ceil(float(cfg["robot_radius"]) / self.res)
    self.soft = self.inflate + 2

    self.wall_score = {}
    self.blocked = set()
    self.unsafe = set()
    self.near_wall = set()
    self.path = []
    self.path_i = 0
    self.ticks = 0
    self.last_plan = -9999
    self.map_changed = True
    self.last_v = (0.0, 0.0)

  def step(self, pose: tuple[float, float], scan: tuple[int, int, list[str]]) -> tuple[float, float]:
    self.ticks += 1
    x, y = pose
    self.update_map(scan)

    gx, gy = self.goal
    if math.hypot(gx - x, gy - y) <= self.goal_tol:
      self.last_v = (0.0, 0.0)
      return (0.0, 0.0)

    # Most important fast path: if nothing currently blocks the goal direction,
    # do not run A* or make the robot wander left/right.
    if self.clear_line(x, y, gx, gy):
      target = self.goal
    else:
      start = self.nearest_safe(self.cell(x, y))
      goal = self.nearest_safe(self.cell(gx, gy))
      if self.map_changed or not self.path or self.ticks - self.last_plan >= 20:
        self.path = self.astar(start, goal)
        self.path_i = 0
        self.last_plan = self.ticks
        self.map_changed = False
      target = self.path_target(x, y)

      # No current route: wait one tick for a fresh scan rather than driving
      # blindly through a wall.
      if target is None:
        self.last_v = (0.0, 0.0)
        return (0.0, 0.0)

    vx, vy = self.velocity(x, y, target)
    self.last_v = (vx, vy)
    return vx, vy

  def update_map(self, scan: tuple[int, int, list[str]]) -> None:
    cx0, cy0, rows = scan
    changed = False

    for j, row in enumerate(rows):
      for i, value in enumerate(row):
        cell = (cx0 + i, cy0 + j)
        if not self.inside(*cell):
          continue

        score = self.wall_score.get(cell, 0)
        score = min(3, score + 1) if value == "#" else max(0, score - 1)
        self.wall_score[cell] = score

        if score >= 2 and cell not in self.blocked:
          self.blocked.add(cell)
          changed = True
        elif score == 0 and cell in self.blocked:
          self.blocked.remove(cell)
          changed = True

    if changed:
      self.build_safety()
      self.map_changed = True

  def build_safety(self) -> None:
    self.unsafe = set()
    self.near_wall = set()
    for bx, by in self.blocked:
      for dy in range(-self.soft, self.soft + 1):
        for dx in range(-self.soft, self.soft + 1):
          d2 = dx * dx + dy * dy
          if d2 > self.soft * self.soft:
            continue
          cell = (bx + dx, by + dy)
          if not self.inside(*cell):
            continue
          self.near_wall.add(cell)
          if d2 <= self.inflate * self.inflate:
            self.unsafe.add(cell)

  def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    if not self.inside(*start) or not self.inside(*goal):
      return []
    if start in self.unsafe or goal in self.unsafe:
      return []

    heap = [(self.hcost(start, goal), 0.0, start[1], start[0])]
    costs = {start: 0.0}
    parent = {start: None}
    moves = (
      (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
      (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
    )

    while heap:
      _, g, y, x = heapq.heappop(heap)
      here = (x, y)
      if g != costs[here]:
        continue
      if here == goal:
        return self.make_path(parent, goal)

      for dx, dy, step in moves:
        nx, ny = x + dx, y + dy
        nxt = (nx, ny)
        if not self.inside(nx, ny) or nxt in self.unsafe:
          continue
        if dx and dy and ((x + dx, y) in self.unsafe or (x, y + dy) in self.unsafe):
          continue

        # Only a small soft penalty: shortest routes remain attractive, while
        # an equally short wall-hugging route loses to one with more clearance.
        extra = 0.35 if nxt in self.near_wall else 0.0
        new_g = g + step * (1.0 + extra)
        if new_g < costs.get(nxt, float("inf")):
          costs[nxt] = new_g
          parent[nxt] = here
          heapq.heappush(heap, (new_g + self.hcost(nxt, goal), new_g, ny, nx))

    return []

  def path_target(self, x: float, y: float):
    if not self.path:
      return None

    current = self.cell(x, y)
    while self.path_i + 1 < len(self.path):
      nx, ny = self.path[self.path_i + 1]
      if (nx - current[0]) ** 2 + (ny - current[1]) ** 2 <= 4:
        self.path_i += 1
      else:
        break

    target = self.path_i
    # About 1.2 m is far enough to keep speed up but still well inside the
    # 2.5 m sensing radius.
    for i in range(self.path_i, min(len(self.path), self.path_i + 13)):
      tx = (self.path[i][0] + 0.5) * self.res
      ty = (self.path[i][1] + 0.5) * self.res
      if math.hypot(tx - x, ty - y) <= 1.2 and self.clear_line(x, y, tx, ty):
        target = i

    cx, cy = self.path[target]
    return (cx + 0.5) * self.res, (cy + 0.5) * self.res

  def clear_line(self, x1: float, y1: float, x2: float, y2: float) -> bool:
    distance = math.hypot(x2 - x1, y2 - y1)
    steps = max(2, math.ceil(distance / (self.res * 0.4)))
    for i in range(1, steps + 1):
      t = i / steps
      cell = self.cell(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)
      if cell in self.unsafe:
        return False
    return True

  def velocity(self, x: float, y: float, target: tuple[float, float]) -> tuple[float, float]:
    dx = target[0] - x
    dy = target[1] - y
    distance = math.hypot(dx, dy)
    if distance < 0.03:
      return (0.0, 0.0)

    ux, uy = dx / distance, dy / distance
    speed = self.v_max
    old_speed = math.hypot(*self.last_v)
    if old_speed > 0.1:
      dot = (self.last_v[0] * ux + self.last_v[1] * uy) / old_speed
      if dot < -0.2:
        speed = 0.55
      elif dot < 0.3:
        speed = 0.85
      elif dot < 0.7:
        speed = 1.35

    if distance < 0.7:
      speed = min(speed, max(0.45, distance * 2.5))
    return ux * speed, uy * speed

  def nearest_safe(self, start: tuple[int, int]) -> tuple[int, int]:
    if self.inside(*start) and start not in self.unsafe:
      return start
    queue = deque([start])
    seen = {start}
    while queue:
      x, y = queue.popleft()
      for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
        cell = (nx, ny)
        if not self.inside(nx, ny) or cell in seen:
          continue
        if cell not in self.unsafe:
          return cell
        seen.add(cell)
        queue.append(cell)
    return start

  @staticmethod
  def hcost(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

  @staticmethod
  def make_path(parent, goal):
    path = []
    while goal is not None:
      path.append(goal)
      goal = parent[goal]
    return path[::-1]

  def cell(self, x: float, y: float) -> tuple[int, int]:
    return int(x / self.res), int(y / self.res)

  def inside(self, x: int, y: int) -> bool:
    return 0 <= x < self.w and 0 <= y < self.h

  def debug(self) -> dict:
    return {
      "blocked": list(self.blocked),
      "free": [],
      "path": [((x + 0.5) * self.res, (y + 0.5) * self.res) for x, y in self.path],
    }
