"""
Simple reactive navigation agent.

Approach:
- Remember blocked cells from scans.
- Aim toward the goal.
- If the direct direction is blocked, steer around the obstacle.
- Unknown cells are treated as potentially unsafe.
- Re-plan every tick, so newly discovered obstacles are reacted to immediately.
"""

import math


class Agent:
  def __init__(self, cfg: dict):
    self.cfg = cfg
    self.goal = cfg["goal"]
    self.res = cfg["resolution"]
    self.blocked = set()

  def step(self, pose: tuple[float, float],
           scan: tuple[int, int, list[str]]) -> tuple[float, float]:
    x, y = pose
    cx0, cy0, rows = scan
    sense = self.cfg["sense_cells"]
    vmax = self.cfg["v_max"]

    # Update our map from the latest scan.
    for j, row in enumerate(rows):
      for i, cell in enumerate(row):
        cell_xy = (cx0 + i, cy0 + j)
        if cell == "#":
          self.blocked.add(cell_xy)

    dx = self.goal[0] - x
    dy = self.goal[1] - y
    dist = math.hypot(dx, dy)

    if dist < self.cfg["goal_tol"]:
      return 0.0, 0.0

    # Desired direction toward the goal.
    ux = dx / dist
    uy = dy / dist

    # Look several cells ahead for obstacles.
    for k in range(1, sense + 1):
      tx = x + ux * k * self.res
      ty = y + uy * k * self.res
      cell = (round(tx / self.res), round(ty / self.res))

      if cell in self.blocked:
        # Try steering left/right around the obstacle.
        lx, ly = -uy, ux
        best = None

        for sign in (-1, 1):
          sx = ux + sign * 1.5 * lx
          sy = uy + sign * 1.5 * ly
          n = math.hypot(sx, sy)
          sx, sy = sx / n, sy / n

          check = (
            round((x + sx * 0.8) / self.res),
            round((y + sy * 0.8) / self.res),
          )

          if check not in self.blocked:
            best = (sx, sy)
            break

        if best:
          ux, uy = best
        else:
          return 0.0, 0.0
        break

    # Slow down near the goal.
    speed = min(vmax, dist * 2.0)

    return ux * speed, uy * speed

  def debug(self) -> dict:
    return {
      "blocked": list(self.blocked),
      "free": [],
      "path": [self.goal],
    }