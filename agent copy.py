"""
agent template for the nav challenge
"""

class Agent:
  def __init__(self, cfg:dict):
    """cfg keys: width_m, height_m, resolution, robot_radius, v_max, a_max, dt, sense_cells, goal_tol, goal (x, y)."""
    self.cfg = cfg
    self.goal = cfg["goal"]
    self.res = cfg["resolution"]
    self.blocked = set()


  def step(self, pose:tuple[float, float], scan:tuple[int, int, list[str]]) -> tuple[float, float]:
    """
    called once per tick.

    pose: (x, y) metres from SLAM, ~2 cm gaussian noise.
    scan: (cx0, cy0, rows) -- a (2*sense_cells+1)^2 window of '#'/'.' around the robot. rows[j][i] is cell (cx0+i, cy0+j).
          everything in the window is observed, nothing outside it is.
          the window origin comes from the noisy pose, so walls can land one cell off between scans.
    returns: (vx, vy) world-frame velocity command in m/s. sim clamps speed and acceleration.
    """
    self.ticks += 1
    x = float(pose[0])
    y = float(pose[1])

    sense = self.cfg["sense_cells"]
    vmax = self.cfg["v_max"]

    

    
    return (0.0, 0.0)

  def debug(self) -> dict:
    """
    optional, for `harness.py --viz` only.
    keys: blocked (cells), free (cells), path ([(x, y), ...]).
    """
    return {}
