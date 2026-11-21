"""NMPC parameters (docs/16). Every value is an ASSUMPTION chosen for the sandbox."""
from dataclasses import dataclass


@dataclass(frozen=True)
class NMPCConfig:
    dt: float = 0.1        # s, step of the RK4 discretisation (ASSUMPTION)
    N: int = 20            # stages in the horizon (ASSUMPTION, 2 s)
    V_MAX: float = 1.0     # m/s, forward speed bound (ASSUMPTION)
    W_MAX: float = 1.5     # rad/s, yaw-rate bound (ASSUMPTION)
    Q_P: float = 1.0       # position tracking weight (ASSUMPTION)
    Q_TERM: float = 10.0   # terminal position weight (ASSUMPTION)
    R_U: float = 0.05      # input weight (ASSUMPTION)
    R_DU: float = 0.1      # input-rate weight (ASSUMPTION)
    MARGIN: float = 0.2    # m, clearance added to each obstacle radius (ASSUMPTION)
    W_SLACK: float = 1e4   # penalty on obstacle-constraint slack (ASSUMPTION)
    MAX_OBS: int = 2       # obstacles the problem can take (fixed-size NLP)
    max_iter: int = 200    # IPOPT iteration limit per solve (ASSUMPTION)
