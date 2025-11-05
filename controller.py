import numpy as np
import matplotlib.pyplot as plt
from vehicle_model import vehicle_status, vehicle_model
from referenceline import reference_line
from math import *
import osqp
import scipy.sparse as sp


ts = 0.2  # sample time
horizon = 10  # horizon length

max_jerk = 0.5
max_kappa_rate = 0.05


class MPC_Controller:
    def __init__(self, ts: float, horizon: int):
        self.ts = ts
        self.horizon = horizon
        # weight matrix
        self.Q = sp.eye(4)
        self.R = sp.eye(4)
        self.QN = self.Q
        # reference
        self.ref = []
        # initial status
        self.init_status = vehicle_status(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def Update(self, init_status: vehicle_status, trajectory):
        self.init_status = init_status
        x0 = init_status.x
        y0 = init_status.y
        theta0 = init_status.theta
        kappa0 = init_status.kappa
        v0 = init_status.velocity
        self.ref = trajectory  # update reference
        state0 = np.array([x0, y0, v0, kappa0])  # initial state
        Ad = sp.csc_matrix(
            [
                [1.0, 0.0, -v0 * sin(theta0) * ts, 0.0],
                [0.0, 1.0, v0 * cos(theta0) * ts, 0.0],
                [0.0, 0.0, 1.0, v0 * ts],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        Bd = sp.csc_matrix([[0.0], [0.0], [0.0], [1.0]])
        Wd = sp.csc_matrix(
            [v0 * np.cos(theta0) * ts, v0 * np.sin(theta0) * ts, 0.0, 0.0]
        )
        [nx, nu] = Bd.shape
        u_lower = -max_kappa_rate
        u_upper = max_kappa_rate
        # construct the cost
        P = sp.block_diag(
            [
                sp.kron(sp.eye(horizon), self.Q),
                self.QN,
                sp.kron(sp.eye(horizon), self.R),
            ],
            format="csc",
        )
        q = np.array([])
        for i in range(horizon - 1):
            xr = trajectory[i].x
            yr = trajectory[i].y
            dl_theta = trajectory[i].angle - trajectory[0].angle
            kappa_r = trajectory[i].kappa
            state_r = np.array([xr, yr, dl_theta, kappa_r])
            q = np.hstack([q, -self.Q @ state_r])
        state_rn = np.array([self.ref[-1].x, self.ref[-1].y, self.ref[-1].angle - trajectory[0].angle,
                             self.ref[-1].kappa])
        q = np.hstack([q, -self.QN @ state_rn, np.zeros(horizon * nu)])
        q = np.hstack([q, np.zeros(horizon*nu)])
        # - linear equations
        Ax = sp.kron(sp.eye(horizon + 1), -sp.eye(nx)) + sp.kron(sp.eye(horizon + 1, k=-1), Ad)
        Bu = sp.kron(sp.vstack([sp.csc_matrix((1, horizon)), sp.eye(horizon)]), Bd)
        Aeq = sp.hstack([Ax, Bu])
        beq = np.zeros(horizon * nx + horizon * nu)
        leq = np.hstack([-state0, np.zeros(horizon * nx)])
        ueq = leq
        # - inequality constraints
        Aineq = sp.eye((horizon + 1) * nx + horizon)
        lineq = np.hstack([np.kron(np.ones(horizon + 1), -np.inf), np.kron(np.ones(horizon), u_lower)])
        uineq = np.hstack([np.kron(np.ones(horizon + 1), np.inf), np.kron(np.ones(horizon), u_upper)])
        # OSQP constraints
        A = sp.vstack([Aeq, Aineq])
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])
        # Create an OSQP object
        prob = osqp.OSQP()
        prob.setup(P, q, A, l, u, warm_starting = True)
        res = prob.solve()
        # get control
        ctrl = res.x[-horizon * nu:-(horizon - 1) * nu]

        return ctrl

    def get_ref_points(self, ref_points: list):
        self.ref = ref_points
        return True


if __name__ == "__main__":
    planner_t = 10.0
    E0Y = vehicle_model("E0Y", 0.0, 0.005, 10.0, 0.5, -5.0, -5.0)
    ref_lin = reference_line(-5.0, 0.005, 0.0005)
    trajectory = ref_lin.get_ref_points(planner_t * E0Y.velocity)

    controller = MPC_Controller(ts, horizon)
    nearest_point = ref_lin.get_nearest_point(E0Y.X, E0Y.Y)

    ts = controller.ts
    horizon = controller.horizon
    ds = [E0Y.velocity * ts * i for i in range(horizon)]
    control_ref = []
    for i in range(horizon):
        control_ref.append(ref_lin.get_point_from_S(nearest_point, ds[i]))
    controller.Update(E0Y.get_vehicle_status(), control_ref)

    print("pause")
    '''
    N = 3
    nx = 4
    nu = 1

    ts = 0.2


    Q = sp.eye(2)
    QN = sp.eye(2)
    R = sp.eye(2)
    P = sp.block_diag(
        [
            sp.kron(sp.eye(horizon), Q),
            QN,
            sp.kron(sp.eye(horizon), R),
        ]
    ).toarray()

    print(P)

    xr = np.array([1.0, 0.5])

    q = np.hstack(
        [
            np.kron(np.ones(1), -Q @ xr),
        ]
    )
    print(q)
    q = np.hstack([q, QN @ xr])
    print(q)
    Ad = sp.csc_matrix(
        [
            [1.0, 0.0, -10.0 * sin(0.02) * ts, 0.0],
            [0.0, 1.0, 10.0 * cos(0.02) * ts, 0.0],
            [0.0, 0.0, 1.0, 10.0 * ts],
            [0.0, 0.0, 0.0, 1.0]
        ]
    )
    print("Ad:", Ad.toarray())
    Ax = sp.kron(sp.eye(N + 1), -sp.eye(nx)) + sp.kron(sp.eye(N + 1, k=-1), Ad)
    print("Ax:", Ax.toarray())
    Aineq = sp.eye((3 + 1) * 2 + 3 * 1).toarray()
    print(Aineq)
    '''
