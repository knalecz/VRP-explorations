from classical.vrptw import VRPTW
from initializer import SimpleInitializer


if __name__ == "__main__":
    # Create VRPTW problem instance:
    n = 3  # number of clients
    m = 2  # number of vehicles

    initializer = SimpleInitializer(n + 1, n + 1, 0)
    xc, yc, dist, time, tw = initializer.generate_nodes_weight_matrix_time_windows()

    # Solve VRPTW with classical solver:
    classical_solver = VRPTW(n, m, dist, xc=xc, yc=yc, tw=tw, time=time)
    solution = classical_solver.solve()
    print(solution)
