import numpy as np
from classical.vrptw import VRPTW


class Initializer:
    def __init__(self, n, a, b):
        self.n = n
        self.a = a
        self.b = b

        self.MIN_TIME_WINDOW = 5  # lower bound for time window
        self.MAX_TIME_WINDOW = 30  # upper bound for time window

        self.MIN_SERVICE_TIME = 1  # lower bound for service time
        self.MAX_SERVICE_TIME = 20  # upper bound for service time

    def generate_nodes_weight_matrix_time_windows(self):
        n = self.n
        a = self.a
        b = self.b

        np.random.seed(100 * a + b)

        x = (np.random.rand(n) - 0.5) * 20
        y = (np.random.rand(n) - 0.5) * 20

        weight_matrix = np.zeros([n, n])
        for i in range(n):
            for j in range(i + 1, n):
                weight_matrix[i, j] = np.sqrt((x[i] - x[j]) ** 2 + (y[i] - y[j]) ** 2)
                weight_matrix[j, i] = weight_matrix[i, j]

        time_windows = []
        for _ in range(n):
            earliest_time = round(np.random.rand() * self.MIN_TIME_WINDOW)
            latest_time = round(np.random.rand() * self.MAX_TIME_WINDOW + earliest_time)
            time_windows.append([earliest_time, latest_time])
        time_windows[0][0] = 0
        # TODO check whether the following line will not be necessary
        # in the case of implementation in accordance with the Adam and Ozlem draft
        # time_windows = np.insert(time_windows, 0, [0, 0], axis=0)  # depot time window
        time_windows = np.array(time_windows)

        travel_times = np.random.randint(
            self.MIN_SERVICE_TIME, self.MAX_SERVICE_TIME, size=(n, n)
        )
        np.fill_diagonal(travel_times, 0)
        travel_times = ((travel_times + travel_times.T) / 2).astype(int)

        return x, y, weight_matrix, travel_times, time_windows


if __name__ == "__main__":
    # Create VRPTW problem instance:
    n = 3  # number of clients
    m = 2  # number of vehicles

    initializer = Initializer(n + 1, n + 1, 0)
    xc, yc, dist, time, tw = initializer.generate_nodes_weight_matrix_time_windows()

    # Solve VRPTW with classical solver:
    classical_solver = VRPTW(n, m, dist, xc=xc, yc=yc, tw=tw, time=time)
    solution = classical_solver.solve()
    print(solution)
