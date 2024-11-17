import heapq
import math
import random


# Định nghĩa hàm lượng giá h(n) (Manhattan hoặc Euclidean)
def heuristic(current, goal, method="manhattan"):
    x1, y1 = current
    x2, y2 = goal
    if method == "manhattan":
        return abs(x1 - x2) + abs(y1 - y2)
    elif method == "euclidean":
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# Định nghĩa các luật di chuyển (các ô lân cận)
def get_neighbors(position, grid):
    x, y = position
    neighbors = []
    # Di chuyển lên, phải, xuống, trái
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        # Kiểm tra nếu (nx, ny) là ô hợp lệ
        # 0 là ô trống
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
            neighbors.append((nx, ny))
    return neighbors


# Hàm triển khai thuật toán A*
def astar(start, goal, grid):
    # Hàng đợi ưu tiên để quản lý các ô sẽ thăm
    open_set = []
    # Đưa (chi phí, trạng thái) vào hàng đợi
    heapq.heappush(open_set, (0, start))

    # Tập hợp lưu trữ chi phí thấp nhất đã biết để đến từng ô
    g_costs = {start: 0}
    # Tập hợp lưu vết để theo dõi đường đi
    came_from = {}

    while open_set:
        # Lấy ô có chi phí f(n) thấp nhất
        current_priority, current = heapq.heappop(open_set)

        # Kiểm tra nếu đã đến trạng thái đích
        if current == goal:
            # Tạo đường đi từ trạng thái đích ngược lại trạng thái khởi đầu
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path  # Trả về đường đi

        # Lấy các ô lân cận
        for neighbor in get_neighbors(current, grid):
            # Tính g(n) của ô lân cận
            tentative_g_cost = g_costs[current] + \
                1  # Giả định chi phí di chuyển là 1

            # Nếu tìm thấy đường đi tốt hơn đến neighbor
            if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                came_from[neighbor] = current
                g_costs[neighbor] = tentative_g_cost
                f_cost = tentative_g_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_cost, neighbor))

    # Trả về None nếu không tìm được đường đi
    return None


def generate_matrix(rows, cols, obstacle_probability=0.3):
    matrix = []
    for i in range(rows):
        row = []
        for j in range(cols):
            if random.random() < obstacle_probability:
                row.append(1)  # Obstacle
            else:
                row.append(0)  # Free space
        matrix.append(row)
    return matrix


# Parameters for the matrix
num_rows = 5
num_cols = 5
obstacle_probability = 0.3  # 30% chance for obstacles

# Generate the matrix
matrix = generate_matrix(num_rows, num_cols, obstacle_probability)
print(matrix)
# Ví dụ sử dụng với một lưới ô vuông
# grid = [
#     [0, 0, 0, 0, 1],
#     [1, 1, 0, 1, 0],
#     [0, 0, 0, 0, 0],
#     [0, 1, 1, 1, 0],
#     [0, 0, 0, 0, 0],
# ]

start = (0, 0)
goal = (4, 4)

path = astar(start, goal, matrix)
print("Path:", path)
