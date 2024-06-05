import gurobipy as gp
from gurobipy import GRB
import numpy as np
from scipy.special import binom

# Degree of the Bezier curve
m = 3  # Change this as needed

# Polygon constraints Ax + b < 0
A = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])
b = np.array([1, 1, 1, 1])  # Example polygon

# Initial and goal points
start = np.array([0, 0])
goal = np.array([1, 1])

# Create a new model
model = gp.Model("BezierOptimization")

# Control points as decision variables
P = model.addVars(m + 1, 2, lb=-GRB.INFINITY, name="P")

# s_f as a decision variable
s_f = model.addVar(lb=0, name="s_f")

# Objective function: minimize the square distance of control points plus minimizing s_f
objective = gp.quicksum((P[i, 0] - start[0])**2 + (P[i, 1] - start[1])**2 for i in range(m + 1)) + s_f
model.setObjective(objective, GRB.MINIMIZE)

# Polygon constraints
for i in range(m + 1):
    for j in range(A.shape[0]):
        model.addConstr(A[j, 0] * P[i, 0] + A[j, 1] * P[i, 1] + b[j] <= 0)

# Start point constraints
model.addConstr(P[0, 0] == start[0])
model.addConstr(P[0, 1] == start[1])

# End point constraints (goal at t = s_f)
# For Bezier curve r(t) at t = s_f:
r(s_f) = sum_{i=0}^m binom(m, i) * (s_f / s_f)^i * (1 - s_f / s_f)^(m - i) * P_i
# Simplifies to r(s_f) = P[m]

model.addConstr(P[m, 0] == goal[0])
model.addConstr(P[m, 1] == goal[1])

# Zero velocity constraints at t = 0
model.addConstr(m * (P[1, 0] - P[0, 0]) == 0)
model.addConstr(m * (P[1, 1] - P[0, 1]) == 0)

# Zero velocity constraints at t = s_f
model.addConstr(m * (P[m, 0] - P[m-1, 0]) / s_f == 0)
model.addConstr(m * (P[m, 1] - P[m-1, 1]) / s_f == 0)

# Optimize the model
model.optimize()

# Print the results
if model.status == GRB.OPTIMAL:
    control_points = np.array([[P[i, j].X for j in range(2)] for i in range(m + 1)])
    final_s_f = s_f.X
    print("Optimal control points:")
    print(control_points)
    print(f"Optimal s_f: {final_s_f}")
else:
    print("No optimal solution found.")
