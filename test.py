import gurobipy as gp
from gurobipy import GRB
import numpy as np

# Define the degree of the Bezier curve
m = 20  # example degree

# Define the number of control points
n = m + 1

# Define the initial and goal points
start = np.array([0.1, 0.1, 0.1, 0.1])  # example start point
goal = np.array([0.5, 0.5, 0.5, 0.5])   # example goal point

# Define the polygon constraints
A = np.array([[1, 0], [0, 1]])  # example polygon coefficients
b = np.array([1, 1])  # example polygon offsets

# Create a Gurobi model
model = gp.Model("BezierCurveOptimization")

# Define the control points as variables
r = model.addVars(n, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, name="r")

# Define the alpha variable (can be fixed or another variable depending on the problem)
alpha = 0.5  # example value

# Add polygon constraints
# for i in range(n):
#     model.addConstr(A[0, 0] * r[i, 0] + A[0, 1] * r[i, 1] + b[0] <= 0)
#     model.addConstr(A[1, 0] * r[i, 2] + A[1, 1] * r[i, 3] + b[1] <= 0)

# Add relationship constraints between control points
for i in range(n):
    model.addConstr(r[i, 0] == alpha * r[i, 2])
    model.addConstr(r[i, 1] == alpha * r[i, 3])

# Add boundary conditions (initial and final points with zero velocity)
for j in range(4):
    model.addConstr(r[0, j] == start[j])
    model.addConstr(r[n-1, j] == goal[j])

# Add zero velocity constraints (first derivative at the boundaries)
# This can be done by ensuring the first and second control points are the same
# and the last two control points are the same
for j in range(4):
    model.addConstr(r[1, j] == r[0, j])
    model.addConstr(r[n-2, j] == r[n-1, j])

# Define the objective function (minimize the square distance of weight points)
objective = gp.quicksum((r[i, j] - r[i+1, j])*(r[i, j] - r[i+1, j]) for i in range(n-1) for j in range(4))

model.setObjective(objective, GRB.MINIMIZE)

# Optimize the model
model.optimize()

# Retrieve and print the optimized control points
if model.status == GRB.OPTIMAL:
    optimized_control_points = np.array([[r[i, j].X for j in range(4)] for i in range(n)])
    print("Optimized control points:")
    print(optimized_control_points)
else:
    print("No optimal solution found.")
