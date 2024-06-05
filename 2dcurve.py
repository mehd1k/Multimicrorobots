import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import scipy.special

def optimize_bezier_curve(initial_point, goal_point, Ax, b, m):
    """
    Optimizes a Bezier curve of degree m in a given polygon defined by Ax@x + b <= 0.

    Parameters:
    - initial_point: The starting point of the Bezier curve.
    - goal_point: The ending point of the Bezier curve.
    - Ax: The matrix defining the polygon constraints.
    - b: The vector defining the polygon constraints.
    - m: The degree of the Bezier curve.
    -T : Time 

    Returns:
    - control_points: The optimized control points of the Bezier curve.
    """
    # Number of dimensions
    dim = len(initial_point)
    
    # Centroid for reference (optional, can be replaced with another point)
    centroid = (initial_point + goal_point) / 2

    # Create a new model
    model = gp.Model("BezierCurveOptimization")
    T_max = 1
    v_max = 5
    v_min = 2
    T = model.addVar(lb = 0, ub = T_max, name = 'Time')
    # Add variables for the control points
    r = []
    for i in range(m+1):
        r.append(model.addMVar(dim, lb=-GRB.INFINITY, name=f"r_{i}"))

    ####Velocity Constraints
    h = []
    for i in range(m+1):
        h.append(model.addMVar(dim, lb=-GRB.INFINITY, name=f"h_{i}"))
    ###Boundry Condition on h
    for d in range(dim):
        model.addConstr(h[0][d] == 0)
        model.addConstr(h[m][d] == T)
    ###Velocity Constraints
    for i in range(m-1):
        model.addConstr(m*(r[i+1]-r[i])<=v_max*m*(h[i+1]-h[i]))
        model.addConstr(m*(r[i+1]-r[i])>=v_min*m*(h[i+1]-h[i]))


    ###Constraint for monotonic increasing of h (dot(h)>0.) 
    for i in range(m):
        model.addConstr(m*h[i+1]-m*h[i]>= 10**-6)

    # Initial and goal points constraints
    for d in range(dim):
        model.addConstr(r[0][d] == initial_point[d])
        model.addConstr(r[m][d] == goal_point[d])
        ###initial and final velocity is zero
        model.addConstr(r[0][d] == r[1][d])
        model.addConstr(r[m][d] == r[m-1][d])

    # Polygon constraints
    for i in range(1, m):
        for j in range(len(Ax)):
            model.addConstr(gp.quicksum(Ax[j, d] * r[i][d] for d in range(dim)) + b[j] <= 0)

    # Objective function: minimize the squared distance to the centroid
    objective = gp.QuadExpr()
    for i in range(1, m-1):
        for d in range(dim):
            objective += (r[i][d]**2+r[i+1][d]**2-2*r[i+1][d]*r[i][d]) 

    model.setObjective(T+objective, GRB.MINIMIZE)

    # Optimize the model
    model.optimize()

    # Extract the results
    control_points_x = []
    for i in range(m+1):
        control_points_x.append([r[i][d].X for d in range(dim)])

    return np.array(control_points_x)

def bezier_curve(control_points, num_points=100):
    """
    Generates a Bezier curve from control points.

    Parameters:
    - control_points: List of control points, where each point is a tuple or list.
    - num_points: Number of points to generate on the curve.

    Returns:
    - List of points on the Bezier curve.
    """
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    curve_points = []

    for t in t_values:
        point = np.zeros(len(control_points[0]))
        for i, P in enumerate(control_points):
            bernstein_poly = scipy.special.comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
            point += bernstein_poly * np.array(P)
        curve_points.append(point)
    
    return np.array(curve_points)

def plot_bezier_curve(control_points):
    """
    Plots a Bezier curve given the control points.

    Parameters:
    - control_points: List of control points, where each point is a tuple or list.
    """
    curve_points = bezier_curve(control_points)
    
    # Plotting the Bezier curve
    plt.plot(curve_points[:, 0], curve_points[:, 1], label="Bezier Curve")
    
    # Plotting the control points and lines connecting them
    control_points = np.array(control_points)
    plt.plot(control_points[:, 0], control_points[:, 1], 'ro-', label="Control Points")
    
    # Setting plot limits
    plt.xlim(-0.1, 1.1)
    plt.ylim(-0.1, 1.1)
    
    # Adding labels and legend
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Bezier Curve")
    plt.legend()
    plt.grid(True)
    
    # Show plot
    plt.show()

# Example usage with the control points found from the optimization
initial_point = np.array([0.1, 0.1])  # Example initial point inside the unit square
goal_point = np.array([0.9, 0.9])  # Example goal point inside the unit square
Ax = np.array([
    [-1, 0],
    [1, 0],
    [0, -1],
    [0, 1]
])
b = np.array([0, -1, 0, -1])
m = 5  # Degree of the Bezier curve

control_points = optimize_bezier_curve(initial_point, goal_point, Ax, b, m)
print("Control Points:", control_points)

plot_bezier_curve(control_points)
