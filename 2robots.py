import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import scipy.special

# def optimize_bezier_curve(initial_point, goal_point, Ax, b, m):
#     """
#     Optimizes a Bezier curve of degree m in a given polygon defined by Ax@x + b <= 0.

#     Parameters:
#     - initial_point: The starting point of the Bezier curve.
#     - goal_point: The ending point of the Bezier curve.
#     - Ax: The matrix defining the polygon constraints.
#     - b: The vector defining the polygon constraints.
#     - m: The degree of the Bezier curve.
#     -T : Time 

#     Returns:
#     - control_points: The optimized control points of the Bezier curve.
#     """
#     # Number of dimensions
#     dim = 4
    
#     # Centroid for reference (optional, can be replaced with another point)
    

#     # Create a new model
#     model = gp.Model("BezierCurveOptimization")

    
#     r = model.addMVar((m+1, 4), lb=-GRB.INFINITY, ub=GRB.INFINITY, name="r")
#     alpha = 2  # example value
   
#     # Add polygon constraints
#     for i in range(m):
#         model.addConstr(Ax[0, 0] * r[i, 0] + Ax[0, 1] * r[i, 1] + b[0] <= 0)
#         model.addConstr(Ax[1, 0] * r[i, 2] + Ax[1, 1] * r[i, 3] + b[1] <= 0)

#     # Add relationship constraints between control points
#     # for i in range(m):
#     #     model.addConstr(r[i+1, 0]-r[i, 0] == alpha * (r[i+1, 2]-r[i,2]))
#     #     model.addConstr(r[i+1, 1]-r[i, 1] == alpha * (r[i+1, 3]-r[i,3]))


    


#     # Initial and goal points constraints
#     for d in range(dim):
#         model.addConstr(r[0][d] == initial_point[d])
#         model.addConstr(r[m][d] == goal_point[d])
#         ###initial and final velocity is zero
#         model.addConstr(r[0][d] == r[1][d])
#         model.addConstr(r[m][d] == r[m-1][d])
        


    
#     # Objective function: minimize the squared distance to the centroid
#     objective = gp.QuadExpr()
#     for i in range(1, m-1):
#         for d in range(dim):
#             objective += (r[i][d]**2+r[i+1][d]**2-2*r[i+1][d]*r[i][d]) 
#     for i in range(m):
#         objective +gp.abs(r[i+1, 0]-r[i, 0] -alpha * (r[i+1, 2]-r[i,2]))
#         objective +=gp.abs(r[i+1, 1]-r[i, 1] -alpha * (r[i+1, 3]-r[i,3]))
#     model.setObjective(objective, GRB.MINIMIZE)

#     # Optimize the model
#     model.optimize()

#     # Extract the results
#     control_points_x = []
#     for i in range(m+1):
#         control_points_x.append([r[i][d].X for d in range(dim)])

#     return np.array(control_points_x)
import gurobipy as gp
from gurobipy import GRB
import numpy as np

def optimize_bezier_curve(initial_point, goal_point, Ax, b, m):
    """
    Optimizes a Bezier curve of degree m in a given polygon defined by Ax@x + b <= 0.

    Parameters:
    - initial_point: The starting point of the Bezier curve.
    - goal_point: The ending point of the Bezier curve.
    - Ax: The matrix defining the polygon constraints.
    - b: The vector defining the polygon constraints.
    - m: The degree of the Bezier curve.
    - T : Time 

    Returns:
    - control_points: The optimized control points of the Bezier curve.
    """
    # Number of dimensions
    dim = 4

    # Create a new model
    model = gp.Model("BezierCurveOptimization")

    r = model.addMVar((m+1, dim), lb=0, ub=1, name="r")
    alpha = 2  # example value

    # Add polygon constraints
    for i in range(1,m):
        model.addConstr(Ax[0, 0] * r[i, 0] + Ax[0, 1] * r[i, 1] + b[0] <= 0)
        model.addConstr(Ax[1, 0] * r[i, 2] + Ax[1, 1] * r[i, 3] + b[1] <= 0)

    # Initial and goal points constraints
    for d in range(dim):
        model.addConstr(r[0][d] == initial_point[d])
        model.addConstr(r[m][d] == goal_point[d])
        # Initial and final velocity is zero
        model.addConstr(r[0][d] == r[1][d])
        model.addConstr(r[m][d] == r[m-1][d])
    # Add relationship constraints between control points
    delta1 = model.addMVar((m, ), lb=-GRB.INFINITY, ub=GRB.INFINITY, name="abs_terms_x")
    delta2 = model.addMVar((m, ), lb=-GRB.INFINITY, ub=GRB.INFINITY, name="abs_terms_x")
    for i in range(m):
        model.addConstr(r[i+1, 0]-r[i, 0] == alpha * (r[i+1, 2]-r[i,2])+delta1[i])
        model.addConstr(r[i+1, 1]-r[i, 1] == alpha * (r[i+1, 3]-r[i,3])+delta2[i])
        # model.addConstr(r[i+1, 0]-r[i, 0] == alpha * (r[i+1, 2]-r[i,2]))
        # model.addConstr(r[i+1, 1]-r[i, 1] == alpha * (r[i+1, 3]-r[i,3]))


    # Auxiliary variables for absolute values
    abs_terms_x = model.addMVar((m, ), lb=0, name="abs_terms_x")
    abs_terms_y = model.addMVar((m, ), lb=0, name="abs_terms_y")

    for i in range(m):
        # Absolute value constraints for x direction
        model.addConstr(abs_terms_x[i] >= delta1[i])
        model.addConstr(abs_terms_x[i] >= -delta1[i])
        # Absolute value constraints for y direction
        model.addConstr(abs_terms_y[i] >= delta2[i])
        model.addConstr(abs_terms_y[i] >= -delta2[i])

    # Objective function: minimize the squared distance to the centroid
    # objective = gp.QuadExpr()
    # for i in range(1, m):
    #     for d in range(dim):
    #         objective += (r[i][d]**2 + r[i+1][d]**2 - 2*r[i+1][d]*r[i][d])
    objective = 0
    for i in range(m):
        objective += abs_terms_x[i] + abs_terms_y[i]
    model.setObjective(objective, GRB.MINIMIZE)

    # Optimize the model
    model.optimize()

    # Extract the results
    control_points_x = []
    for i in range(m+1):
        control_points_x.append([r[i][d].X for d in range(dim)])



    print(delta1.X)
    print(delta2.X)
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
    curve_points_1 = bezier_curve(control_points[:,0:2])
    curve_points_2 = bezier_curve(control_points[:,2:])
    # Plotting the Bezier curve
    plt.plot(curve_points_1[:, 0], curve_points_1[:, 1], label="Bezier Curve")
    plt.plot(curve_points_2[:, 0], curve_points_2[:, 1], label="Bezier Curve")
    # Plotting the control points and lines connecting them
    control_points = np.array(control_points)
    # plt.plot(control_points[:, 0], control_points[:, 1], 'ro-', label="Control Points")
    # plt.plot(control_points[:, 1], control_points[:, 2], 'ro-', label="Control Points")
    
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
initial_point = np.array([0.4, 0.1, 0.4,0.1])  # Example initial point inside the unit square
goal_point = np.array([0.4, 0.2,0.5,0.1])  # Example goal point inside the unit square
Ax = np.array([
    [-1, 0],
    [1, 0],
    [0, -1],
    [0, 1]
])
b = np.array([0, -1, 0, -1])
m = 200  # Degree of the Bezier curve

control_points = optimize_bezier_curve(initial_point, goal_point, Ax, b, m)
# print("Control Points:", control_points)

plot_bezier_curve(control_points)
