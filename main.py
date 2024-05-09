from flask import Flask, render_template, jsonify, request
from problems import (
    problemSimple,
    problem1,
    problem2,
    problem3,
    problem4,
    problemParameters,
)
from sweepVisGraph import main
import time
import plotly.graph_objects as go

app = Flask(__name__)

# Define a dictionary to map problem names to their corresponding problem objects
problems = {
    "problemSimple": problemSimple,
    "problem1": problem1,
    "problem2": problem2,
    "problem3": problem3,
    "problem4": problem4,
}


def plotPointsAndObstacles(start, goal, obstacles, shortestPath=None):
    """Generate a Plotly figure for points and obstacles"""
    fig = go.Figure()

    # Plot obstacles
    for obs in obstacles:
        x_obs, y_obs = zip(*obs)
        fig.add_trace(
            go.Scatter(
                x=x_obs,
                y=y_obs,
                fill="toself",
                fillcolor="rgba(128,128,128,0.5)",
                line=dict(color="rgba(128,128,128,0.5)"),
                showlegend=False,
            )
        )
        for vertex in obs:
            fig.add_trace(
                go.Scatter(
                    x=[vertex[0]],
                    y=[vertex[1]],
                    mode="markers",
                    marker=dict(color="black"),
                    showlegend=False,
                )
            )  # Plot obstacle vertices

    # Plot start and goal
    fig.add_trace(
        go.Scatter(
            x=[start[0]],
            y=[start[1]],
            mode="markers",
            marker=dict(color="red"),
            name="Start",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=[goal[0]],
            y=[goal[1]],
            mode="markers",
            marker=dict(color="blue"),
            name="Goal",
        )
    )

    if shortestPath:
        path = [(x, y) for x, y, _ in shortestPath]
        path_x, path_y = zip(*path)
        fig.add_trace(
            go.Scatter(
                x=path_x,
                y=path_y,
                mode="lines",
                line=dict(color="red"),
                name="Shortest Path",
            )
        )

    # Update layout
    fig.update_layout(
        xaxis=dict(title="X"),
        yaxis=dict(title="Y"),
        title="Graph with Obstacles",
        showlegend=True,
        hovermode="closest",
        plot_bgcolor="white",
    )

    return fig


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/get_problems")
def get_problems():
    # Extract problem names from the dictionary keys
    problem_names = list(problems.keys())
    return jsonify(problem_names)


@app.route("/calculate_shortest_path", methods=["POST"])
def calculate_shortest_path():
    problem_name = request.json["problem"]
    epsilon = float(request.json["epsilon"])

    # Retrieve the problem object based on the selected problem name
    problem = problems.get(problem_name)

    if problem:
        # Calculate the shortest path for the selected problem
        start_time = time.time()
        result = main(problem, epsilon)
        path = [(x, y) for x, y, _ in result]
        formattedPathString = ", ".join([str(coord) for coord in path])
        end_time = time.time()
        execution_time = end_time - start_time

        # Generate Plotly figure
        fig = plotPointsAndObstacles(
            problem.start, problem.goal, problem.obstacles, result
        )

        # Convert Plotly figure to JSON format
        fig_json = fig.to_json()

        return jsonify(
            {
                "result": formattedPathString,
                "execution_time": execution_time,
                "fig_data": fig_json,  # Pass Plotly figure data
            }
        )
    else:
        return jsonify({"error": "Problem not found"})


if __name__ == "__main__":
    app.run(debug=True)
