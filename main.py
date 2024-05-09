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

app = Flask(__name__)

# Define a dictionary to map problem names to their corresponding problem objects
problems = {
    "problemSimple": problemSimple,
    "problem1": problem1,
    "problem2": problem2,
    "problem3": problem3,
    "problem4": problem4,
}


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/get_problems")
def get_problems():
    # Extract problem names from the dictionary keys
    problem_names = list(problems.keys())
    return jsonify(problem_names)


@app.route("/visual_display")
def visual_display():
    # Call the main function to get problem data and shortest path
    # Replace 'problem1' with the actual problem you want to visualize
    problem = problem1
    epsilon = 1
    result = main(problem, epsilon)
    path = [(x, y) for x, y, _ in result]

    # Pass problem data and shortest path to the HTML template
    return render_template(
        "visual_display.html", problemData=problem.__dict__, pathData={"result": path}
    )


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
        return jsonify(
            {"result": formattedPathString, "execution_time": execution_time}
        )
    else:
        return jsonify({"error": "Problem not found"})


if __name__ == "__main__":
    app.run(debug=True)
