// Define variables to reference HTML elements
const problemSelect = document.getElementById('problemSelect');
const epsilonRange = document.getElementById('epsilonRange');
const epsilonValue = document.getElementById('epsilonValue');
const calculateButton = document.getElementById('calculateButton');
const resultDiv = document.getElementById('result');

// Fetch available problems from the server and populate the dropdown/select element
fetch('/get_problems')
    .then(response => response.json())
    .then(problemNames => {
        problemNames.forEach(problemName => {
            const option = document.createElement('option');
            option.value = problemName;
            option.textContent = problemName;
            problemSelect.appendChild(option);
        });
    });

// Update the epsilon value displayed dynamically as the slider/input changes
epsilonRange.addEventListener('input', () => {
    epsilonValue.textContent = epsilonRange.value;
});

// Handle the click event on the "Calculate Shortest Path" button
calculateButton.addEventListener('click', () => {
    // Retrieve the selected problem and epsilon value
    const problem = problemSelect.value;
    const epsilon = epsilonRange.value;

    // Make a POST request to the server to calculate the shortest path and get the Plotly figure data
    fetch('/calculate_shortest_path', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ problem, epsilon })
    })
    .then(response => response.json())
    .then(data => {
        // resultDiv.textContent = `Shortest path: ${data.result}, Execution time: ${data.execution_time} seconds`;
        resultDiv.textContent = `Execution time: ${data.execution_time} seconds`;

        const figData = JSON.parse(data.fig_data);

        Plotly.newPlot('plotlyDiv', figData);
    });
});

