async function runSimulation(simData) {
  const response = await fetch("http://localhost:8080/simulate", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(simData)
  });
  const result = await response.json();
  console.log(result.snapshots); // trajectories
}
