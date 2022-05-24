# Recruitment test
A pdf report is expected to be given to your contact person when the test is completed.

- Follow the guidelines to install [bioptim](https://github.com/pyomeca/bioptim) on your computer.
- Download the whole repository and run the script recruitment_script.py on your computer. What is the error message displayed in your console.
- A bug is hidden in the script find it, modify it, to make the script work, explain why it didn't work in one or two sentences.
- An Objective function was set in the optimal control problem. What is it ?
- Modify the objective function to minimize generalized velocities `qdot` instead. What has changed with this new cost function ?
- The result of the simulation can be found in `sol`. 
You can find the values of the angle and translation of the system in `sol.states['q']` and torques in `sol.controls['tau']`.
Modify the code to plot them on one figure in the clearest way.
- Summarize in three sentences what the simulation did.
