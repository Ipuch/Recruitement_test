# Recruitment test
A pdf report is expected to be given to your contact person when the test is completed.

- Follow the guidelines to install [bioptim](https://github.com/pyomeca/bioptim) on your computer.
- Download the whole repository and run the script recruitment_script.py on your computer. What is the error message displayed in your console.
- A bug is hidden in the script find it, modify it, to make the script work, explain why it didn't work in one or two sentences. (hint: the file bioptim/misc/enums.py will be useful)
- Modify the code to add a second phase where the objective is to maximize the angular velocity of the pendulum after 1 second. (hint: the file bioptim/examples/getting_started/example_multiphase.py will be useful, maximizing can be done using weight=-1 and the angular velocity is the second DoF so you can specify it using index=1)
- Summarize in few sentences what the simulation did (hint: use the graphics generated with sol.graph() to support your explanation).
- You can find the values of the angular velocity in the second component of `sol.states['qdot']`, modify the code to plot it in the clearest way possible.