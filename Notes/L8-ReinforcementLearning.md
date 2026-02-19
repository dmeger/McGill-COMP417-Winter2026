# Reinforcement Learning

While the previous section described very useful tools to understand and control robots, we can note that there was no "learning" happening. We didn't need to collect any data and we did require full model knowledge: that is, the MDP state space, action space, transition function and reward function were needed as inputs to Iterative Policy Evaluation and Value Iteration.

Learning in this type of system means trial-and-error: making behaviors, observing their outcomes and using the generated data to solve for components such as the Value function or policy. 