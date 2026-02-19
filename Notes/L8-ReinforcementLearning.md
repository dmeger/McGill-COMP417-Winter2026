# Reinforcement Learning

While the previous section described very useful tools to understand and control robots, we can note that there was no "learning" happening. We didn't need to collect any data and we did require full model knowledge: that is, the MDP state space, action space, transition function and reward function were needed as inputs to Iterative Policy Evaluation and Value Iteration.

Learning in this type of system means trial-and-error: making behaviors, observing their outcomes and using the generated data to solve for components such as the Value function or policy. The data that a Reinforcement Learner would receive are tuples (s,a,r,s'), where the $a=\pi(s)$ for some behavior policy that was used to collect the data. This may be the same, or may be different from the learner's current best guess at the optimal behavior currently, for reasons of computation or exploration. This distinction makes learners:
- On-policy: when the data they learn from is drawn such that $a=\pi(s)$ with the current $\pi$ under consideration, or
- Off-policy: when the actions in the data can be from a different $\pi$.

## Q-Learning Off Policy RL for Discrete State/Action

(NOTE, this document is being actively typed and added to. Refresh in a few days to find more content.)