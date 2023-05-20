# 2D Collision Avoidance Trajectory Planning Using MI-SOCP
An unofficial python implementation of JGCD artical:

[Zhang, Guoxu, and Xinfu Liu. "UAV Collision Avoidance Using Mixed-Integer Second-Order Cone Programming." Journal of Guidance, Control, and Dynamics 45.9 (2022): 1732-1738.](https://www.researchgate.net/publication/360330736_UAV_Collision_Avoidance_Using_Mixed-Integer_Second-Order_Cone_Programming)

## Requirements
-----
Python >= 3.7, numpy, cvxpy, matplotlib, yacs

## Running
-----
You can modify configuration in `config.py`.

To run with Alg 1
```bash
python solve --alg 1
```

## Tips for Coding
-----
To implement the constrain of 
$$
\delta \geq \sqrt{1 + \vartheta^2}
$$

You don't want to write the code just like the equation above, because in this case DCP rules are not able to verify convexity. [[Ref]](https://dcp.stanford.edu/rules)

Instead, do
```python
generalized_inequality_constraint = [delta[i] >= cp.norm(1 + vartheta[i]) for i in range(N)]
```

-----
Disciplined Parametrized Programming (DPP) forbids taking the product of two parametrized expressions like:
```python
last_delta = Parameter(N)
last_delta.value = delta.value
control_constraint = [
    cp.abs(u[i]) <= UAV_ANGULAR_RATE_MAX / V * (3 * last_delta[i]**2 * delta[i] - 2 * last_delta[i]**3) for i in range(N)
]
...
while not converged:
    problem.solve(solver=cp.ECOS_BB, mi_max_iters=1)
    last_delta.value = delta.value
```
This will result in recompiling of the problem in every `problem.solve`.

We need to do the power outside of the DSL [[Ref]](https://www.cvxpy.org/tutorial/advanced/index.html#disciplined-parametrized-programming).

```python
last_delta_squred = Parameter(N)
last_delta_cubic = Parameter(N)
last_delta_squred.value = np.power(delta.value, 2)
last_delta_cubic.value = np.power(delta.value, 3)
control_constraint = [
    cp.abs(u[i]) <= UAV_ANGULAR_RATE_MAX / V * (3 * last_delta_squred[i] * delta[i] - 2 * last_delta_cubic[i]) for i in range(N)
]
...
while not converged:
    problem.solve(solver=cp.ECOS_BB, mi_max_iters=1)
    last_delta_squred.value = np.power(delta.value, 2)
    last_delta_cubic.value = np.power(delta.value, 3)
```