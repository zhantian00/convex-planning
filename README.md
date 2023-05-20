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
