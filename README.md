

**Algorithm 2 Motion Planning in Free Space**
**Input:** Start pose \$q\_0=(x\_0,y\_0,\theta\_0)\$, Goal pose \$q\_g=(x\_g,y\_g,\theta\_g)\$, Map (empty costmap)
**Output:** Dynamically feasible and smooth trajectory \$(x(t),y(t),\theta(t))\$

1. **Step 1. Global path planning (A\* in free space)**
   1.1 Construct empty occupancy grid from costmap.
   1.2 Run A\* search between \$q\_0\$ and \$q\_g\$.
   1.3 Obtain discrete waypoints \$P={p\_0,\dots,p\_N}\$.

2. **Step 2. Path smoothing**
   2.1 Fit spline/clothoid curve through waypoints: \$(x(s),y(s)),;s\in\[0,L]\$.
   2.2 Enforce curvature constraint: if \$|\kappa(s)|>\kappa\_{\max}\$, reparameterize with additional control points.

3. **Step 3. Trajectory parameterization**
   3.1 Compute orientation along path:
   \$
   \theta(s)=\operatorname{atan2}(y'(s),x'(s))
   \$
   3.2 Compute curvature:
   \$
   \kappa(s)=\frac{x'(s)y''(s)-y'(s)x''(s)}{(x'(s)^2+y'(s)^2)^{3/2}}
   \$
   3.3 Assign speed profile:
   \$
   v(s) \le \min(v\_{\max}, \omega\_{\max}/|\kappa(s)|)
   \$.

4. **Step 4. Time-scaling for feasibility**
   4.1 Run forward pass to enforce acceleration limit: $|\dot v|\le a_{\max}$.
   4.2 Run backward pass to ensure braking feasibility near goal.
   4.3 Output feasible time-parameterized trajectory \$(x(t),y(t),\theta(t))\$.

5. **Step 5. Local control via DWA**
   5.1 At each control cycle, compute dynamic window:
   \$
   W={(v,\omega)\mid v\in\[v\_c-a\_{\max}\Delta t,;v\_c+a\_{\max}\Delta t],;\omega\in\[\omega\_c-\alpha\_{\max}\Delta t,;\omega\_c+\alpha\_{\max}\Delta t]}
   \$.
   5.2 Sample candidate \$(v\_i,\omega\_j)\in W\$.
   5.3 Simulate short horizon trajectory for each candidate.
   5.4 Evaluate score:
   \$
   \text{score}(v,\omega)=w\_{\text{goal}}\cdot heading+w\_{\text{vel}}\cdot\frac{v}{v\_{\max}}+w\_{\text{path}}\cdot(-path\_dist)
   \$.
   5.5 Select control pair maximizing score.

6. **Step 6. Execute trajectory**
   6.1 Send selected \$(v,\omega)\$ to robot actuators.
   6.2 Repeat Step 5 until goal \$q\_g\$ reached.





