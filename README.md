# Vehicle half-car suspension model

TODO

<a href="https://youtu.be/_CIp4ywYVUs"><img src="doc/img/result.png"/></a><br>
<a href="https://youtu.be/_CIp4ywYVUs">Watch video on YouTube</a>

```
git clone https://github.com/nrsyed/half-car.git
cd half-car
pip install -e .
python example.py
```

## Background

For a more detailed treatment of the theory, refer to 
<a href="https://nrsyed.com/2018/01/07/numerical-approach-to-studying-vehicle-dynamics-with-a-half-car-suspension-model/">
my original blog post</a>.

In the world of vehicle dynamics, a quarter-car suspension model considers a
single wheel, modeling both the suspension shock/strut and the tire as a
spring-mass-damper. This 1-dimensional, 2-DOF system consists of two differential
equations that can be solved to find the displacement of both the wheel
(the "unsprung mass") and the chassis (the "sprung mass") in response
to the input (the road).

<img src="doc/img/quarter_model.jpg" />

A half-car model extends this to two wheels and adds two degrees of freedom,
forming a 4-DOF system of four differential equations that can be solved for
the vertical displacements of the front wheel, rear wheel, and chassis, as well
as the pitch (angular displacement) of the chassis.

<img src="doc/img/half_model.jpg" width="50%" />

In this case, the input (the road) must be evaluated at two points. The system
and variables are shown below.

<img src="doc/img/accord_vars2.jpg" width="70%" />

To characterize the system dynamics, the normal forces on the wheels must also
be taken into account, or more specifically, the change in the normal forces
in response to horizontal acceleration, which is based on the wheelbase (the
distance between the wheels and the distance of each from the car's overall
center of gravity). This phenomenon is what causes a car to pitch back when
accelerating and pitch forward when braking.

<img src="doc/img/normal_forces.jpg" width="70%" />


