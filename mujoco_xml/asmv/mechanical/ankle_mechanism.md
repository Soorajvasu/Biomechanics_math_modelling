## Goal 

1. convert ankle pitch and roll to movements motor A and B
2. convert A and B motor movements to ankle pitch and roll

## Background 

We have a 2 DOF ankle that is NOT direct driven. Instead, the pitch and roll is controlled by manipulating 2 end points on a straight bar. These 2 end points are connected by parallel linkages to motors A and B. 

We treat the direction towards the ground as `-y` and away from the ground as `+y`. 

### PR -> AB derivation

We need to map the `y` displacement of the left and right most points on the bar to the rotation of the motors. 

Assuming we are looking at the right leg, the left most point is attached to motor B and the right most point is attached to motor A.

`y` displacement of points A and B can be treated as a summation of displacement caused by a pitch movement and a roll movement. 

```r
y_A = y_p + y_Ar
y_B = y_p + y_Br

where:
y_A is the vertical displacement at point A
y_B is the veritcal displacement at point B
y_p is the vertical displacement due to a pitch movement
y_Ar is the vertical displacement due to a roll movement on point A
y_Br is the vertical displacement due to a roll movement on point B
```

Observing the movement of the bar relative to the pivot point at the ankle, we can define `y_p` as:

```r
y_p = d * tan(theta_p)

where:
y_p is the vertical displacement due to a pitch movement
d is the distance between the pivot point and the bar
theta_p is the pitch rotation
```

Observing the bar from the front, a roll movement in -ve results in point B moving down and point A moving up. The opposite is true for a +ve roll movement, with point B moving up and point A moving down. As such we can define `y_Ar` and `y_Br` as:

```r
y_Ar = - (c / 2) * sin(theta_r)
y_Br =   (c / 2) * sin(theta_r)

where: 
c is the length of the bar
theta_r is the roll rotation
```

The final piece is to define how the rotations of motor A and B affect the height of the linkages which attach to the bar:

```r
y_A =   r_A * sin(theta_A)
y_B = - r_B * sin(theta_B)

where:
r_A is the radius of the circular mount on motor A that attaches to the linkage
r_B is the radius of the circular mount on motor B that attaches to the linkage
theta_A is the rotation of motor A
theta_B is the rotation of motor B
```

Combining it all together:

```r
r_A * sin(theta_A) = d * tan(theta_p) - (c / 2) * sin(theta_r)
sin(theta_A) = (d / r_A) * tan(theta_p) - (c / (2 * r_A)) * sin(theta_r)

- r_B * sin(theta_B) = d * tan(theta_p) + (c / 2) * sin(theta_r)
sin(theta_B) = - (d / r_B) * tan(theta_p) - (c / (2 * r_B)) * sin(theta_r)
```

Applying the small angle approximation:

```r
theta_A =   K_P * theta_p - K_R * theta_r
theta_B = - K_P * theta_p - K_R * theta_r

where:
K_P is (d / r), r = r_A = r_B
K_R is (c / (2 * r)), r = r_A = r_B
```

Code example:

```cpp
float right_A = ANKLE_COUPLING_K_PITCH * right_pitch - ANKLE_COUPLING_K_ROLL * right_roll;
float right_B = -ANKLE_COUPLING_K_PITCH * right_pitch - ANKLE_COUPLING_K_ROLL * right_roll;

float left_A = ANKLE_COUPLING_K_PITCH * left_pitch - ANKLE_COUPLING_K_ROLL * left_roll;
float left_B = -ANKLE_COUPLING_K_PITCH * left_pitch - ANKLE_COUPLING_K_ROLL * left_roll;
```

Due to the mirrored rotation axis on the pitch ankle (view URDF) and the asymmetrical linkages, the equations for PR -> AB for the same for both legs.

### AB -> PR derivation

Due to the mechanism mapping 2 rotations to a single rotation on pitch/roll or a combination of the two, we can't take the inverse of the PR -> AB derivation. Instead, we will calculate the forward kinematics from AB -> PR directly. 

Starting from the right leg again, we can define the pitch rotation as middle point of the bar relative to the pivot point:

```r
tan(theta_p) = y_mid / d
tan(theta_p) = (y_A + y_B) / (2 * d) where y_mid = (y_A + y_B) / 2

Expanding and simplifying: 
tan(theta_p) = (r_A * sin(theta_A) - r_B * sin(theta_B)) / (2 * d)
```

Taking a page out of the PR -> AB mapping, we can define the vertical displacement due to ankle roll as the veritcal position of the linkage relative to the midpoint of the bar:

```r
y_Ar = y_A - y_mid

where:
y_Ar = - (c / 2) * sin(theta_r)
y_A = r_A * sin(theta_A)
y_mid = (y_A + y_B) / 2
```

With some algebra, we can get the ankle roll mapping:

```
y_A - ((y_A + y_B) / 2) = - (c / 2) * sin(theta_r)
((y_A + y_B) / 2) - y_A = (c / 2) * sin(theta_r)
(((y_A + y_B) / 2) - y_A) * (2 / c) = sin(theta_r)
sin(theta_r) = (-0.5y_A + 0.5y_B) * (2 / c)
sin(theta_r) = (-0.5 * r_A * sin(theta_A) + 0.5 * - r_B * sin(theta_B)) * (2 / c)

Expanding and simplifying:
sin(theta_r) = - (1 / c) * (r_A * sin(theta_A) + r_B * sin(theta_B))
```

Applying small angle approximations and assuming r = r_A = r_B:

```r
theta_p = (r_A * theta_A - r_B * theta_B) / (2 * d)
theta_p = (theta_A - theta_B) * (r / (2 * d))

theta_r = - (1 / c) * (r_A * theta_A + r_B * theta_B)
theta_r = (theta_A + theta_B) * (r / c)
```

Code example:

```
float right_pitch = (right_A - right_B) / (2.0f * K_PITCH);
float right_roll =  -(right_A + right_B) / (2.0f * K_ROLL);

float left_pitch = (left_A - left_B) / (2.0f * K_PITCH);
float left_roll =  -(left_A + left_B) / (2.0f * K_ROLL);
```

