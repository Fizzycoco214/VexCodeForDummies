# PID
PID stands for:

**P**roportional

**I**ntegral

**D**erivative

These are the main variables you tweek to make the system behave the way you want it to. The variables are used to calculate our motor speed from our current error, but what is error?

# Error
Error in its simplest terms is our distance from the desired result. In a turning PID system, it's the difference of our current angle and our target angle. 
In a driving PID system, it can be either the distance from our current position to our target position or difference in rotation between one side of the drivetrain and the other.

In code, this roughly translates to:
```c++
float error = target - current;
```
It's very important that the error can be negative if we overshoot our target.

# Using the variables
Now that we have our error, it's now time to start calculating our final motor speed. To get this, we calculate all 3 types of error and then add them all together.
## P
P stands for proportional and to get value this it's pretty simply:
```c++
float pError = error * p;
```

75% of the time just using p is fine

## I
I stands for integral which we can achieve in code by adding summing up our error each time we go through the loop.
In code this translates to:
```c++
float integralSum = 0;
while(true)
{
  integralSum += error;
  float iError = integralSum * i;
}
```
This is one of the reasons it's importatant that our error can be negative, because if not, this integral sum will shoot off to infinity and your bot will just be left spinning in one direction

I is mostly used for equaling out the sides of a drivetrain as it fixes steady state error. Only use it if your drivetrain is like a bad shopping cart
and drifts left or right while driving forward.

## D
D stands for derivative, or the change in error between loops

In code this is:
```c++
float lastError = 0;
while(true)
{
  float dError = (error - lastError) * d;
  lastError = error;
}
```

This is mostly used to fix oscellation, where the PID system never settles and just wobbles around the target.

# Combining all of this together
Now that we know what each variable does we can combine all of this into this block of code:
```c++
float error = target - current;
float integralSum = 0;
float lastError = error;
while(true)
{
  error = target - current;
  integralSum += error;
  
  float finalValue = (p * error) + (i * integralSum) + (d * (error - lastError));
  lastError = error;
}
```

#Tuning the variables
There's not an exact science to tuning the variables but I do have a general process:
- Start at around p = 0.3, i = 0, d = 0.1 
- See how the system does with these variables
-   If it's too slow increase P
-   If there's drift, increase I
-   If it's oscelating, either increase D or decrease P
- Tweak the variables and see how it does with your new varibles
For heavier robots, a P value around ~0.15 is good to prevent oscelation due to the weight

For lighter, 6 motor drives, I've found that a P value of ~0.4 is a good value since it can stop it's self faster
