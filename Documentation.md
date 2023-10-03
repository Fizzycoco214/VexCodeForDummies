# Utility Functions

```c++
signOf(float num)
```
- Returns the 1 if num >= 0, returns -1 otherwise
  
***
  
```c++
min(float x, float y)
```
- Returns the lesser of x or y
  
***
  
```c++
max(float x, float y)
```
- Returns the greater of x or y

***

```c++
clamp(float val, float min, float max)
```
- Returns val if it is within the bounds of min and max
- Returns min if val is lesser than min
- Returns max if val is greater than max
- **Example**
  - ```float percentage = clamp(num, 0, 100); ```

***

```c++
wrap(float val, float lowerBound, float upperBound, float increment)
```
- Essentially `%` that works for both positive and negative numbers
- Returns a value that's been incremented by increment to be within the lower and upper bounds
- **Example**
  - ```float angle = wrap(num, 0, 360, 180); ```

***

```c++
inBounds(float val, float lowerBound, float upperBound)
```
- Returns true if val is within the lower and upper bound

***

```c++
degToRad(float deg)
```
- Converts degrees to radians

***

```c++
radToDeg(float rad)
```
- Converts radians to degrees

***

```c++
wrap360(float val, ::rotationUnit unit)
```
- Wraps an angle of a specified unit from 0 to 360
  - See `wrap()`

***

```c++
wrapNegative180To180(float val, ::rotationUnit unit)
```
- Wraps an angle of a specified unit from -180 to 180
  - See `wrap()`

***

```c++
angleDifference(float angle1, float angle2, ::rotationUnit unit)
```
- Returns the difference between two angles of a specified unit
- **Example**
  - `float amountToTurn = angleDifference(currentAngle, TargetAngle, degrees);`

***

```c++
lerp(float val1, float val2, float t)
```
- Linearly interpolates between val1 and val2 using t
  - If t = 0, val1 is returned
  - If t = 1, val2 is returned
  - If t = 0.5, a value halfway between val1 and val2 is returned
- Useful for smoothly moving between two values
- **Example**
```c++
float value = 0;
while(value <= 0.99)
{
  value = lerp(value, 1, 0.5); //Keeps moving half of the distance remaining to 1
  wait(1,seconds);
}
```

# Vector2

A Vector2 is a vector with 2 axis (x and y). It's most commonly used for field posisitions but can also be used for angles and directions.
It mostly works like any other data type.

For directions, vectors have a direction and a length or magnitude


## Constructors
```c++
Vector2()

Vector2(float x, float y)
```

**Examples**
- `Vector2 targetPosition;`
- `Vector2 position = Vector2(120,-400);`
- `float currentX = position.x;`
- `float currentY = position.y;`

## Vector2 Functions
```c++
angleToVector(float angle)
```
-Converts an angle in degrees into a vector

***

```c++
vectorToAngle(Vector2 vector)
```
-Converts a vector into an angle in degrees

```c++
Distance(Vector2 p1, Vector2 p2)
```
-Returns the distance between two points

***

```c++
lerp(Vector2 v1, Vector2 v2, float t)
```
-Linearly interpolates between two points, see [Lerp](https://github.com/Fizzycoco214/SpinUpLibrary/blob/main/Documentation.md#utility-functions)

***

```c++
inBounds(Vector2 val, Vector2 p1, Vector2 p2)
```
-Returns true if val is within the bounds of p1 and p2, see [inBounds](https://github.com/Fizzycoco214/SpinUpLibrary/blob/main/Documentation.md#utility-functions)
