


#include "vex.h"
#include <vector>
 
/*
#########################
    Utility Functions
#########################
*/

float PI = 3.14159265359;

enum rotationUnits
{
    Degrees, Radians
};

int signOf(float num)
{
  if(num == 0){return 1;}
  return abs(num) / num;
}

float min(float x, float y)
{
    if(x < y)
    {
        return x;
    }
    return y;
}

float max(float x, float y)
{
    if(x > y)
    {
        return x;
    }
    return y;
}

float clamp(float val, float min, float max)
{
    if(val < min){return min;}
    if(val > max){return max;}
    return val;
}

float wrap(float val, float lowerBound, float upperBound, float increment)
{
  while(val > upperBound || val < lowerBound)
  {
      if(val > upperBound){val -= increment;}
      if(val < lowerBound){val += increment;}
  }
  return val;
}

bool inBounds(float val, float lowerBound, float upperBound)
{
  return (lowerBound < val && val < upperBound);
}


float degToRad(float deg){return deg * (PI / 180);}

float radToDeg(float rad){return rad * (180 / PI);}

float wrap360(float val, ::rotationUnits unit)
{
    if(unit == Radians){val = radToDeg(val);}

    float wrapped = wrap(val, 0, 360, 360);

    if(unit == Radians){val = degToRad(val);}

    return val;
}

float wrapNegative180To180(float val, ::rotationUnits unit)
{
    if(unit == Radians){val = radToDeg(val);}

    float wrapped = wrap(val, -180, 180, 180);

    if(unit == Radians){val = degToRad(val);}

    return val;
}

float angleDifference(float angle1, float angle2, ::rotationUnits unit)
{
  if(unit == Radians){angle1 = radToDeg(angle1); angle2 = radToDeg(angle2);}

  float angle = 0;
  if(angle2 > angle1)
  {
    angle = 1;
  }
  else {angle = -1;}

  if(abs(angle2 - angle1) > 180){angle *= -1; angle *= 360 - abs(angle2 - angle1);}
  else{angle *= abs(angle2 - angle1);}

  if(unit == Radians){angle = degToRad(angle);}
  return angle;
}

/*
#########################
      Vector 2
#########################
*/

struct Vector2
{
  float x;
  float y;

  Vector2()
  {
    x = 0; y = 0;
  };

  Vector2(float a, float b)
  {
    x = a; y = b;
  };

  #pragma region Vector2 Operators
  Vector2 operator + (float const &obj)
  {
    return Vector2(x - obj, y - obj);
  }

  Vector2 operator - (float const &obj)
  {
    return Vector2(x - obj, y - obj);
  }

  Vector2 operator * (float const &obj)
  {
    return Vector2(x * obj, y * obj);
  }

  Vector2 operator / (float const &obj)
  {
    return Vector2(x / obj, y / obj);
  }

  Vector2 operator + (Vector2 const &obj)
  {
    return Vector2(x + obj.x, y + obj.y);
  }

  Vector2 operator - (Vector2 const &obj)
  {
    return Vector2(x - obj.x, y - obj.y);
  }
 #pragma endregion Vector2 Operators
};

Vector2 angleToVector(float angle)
{
  angle = degToRad(angle);
  return Vector2(cos(angle), sin(angle));
}

float vectorToAngle(Vector2 vector)
{
  return wrap360(radToDeg(atan2(vector.y, vector.x)), Degrees);
}

float Distance(Vector2 p1, Vector2 p2)
{
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;

  return sqrt((dx * dx) + (dy * dy));
}

float lerp(float val1, float val2, float t)
{
  t = clamp(t,0,1);

  return (t * (val2 - val1)) + val1;
}

Vector2 lerp(Vector2 vector1, Vector2 vector2, float t)
{
  return Vector2(lerp(vector1.x, vector2.x, t), lerp(vector1.y, vector2.y, t));
}

bool inBounds(Vector2 val, Vector2 p1, Vector2 p2)
{
  if(inBounds(val.x, min(p1.x,p2.x), max(p1.x, p2.x)) && inBounds(val.y, min(p1.y,p2.y), max(p1.y, p2.y)))
  {
    return true;
  }
  return false;
}

/*
#########################
       Drivetrain
#########################
*/

enum turningDirection
{
  Left,Right
};

class chassis
{
  //when the pointer is pointing!!!
  public:
  vex::motor LeftFront = vex::motor(vex::PORT21);
  vex::motor LeftMiddle = vex::motor(vex::PORT21);
  vex::motor LeftBack = vex::motor(vex::PORT21);

  vex::motor RightFront = vex::motor(vex::PORT21);
  vex::motor RightMiddle = vex::motor(vex::PORT21);
  vex::motor RightBack = vex::motor(vex::PORT21);

  chassis()
  {

  }

  chassis(vex::motor LFront, vex::motor LMiddle, vex::motor LBack, vex::motor RFront, vex::motor RMiddle, vex::motor RBack)
  {

    LeftBack = LFront;
    LeftMiddle = LMiddle;
    LeftBack = LBack;

    RightFront = RFront;
    RightMiddle = RMiddle;
    RightFront = RBack;
  }

  void setLeftSideVelocity(float percentage, vex::percentUnits unit){
    (LeftFront).setVelocity(percentage, unit);
    (LeftMiddle).setVelocity(percentage, unit);
    (LeftBack).setVelocity(percentage, unit);
  }

  void setRightSideVelocity(float percentage, vex::percentUnits unit){
    (RightFront).setVelocity(percentage,unit);
    (RightMiddle).setVelocity(percentage, unit);
    (RightBack).setVelocity(percentage, unit);
  }

  void setVelocity(float percentage, vex::percentUnits unit){
    (LeftFront).setVelocity(percentage, unit);
    (LeftMiddle).setVelocity(percentage, unit);
    (LeftBack).setVelocity(percentage, unit);

    (RightFront).setVelocity(percentage,unit);
    (RightMiddle).setVelocity(percentage, unit);
    (RightBack).setVelocity(percentage, unit);
  }

  #pragma region DriveAndTurnCommands
  void stop(){
    (LeftFront).stop();
    (LeftMiddle).stop();
    (LeftBack).stop();

    (RightFront).stop();
    (RightMiddle).stop();
    (RightBack).stop();
  }
 
  void drive(vex::directionType direction){
    (LeftFront).spin(direction);
    (LeftMiddle).spin(direction);
    (LeftBack).spin(direction);

    (RightFront).spin(direction);
    (RightMiddle).spin(direction);
    (RightBack).spin(direction);
  }
  void drive(vex::directionType direction, float voltage, vex::voltageUnits unit){
    (LeftFront).spin(direction,voltage,unit);
    (LeftMiddle).spin(direction,voltage,unit);
    (LeftBack).spin(direction,voltage,unit);

    (RightFront).spin(direction,voltage,unit);
    (RightMiddle).spin(direction,voltage,unit);
    (RightBack).spin(direction,voltage,unit);
  }
  void drive(vex::directionType direction, float percentage, vex::percentUnits unit){
    (LeftFront).setVelocity(percentage, unit);
    (LeftMiddle).setVelocity(percentage, unit);
    (LeftBack).setVelocity(percentage, unit);

    (RightFront).setVelocity(percentage,unit);
    (RightMiddle).setVelocity(percentage, unit);
    (RightBack).setVelocity(percentage, unit);
   
    (LeftFront).spin(direction);
    (LeftMiddle).spin(direction);
    (LeftBack).spin(direction);

    (RightFront).spin(direction);
    (RightMiddle).spin(direction);
    (RightBack).spin(direction);
  }

  void turn(turningDirection direction){
    if(direction == Right)
    {
      (LeftFront).spin(vex::forward);
      (LeftMiddle).spin(vex::forward);
      (LeftBack).spin(vex::forward);
 
      (RightFront).spin(vex::reverse);
      (RightMiddle).spin(vex::reverse);
      (RightBack).spin(vex::reverse);
    }
    else
    {
      (LeftFront).spin(vex::reverse);
      (LeftMiddle).spin(vex::reverse);
      (LeftBack).spin(vex::reverse);
 
      (RightFront).spin(vex::forward);
      (RightMiddle).spin(vex::forward);
      (RightBack).spin(vex::forward);
    }
  }
  void turn(turningDirection direction, float voltage, vex::voltageUnits unit){
    if(direction == Right)
    {
      (LeftFront).spin(vex::forward,voltage,unit);
      (LeftMiddle).spin(vex::forward,voltage,unit);
      (LeftBack).spin(vex::forward,voltage,unit);
 
      (RightFront).spin(vex::reverse,voltage,unit);
      (RightMiddle).spin(vex::reverse,voltage,unit);
      (RightBack).spin(vex::reverse,voltage,unit);
    }
    else
    {
      (LeftFront).spin(vex::reverse,voltage,unit);
      (LeftMiddle).spin(vex::reverse,voltage,unit);
      (LeftBack).spin(vex::reverse,voltage,unit);
 
      (RightFront).spin(vex::forward,voltage,unit);
      (RightMiddle).spin(vex::forward,voltage,unit);
      (RightBack).spin(vex::forward,voltage,unit);
    }
  }
  void turn(turningDirection direction, float percentage, vex::percentUnits unit){
    (LeftFront).setVelocity(percentage, unit);
    (LeftMiddle).setVelocity(percentage, unit);
    (LeftBack).setVelocity(percentage, unit);

    (RightFront).setVelocity(percentage,unit);
    (RightMiddle).setVelocity(percentage, unit);
    (RightBack).setVelocity(percentage, unit);

    if(direction == Right)
    {
      (LeftFront).spin(vex::forward);
      (LeftMiddle).spin(vex::forward);
      (LeftBack).spin(vex::forward);
 
      (RightFront).spin(vex::reverse);
      (RightMiddle).spin(vex::reverse);
      (RightBack).spin(vex::reverse);
    }
    else
    {
      (LeftFront).spin(vex::reverse);
      (LeftMiddle).spin(vex::reverse);
      (LeftBack).spin(vex::reverse);
 
      (RightFront).spin(vex::forward);
      (RightMiddle).spin(vex::forward);
      (RightBack).spin(vex::forward);
    }
  }

  void DriveFor(float time, vex::timeUnits unit, vex::directionType direction)
  {
    setVelocity(100,vex::percent);
    drive(direction);
    wait(time, unit);
    stop();
  }
  #pragma endregion DriveAndTurnCommands
};

/*
#########################
       Odometer
#########################
*/

class Odometer
{
  public:
  float milimetersPerDegree;

  Vector2 position;

  chassis Drivetrain;

  vex::inertial Inertial = vex::inertial(vex::PORT21);

  float leftDistance = 0;
  float rightDistance = 0;

  Odometer()
  {

  }

  Odometer(chassis drivetrain, vex::inertial inertia, Vector2 pos ,float wheelRadius, float gearRatio, float motorRPM)
  {
    position = pos;
   
    Inertial = inertia;

    Drivetrain = drivetrain;
   
    drivetrain.LeftFront.resetPosition();
    drivetrain.RightFront.resetPosition();

    milimetersPerDegree = 2 * PI * wheelRadius * gearRatio / motorRPM;
  }

  Vector2 CalculateDeltaPos()
  {
    Vector2 deltaVector;

    Vector2 direction = angleToVector((Inertial).heading(vex::degrees));
    
    float deltaLeft = (Drivetrain).LeftFront.position(vex::degrees) - leftDistance;
    float deltaRight = (Drivetrain).RightFront.position(vex::degrees) - rightDistance;

    leftDistance = (Drivetrain).LeftFront.position(vex::degrees);
    rightDistance = (Drivetrain).RightFront.position(vex::degrees);

    deltaVector = direction * ((deltaLeft + deltaRight) / 2) * milimetersPerDegree;
    return deltaVector;
  }



  void OdomLoop()
  {
    (Drivetrain).LeftFront.resetPosition();
    (Drivetrain).RightFront.resetPosition();
    while(true)
    {
      position = position + CalculateDeltaPos();
      wait(5,vex::msec);
    }
  }
};



/*
#########################
          PID
#########################
*/

class PID
{
  public:
  vex::brain* timerBrain;

  chassis* Chassis;
  vex::inertial* inertial;
  Odometer odometer;


  float pRatio;
  float iRatio;
  float dRatio;

  float angleMargin;
  float positionMargin;
  float closeWaitTime;

  float integralTurnSum = 0;
  float integralDriveSum = 0;

  float previousTurnError = 0;
  float previousDriveError = 0;
  float Totalerror = 0;
  bool PIDactive = false;

  float targetAngle = 0;

  Vector2 curPosition;

  Vector2 targetPosition;
  Vector2 startingPos;

  PID()
  {

  }

  PID(vex::brain Brain, chassis driveTrain, Odometer odom, vex::inertial Inertial, float p = 0.3, float i = 0.1, float d = 0.1, float angle_margin = 1, float position_margin = 50, float waitTime = 0.1)
  {
    timerBrain = &Brain;
    odometer = odom;
    inertial = &Inertial;
    Chassis = &driveTrain;
    pRatio = p;
    iRatio = i;
    dRatio = d;
    angleMargin = angle_margin;
    positionMargin = position_margin;
    closeWaitTime = waitTime;

    integralTurnSum = 0;
    integralDriveSum = 0;
  }

  /*Coding sometimes is like sailing a ship execpt
  you have no idea how to sail a ship and your only map
  is a kindergardener's art project.

  This is one of those times*/
  float CalculateTurningError()
  {
    float curRotation = (*inertial).heading(vex::degrees);
    float error = angleDifference(curRotation, targetAngle, Degrees);

    float p = error;
    integralTurnSum += error;
    float d = error - previousTurnError;

    error = (p * pRatio) + (integralTurnSum * iRatio) + (d * dRatio);
    previousTurnError = error;
    return error;
  }

  float CalculateDrivingError()
  {
    float error = Distance(odometer.position, targetPosition);
  
    if(error <= Distance(odometer.position, startingPos)){error *= -1;}

    timerBrain->Screen.setCursor(4,1);
    timerBrain->Screen.print(odometer.position.x);

    float p = error;
    integralDriveSum += error;
    float d = error - previousDriveError;

    error = (p * pRatio) + (integralDriveSum * iRatio) + (d * dRatio);
    //error = (error * error * error) / 250;
    previousDriveError = error;
    return error;
  }

  float lastTime = 0;
  float lastTimeout = 0;
  bool CloseEnough()
  {
    if(lastTimeout + 5 < timerBrain->Timer.time(vex::seconds))
    {
      return true;
    }

    if(abs(previousTurnError) >= angleMargin || abs(previousDriveError) >= positionMargin)
    {
      lastTime = timerBrain->Timer.time(vex::msec);
      return false;
    }
    else if(timerBrain->Timer.time(vex::msec) >= lastTime + closeWaitTime)
    {
      
      return true;
    }
    return false;
  }

  void TurnTo(float angle, ::rotationUnits unit)
  {
    if(unit == Radians)
    {
      angle = radToDeg(angle);
    }
    targetAngle = angle;
    targetPosition = odometer.position;

    startingPos = odometer.position;
    lastTimeout = timerBrain->Timer.time(vex::seconds);

    integralTurnSum = 0;
    integralDriveSum = 0;
    PIDactive = true;
  }

  void Turn(float angle, ::rotationUnits unit)
  {
    if(unit == Radians)
    {
      angle = radToDeg(angle);
    }
    targetAngle = wrap360(angle + inertial->heading(vex::degrees), Degrees);
    targetPosition = odometer.position;

    startingPos = odometer.position;

    lastTimeout = timerBrain->Timer.time(vex::seconds);

    integralTurnSum = 0;
    integralDriveSum = 0;
    PIDactive = true;
  }

  void DriveTo(Vector2 position)
  {
    lastTime = (*timerBrain).Timer.time();

    targetAngle = vectorToAngle(position - odometer.position);
    targetPosition = position;

    startingPos = odometer.position;
    lastTimeout = timerBrain->Timer.time(vex::seconds);

    integralTurnSum = 0;
    integralDriveSum = 0;
    PIDactive = true;
  }

  void PIDLoop()
  {
    while(true)
    {
      while(PIDactive)
      {
        float curRotation = (*inertial).heading(vex::degrees);
        
        float turnError = CalculateTurningError();
        float driveError = 0;//CalculateDrivingError() / 2;

        timerBrain->Screen.setCursor(1,1);
        (*timerBrain).Screen.print(driveError);

        (*Chassis).setLeftSideVelocity(-driveError - turnError, vex::percent);
        (*Chassis).setRightSideVelocity(driveError - turnError, vex::percent);
        (*Chassis).drive(vex::forward);
        if(CloseEnough())
        {
          timerBrain->Screen.setCursor(3,1);
          timerBrain->Screen.print("CLOSE!!!!!!");
          PIDactive = false;
        }
        wait(5,vex::msec);
      }
      Chassis->stop();
      wait(5,vex::msec);
    }
  }
};



struct Path
{
  std::vector<Vector2> points;

  Path()
  {
  }

  void addPoint(Vector2 point)
  {
    points.push_back(point);
  }

};

class Pursuit
{
  public:
  Path path;
  
  float lookAheadDistance;

  chassis Chassis;
  PID pid;
  Odometer odometer;

  Pursuit(float lookAhead, chassis Drivetrain, PID Pid, Odometer Odom)
  {
    lookAheadDistance = lookAhead;
    Chassis = Drivetrain;
    pid = Pid;
    odometer = Odom;
  }

  bool FoundGoalPoint = false;

  Vector2 FindGoalPoint()
  {
    Vector2 goal;

    FoundGoalPoint = false;
    
    Vector2 pos = (odometer).position;

    Vector2 lastPoint = path.points[1];
    if(Distance(lastPoint, pos) < lookAheadDistance)
    {
      FoundGoalPoint = false;
      return lastPoint;
    }

    
    //When the math makes no sense!!!
    for(int i = path.points.size(); i > 0; i--)
    {
      if(Distance(path.points[i], odometer.position) < lookAheadDistance)
      {
        return path.points[i + 1];
      }

      /*Vector2 p1 = path.points[i - 1] - pos;
      Vector2 p2 = path.points[i] - pos;

      float dx = p2.x - p1.x;
      float dy = p2.y - p1.y;

      float dr = sqrt((dx * dx) + (dy * dy));
      float D = (p1.x * p2.y) - (p2.x * p1.y);

      float discriminate = (pow(lookAheadDistance, 2) * pow(dr, 2)) - pow(D,2);

      if(discriminate <= 0)
      {
        continue;
      }  
      else
      {
        //???????
        float x1 = ((D * dy) + (signOf(dy) * dx * sqrt(discriminate))) / pow(dr,2);
        float y1 = ((-D * dx) + (abs(dy) * sqrt(discriminate))) / pow(dr,2);
        
        float x2 = ((D * dy) - (signOf(dy) * dx * sqrt(discriminate))) / pow(dr,2);
        float y2 = ((-D * dx) - (abs(dy) * sqrt(discriminate))) / pow(dr,2);

        Vector2 goalPoint1 = Vector2(x1,y1);
        Vector2 goalPoint2 = Vector2(x2,y2);
        
        if(Distance(goalPoint1, p2) < Distance(goalPoint2, p2) && inBounds(goalPoint1, p1, p2))
        {
          goal = goalPoint1;
          FoundGoalPoint = true;
          break;
        }
        else if(inBounds(goalPoint2,p1,p2))
        {
          FoundGoalPoint = true;
          goal = goalPoint2;
          break;
        }
      }*/
    }

    return goal;
  }

  void FollowPath(Path directions)
  {
    path = directions;
    Vector2 lastPoint = directions.points[directions.points.size() - 1];
    while(Distance(lastPoint, odometer.position) > 50)
    {
      Chassis.drive(vex::forward);

      Vector2 goalPoint = FindGoalPoint();

      float turningError = angleDifference(vectorToAngle(goalPoint - odometer.position), odometer.Inertial.heading(), Degrees);
      float drivingError = Distance(lastPoint, odometer.position);

      Chassis.setLeftSideVelocity(-drivingError - turningError, vex::percent);
      Chassis.setLeftSideVelocity(drivingError - turningError, vex::percent);
    }
    Chassis.stop();
  } 
};