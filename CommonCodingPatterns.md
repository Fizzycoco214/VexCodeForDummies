# Button Toggle
```c++
bool lastPressed = false;
while(true)
{
  if(lastPressed == false && button.pressed() == true)
  {
    DoThing();
  }
  lastPressed = button.pressed();
}
```

