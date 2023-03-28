Here's an interesting coding challenge, the mandlebrot set:
![](brot.jpg)

This image is one of the most famous fractals. A fractal is basically an image that you can zoom into forever because it's infinitely detailed. An interesting property of fractals is that they have a finite area but infinite surface area. If the mandelbrot set was a cake, you could bake it but you could never cover it in icing.

There's still the question of, what is the mandlebrot set? (if you don't want to read the explanation you can just skip it)

# What is the mandlebrot set?
The mandelbrot set exists in the complex plane, meaning that instead of a normal coordinate grid with an x and y axis, it has an x (real) axis but a y (imaginary) axis measured in ***i*** where ***i*** equals the square root of -1. This means that coordinates in the complex plane look like (1.5,0.4*i*).

## Actually calculating the thing
THe mandelbrot set is calculated by taking a point on this complex plain, squaring it, and then adding the original point's value. When done over and over again, the point will do one of two things: it either stays near the origin (it never goes farther than 2 units away from the origin), or it flies off to infinity and is never seen again. If we run that process, for example, a thousand times each for a bunch of points, we can say that points which stay near the origin are inside the mandelbrot set, and points that go off to infinity are not.

# Actually Coding the thing
Now
