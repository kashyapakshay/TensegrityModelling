# Tensegrity Modelling

### Installing ODE

See here:

1. [http://www.cs.cmu.edu/~cga/ode/](http://www.cs.cmu.edu/~cga/ode/)

(**Do not forget to copy the library and header files as instructed**)

2. [https://www.ode-wiki.org/wiki/index.php?title=Manual:_Install_and_Use](https://www.ode-wiki.org/wiki/index.php?title=Manual:_Install_and_Use)

### Compiling

```shell
g++ tensegrity.cpp -o tensegrity -lode -ldrawstuff -lX11 -lGLU -lGL -lpthread
```
