# Tensegrity Modelling

### Installing ODE

See here:

1. [http://www.cs.cmu.edu/~cga/ode/](http://www.cs.cmu.edu/~cga/ode/)

(**Do not forget to copy the library and header files as instructed**)

2. [https://www.ode-wiki.org/wiki/index.php?title=Manual:_Install_and_Use](https://www.ode-wiki.org/wiki/index.php?title=Manual:_Install_and_Use)

### Installing Python Bindings

1. Install ODE using instructions above, but call `./configure` as

```shell
./configure --enable-double-precision --with-trimesh=opcode --enable-new-trimesh --enable-shared
```

2. From the ODE root directory, `cd` into `bindings/python`

3. ``python setup.py install``.

Full instructions can be found [here at the ODE wiki](https://www.ode-wiki.org/wiki/index.php?title=Manual:_Install_and_Use#Install_with_Python_bindings).

####Troubleshooting

1. You may need to use `sudo` when you call `make install` while installing ODE itself.

2. If you receive an error while trying to import ode into python, try running
``sudo ldconfig``

### Compiling

```shell
g++ tensegrity.cpp -o tensegrity -lode -ldrawstuff -lX11 -lGLU -lGL -lpthread
```

Download **tensegrity** and run **./tensegrity** to run the compiled demo.

### ODE Tutorials

Some good ODE tutorials:

1. [http://demura.net/english](http://demura.net/english)
