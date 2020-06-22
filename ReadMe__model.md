# MOTORCYCLE MODEL

## Reference frames

A **mobile reference frame** can be defined starting from a fixed one (ABSOLUTE ORIGIN). The reference frame is moving with a longitudinal velocity, a lateral velocity and an angular velocity.

$$u(t), \; v(t), \; \Omega(t)$$

```
MF_R       := use_moving_frame(u(t),v(t),0, 0,0,Omega(t)):
RF__mobile := moving_frame:
RF1        := RF__mobile: 
```

Some of the variable that are going to be used are for definition are small. Therefore they should be defined inside the linear modeling command.


```
linear_modeling({delta(t),delta__f(t)});
```

We can safely assume that moving reference frame is rigidly moving with the motorcycle along the track. From this frame we can describe the motion of the whole motorcycle.

The first step is to define a plane that will represent the motorcycle when it leans during cornering motion. The frame is called $RF_\phi$


```
RF__phi := RF1 * rotate('X',phi(t));
```

TO BE IMPLEMENTED...

