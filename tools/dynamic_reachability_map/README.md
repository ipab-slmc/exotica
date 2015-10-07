# Dynamic Reachability Map (DRM)
How to use:

## Step 1: Get a DRM sampled space
```
roslaunch dynamic_reachability_map drm_sampler.launch
```

This will create a folder under ``dynamic_reachability_map/result`` named by ``robot name``-``sample size``-``cell resolution``

Note: the result files can be very large, e.g. 1000000 samples, 10cm resolution will create 500MB of files. Make sure you have enough space and DO NOT commit the result to git.

__PS: 100000 samples with 10cm resolution is enough for LWR IK task.__

## Step 2: Launch the DRM action server
``` 
roslaunch dynamic_reachability_map drm.launch
```

## Step 3: Use the DRM as IKsolver
In any EXOTica XML configuration file, add a param named `UseDRM` and set it to true.
```
<IKsolver...>
  ...
  <UseDRM>1</UseDRM>
</IKsolver>
```
