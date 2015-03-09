For the TaskDefinition:
  The user should always set the tolerance value: if to be ignored, it should be set to a negative value or 0...
  Constructor is only place where the interpolation size can be set...
  Ideally the compute functions should also set the appropriate jacobian entry for storage
  
  

  
  OptimisationFunction currently does not support vectorial tasks... will need to fix this
  
  
  Indicate in writeup ability to set debug compilation or not...
  
  In writeup:
    1) Registration : and careful about including full namespacing!!!
    2) Must implement default constructor
    3) Namespace rules: what should be within the library, generality, dependencies etc..
    4) Efficient way of implementing the position solver with updateState...
    
