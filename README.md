[toc]

# Mini Drifter

Code and 3d print files for our Senior Design project. The mini drifter is a solar powered smart buoy designed to survive a year with minimal maintenence. It's housing is a modified pelican 1060 case, with a 3d-printed sensor guard epoxied to the bottom.



## Code conventions
### Function prefixes
Code is organized into functions by component, with prefixes denoting the component that the function relates to.
For example:

```
void sd_foo() {
   //Do something with sd card
}
```

this would be the name of a function foo that exists for interacting with the sd card. As a more concrete example, here's our function that sets up the epaper display:
```
void epaper_setup() {
  epd.begin();
  epd.setTextWrap(true);
  epd.setTextSize(2);
}
```

and all other functions related to the epaper display will also begin with "epaper\_"

### Camel case after prefix
After the prefix of the function, camel case is used. For example:
```
void epaper_doTheThing() {
  //Stuff
}
```



## Organization of functions

The first function in the file should be arduino's setup() function, followed by loop(), then the componenent specific functions (which should be grouped together by component).



# 3d print files

The 3d print for the sensor guard is in the SensorGuardPrint folder. It is provided both as a .gcode file and a .stl file to acommadate different types of printers.