# warmup_project

## Drive in a square

I used a timing approach for this problem. The `DriveSquare` class creates
two `cmd_vel` commands: one that makes the robot drive forward in a straight
line, and one that makes the robot turn in place. The class' main loop keeps
a counter that ticks up every time the `Rate` object causes the loop to be 
run. Using this counter, the loop repeatedly sends turning commands for about
3.5 seconds, then forward commands for about 5 seconds.

### Driving in a square gif

![drive_square.gif](https://gifhub.com/dpan229/warmup_project/blob/main/gifs/drive_square.gif)
