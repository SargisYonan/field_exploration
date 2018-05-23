pointA = [ 1, 2,   0*pi/180 ];     
pointB = [ 2, 1, pi/180 ];    

TurnRadius = .1;   
PathStep = 0.1;  

PATH = dubins_curve(pointA,pointB, TurnRadius, PathStep);