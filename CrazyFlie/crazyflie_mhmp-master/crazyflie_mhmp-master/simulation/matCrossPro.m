% matrix form of cross product of a vector
function y = matCrossPro(v)
y = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];