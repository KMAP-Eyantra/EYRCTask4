pkg load control
A = [   0.            1.            0.            0.     ;   
    0.          -3.52645454  -16.65793859   0.22921955;
    0.            0.            0.            1.        ;
    0.         -26.94257673 162.13373685   1.75126749];
B = [ 0.        ;
  0.51118094;
  0.        ;
 3.90548961;];

Q = [1 0 0 0;
 0 0 0 0;
 0 0 1 0;
 0 0 0 0;];
 R = 1;
 
[K,S,P] = lqr(A,B,Q,R);
K