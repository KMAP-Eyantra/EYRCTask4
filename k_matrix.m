####%robot parameters
##mb = 0.987; %mass of robot
##mw = 0.025; %mass of wheels
##jb = 0.00383; %moment of inertia about the centre of mass
##r = 0.04; %radius of wheels
##jw = 4E-05; %moment of inertia for the wheels
##l = 0.102; %distance from wheel axle to CoM
##ke = 0.855;
##km = 0.316;
##R = 7.2; %motor resistance
##g = 9.81; %gravity
##b = 0.002; %Viscous friction constant

pkg load control;
pkg load signal
mb = (1.0-0.310-0.050);               #Robot Mass Kg 
mw = (91+310)/1000;         #Wheel Mass Kg
h = 0.245       ;         #height m
w = 0.07        ;        #width m #not distance between wheels
jb = 7.53614*0.001 #Moment of Inertia about center Kgm^2
r = 0.065/2              ; #Radius m
jw = 0.5*(mw)*r*r     ;#Moment of Inertia of wheels Kgm^2
l = (0.08)              ; ### Distance of wheel to center of mass
ke = 0.4775;#0.3183             ;### EMF Constant Vs/rad #ke = kb
km = 0.3472#0.41675       ;## Torque Constant Nm/A #km = kt
R = 5               ;### Motor Resistance ohm
g = 9.81               ; #m/s^2     
b = 0.002               ;### Viscous Friction constant Nms/rad

alp = (2*(R*b - ke*km)*(mb*l*l + mb*r*l +jb))/ R*(2*(jb*jw + jw*l*l*mb +
jb*mw*r*r + l*l*mb*mw*r*r)+jb*mb*r*r);
bet = (-l*l*mb*mb*g*r*r)/(jb*(2*jw + mb*r*r + 2*mw*r*r) + 2*jw*l*l*mb +
2*l*l*mb*mw*r*r);
gam = (-2*(R*b -ke*km)*(2*jw + mb*r*r + 2*mw*r*r + l*mb*r))/(R*r*(2*(jb*jw
+ jw*l*l*mb + jb*mw*r*r + l*l*mb*mw*r*r)+jb*mb*r*r));
delt = (l*mb*g*(2*jw + mb*r*r + 2*mw*r*r))/(2*jb*jw + 2*jw*l*l*mb +
jb*mb*r*r +2*jb*mw*r*r + 2*l*l*mb*mw*r*r);
chi = (km*r)/(R*b - ke*km);
A = [ 0 1 0 0;
      0 alp bet -r*alp;
      0 0 0 1;
      0 gam delt -r*alp];
      
B = [0; alp*chi; 0; gam*chi];

C = [ 1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];
      
D = [0;0;0;0];

Q=C'*C;
Q =   [2 0 0 0 ; 
       0 0 0 0 ; 
       0 0 400 0 ; 
       0 0 0 100];
##[n,d]=ss2tf(A,B,C,D);
##n  %num
##d  %den
sys = ss(A,B,C,D);
sys = c2d(sys,0.004);
R_l = 1;
[K,S,e] = dlqr(sys.a,sys.b,Q,R_l);
%[K,S,e] = dlqr(A,B,Q,R);
e
strcat("float k1 = ",num2str(K(1)),",k2 = ",num2str(K(2)),",k3 = ",num2str(K(3)),",k4 = ",num2str(K(4)),";")
