%screw = [wx, wy ,wz, vx,vy,vz]

theta = [0.523599, 0.523599,-0.523599,0,0,0];

s1 = [0,   0,  0,     0,       0,      0];
s2 = [0,   0,  1,     0,       0,      0];
s3 = [0,  -1,  0,     0.105,   0,      0];
s4 = [0,   1,  0,    -0.365,   0,      0];
s5 = [0,  -1,  0,     0.645371,0,      0];
s6 = [1,   0,  0,     0,  0.685371,    0];
s7 = [1,   0,  0,     0,       0,      0];

M1 = [1,   0,   0, 0;
      0,   1,   0, 0;
      0,   0,   1, 0;
      0,   0,   0, 1];


M2 = [1,   0,   0, 0.0002;
      0,   1,   0, 0.0002325;
      0,   0,   1, 0.101555;
      0,   0,   0, 1];

M3 = [1,   0,   0, 0.0000;
      0,   1,   0, -0.010885;
      0,   0,   1, 0.364505;
      0,   0,   0, 1];

M4 = [1,   0,   0, 0.0002;
      0,   1,   0, 0.02325;
      0,   0,   1, 0.556640;
      0,   0,   0, 1];

M5 = [1,   0,   0, -0.055719;
      0,   1,   0, 0.002566;
      0,   0,   1, 0.688473;
      0,   0,   0, 1];

M6 = [1,   0,   0, -0.0013269;
      0,   1,   0, 0.002646;
      0,   0,   1, 0.695533;
      0,   0,   0, 1];

adM2 = Adjoint(inv(M2));
adM3 = Adjoint(inv(M3));
adM4 = Adjoint(inv(M4));
adM5 = Adjoint(inv(M5));
adM6 = Adjoint(inv(M6));

A2 = adM2* s2';
A3 = adM3* s3';
A4 = adM4* s4';
A5 = adM5* s5';
A6 = adM6* s6';


omegaA2 = [0, -A2(3), A2(2);
           A2(3), 0 , -A2(1);
           -A2(2),A2(1),0];

I = eye(3);

vA2 = [A2(4),A2(5), A2(6)];


expA2 = I +sin(theta(2))* omegaA2 + (1-cos(theta(2)))*omegaA2*omegaA2;

G2 = I*theta(2) +(1-cos(theta(2)))*omegaA2 + (theta(2) - sin(theta(2)))*omegaA2*omegaA2;


expTransA2 = [expA2, G2*vA2';
             0,0,0,1];

TransA2 = M1*M2*expTransA2;


%Joint3
        

omegaA3 = [0, -A3(3), A3(2);
           A3(3), 0 , -A3(1);
           -A3(2),A3(1),0];


vA3 = [A3(4),A3(5), A3(6)];


expA3 = I +  (sin(theta(3))* omegaA3) + (1-cos(theta(3)))*(omegaA3*omegaA3);

G3 = I*theta(3) +(1-cos(theta(3)))*omegaA3 + (theta(3) - sin(theta(3)))*omegaA3*omegaA3;


expTransA3 = [expA3, G3*vA3';
             0,0,0,1];


T23 =  inv(M2)*M3*expTransA3;



