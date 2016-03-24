function [ Q ] = Pos2Quat ( x, theta )
% this function used to get Quaternions transformation matrix from position vector (x,y,z)
%% Coded by
% Mohamed Mohamed El-Sayed Atyya
% mohamed.atyya94@eng-st.cu.edu.eg
%% inputs
% x        : the position vector
% theta  :  Euler principal rotation angle in degree
%% outputs
% Q   :  Quaternions transformation matrix from XYZ to xyz
% -----------------------------------------------------------------------------------------------------------------------------------------------------------
l=x(1)/norm(x);
m=x(2)/norm(x);
n=x(3)/norm(x);
q1=l*sind(theta/2);
q2=m*sind(theta/2);
q3=n*sind(theta/2);
q4=cosd(theta/2);
qhat_norm=norm([q1 q2 q3 q4]);  % must equal 1
Q=[q1^2-q2^2-q3^2+q4^2, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4); ...
       2*(q1*q2-q3*q4), -q1^2+q2^2-q3^2+q4^2, 2*(q2*q3+q1*q4); ...
       2*(q1*q3+q2*q4), 2*(q2*q3-q1*q4), -q1^2-q2^2+q3^2+q4^2];
end