function 3D_Differential_Mobile_Inverted_Pendulum
%===========================================================================
% File: 3D_Differential_Mobile_Inverted_Pendulum.m created on Mon Jul 17 2017 by MotionGenesis 5.8.
% Portions copyright (c) 2009-2016 Motion Genesis LLC.  Rights reserved.
% Get Started Licensee: Dongil Choi (until August 2018).
% Paid-up MotionGenesis Get Started licensees are granted the right
% right to distribute this code for legal student-academic (non-professional) purposes only,
% provided this copyright notice appears in all copies and distributions.
%===========================================================================
% The software is provided "as is", without warranty of any kind, express or    
% implied, including but not limited to the warranties of merchantability or    
% fitness for a particular purpose. In no event shall the authors, contributors,
% or copyright holders be liable for any claim, damages or other liability,     
% whether in an action of contract, tort, or otherwise, arising from, out of, or
% in connection with the software or the use or other dealings in the software. 
%===========================================================================
A = zeros( 6, 6 );
B = zeros( 6, 2 );


%-------------------------------+--------------------------+-------------------+-----------------
% Quantity                      | Value                    | Units             | Description
%-------------------------------|--------------------------|-------------------|-----------------
d                               =  0.0;                    % UNITS               Constant
g                               =  0.0;                    % UNITS               Constant
Ipyy                            =  0.0;                    % UNITS               Constant
Ipzz                            =  0.0;                    % UNITS               Constant
J                               =  0.0;                    % UNITS               Constant
K                               =  0.0;                    % UNITS               Constant
lp                              =  0.0;                    % UNITS               Constant
mp                              =  0.0;                    % UNITS               Constant
Mw                              =  0.0;                    % UNITS               Constant
R                               =  0.0;                    % UNITS               Constant
%-------------------------------+--------------------------+-------------------+-----------------



%===========================================================================


%===========================================================================
Output = [];


A(1,1) = 0;
A(1,2) = 1;
A(1,3) = 0;
A(1,4) = 0;
A(1,5) = 0;
A(1,6) = 0;
A(2,1) = 0;
A(2,2) = 0;
A(2,3) = mp^2*g*lp^2/(mp^2*lp^2-(Ipyy+mp*lp^2)*(mp+2*Mw+2*J/R^2));
A(2,4) = 0;
A(2,5) = 0;
A(2,6) = 0;
A(3,1) = 0;
A(3,2) = 0;
A(3,3) = 0;
A(3,4) = 1;
A(3,5) = 0;
A(3,6) = 0;
A(4,1) = 0;
A(4,2) = 0;
A(4,3) = -mp*g*lp*(mp+2*Mw+2*J/R^2)/(mp^2*lp^2-(Ipyy+mp*lp^2)*(mp+2*Mw+2*J/R^2));
A(4,4) = 0;
A(4,5) = 0;
A(4,6) = 0;
A(5,1) = 0;
A(5,2) = 0;
A(5,3) = 0;
A(5,4) = 0;
A(5,5) = 0;
A(5,6) = 1;
A(6,1) = 0;
A(6,2) = 0;
A(6,3) = 0;
A(6,4) = 0;
A(6,5) = 0;
A(6,6) = 0;

B(1,1) = 0;
B(1,2) = 0;
B(2,1) = -(mp*lp+(Ipyy+mp*lp^2)/R)/(mp^2*lp^2-(Ipyy+mp*lp^2)*(mp+2*Mw+2*J/R^2));
B(2,2) = -(mp*lp+(Ipyy+mp*lp^2)/R)/(mp^2*lp^2-(Ipyy+mp*lp^2)*(mp+2*Mw+2*J/R^2));
B(3,1) = 0;
B(3,2) = 0;
B(4,1) = (mp+2*Mw+2*J/R^2+mp*lp/R)/(mp^2*lp^2-(Ipyy+mp*lp^2)*(mp+2*Mw+2*J/R^2));
B(4,2) = (mp+2*Mw+2*J/R^2+mp*lp/R)/(mp^2*lp^2-(Ipyy+mp*lp^2)*(mp+2*Mw+2*J/R^2));
B(5,1) = 0;
B(5,2) = 0;
B(6,1) = -d/(R*(Ipzz+2*K+2*Mw*d^2+2*J*d^2/R^2));
B(6,2) = d/(R*(Ipzz+2*K+2*Mw*d^2+2*J*d^2/R^2));


%=================================================================
end    % End of function 3D_Differential_Mobile_Inverted_Pendulum
%=================================================================
