function points_mean_final  = forward_kinematics(M1,M2,M3,M4)
%FORWARD_KINEMATICS Estimate the experimental four-cable robot position.
%   P = FORWARD_KINEMATICS(M1,M2,M3,M4) returns a 2x1 double [x;y] in mm.
%   Each input is a real scalar double motor displacement in mm after the
%   controller's encoder conversion and reference-offset calibration.
%   Inputs are not raw encoder counts or absolute cable lengths.
%
%   Geometry: a 630 mm square, origin at bottom-left, +x right, +y up.
%   Controller cable order and anchor coordinates [x;y], in mm:
%       1: bottom-left [0;0]       2: top-left [0;630]
%       3: top-right [630;630]    4: bottom-right [630;0]
%   With r0 = 630*sqrt(2)/2, cable lengths are
%       [r0-M1; r0+M2; r0-M3; r0+M4].
%   The controller converts encoder counts at 150/4000 mm/count before
%   reference-offset calibration. That preprocessing is outside this function.
%
%   The four ordered circle pairs are (1,2), (2,3), (3,4), (4,1).
%   In each pair's local frame, the second anchor is at [0;d]. Subtracting
%   the two circle equations gives y=(ri^2+d^2-rj^2)/(2*d), followed by
%   x=sqrt(ri^2-y^2). The positive root points into the square. The four
%   intersections are transformed to the common frame. Points containing NaN
%   are omitted; the remaining points are averaged equally.
%   The intended geometric domain is an end effector inside the frame.
%
%   Status/state: only the position is returned; there is no validity flag.
%   There is no persistent state, fallback or explicit length-domain check.
%   If no points remain, division by zero returns NaNs. In the experimental
%   real-valued generated controller, a negative square-root argument gives
%   NaN and that pair is omitted. Standalone MATLAB can instead return a
%   complex square root; complex points are not rejected by the NaN test.
%   Use positive lengths with real adjacent-circle intersections for this
%   standalone example. Check that P is real and finite before using it.
%   Types and dimensions are assumed, not checked. Diagonal-pair candidates
%   are also calculated in the source but do not affect the returned P.
%
%   For frame-centred coordinates use P-[315;315]. The tension distribution
%   routines use cable order [1 4 3 2] relative to this controller order.
%   Reorder cable-indexed quantities when connecting those two conventions.
%   Forward kinematics is excluded from all reported TDA execution times.
%
%   See also RUN_FORWARD_KINEMATICS_EXAMPLE.

% Storage for the fixed circle-pair calculations.
y= zeros([1 4]);
x= y;
points_pre = zeros([2 4]);
d= 630;
y_c= d;
x_c=y_c;
y2= zeros([2 4]);
x2= zeros([2 4]);
points_pre_p= zeros([2 2]);
points_pre_n= zeros([2 2]);
points_pre_p_diff= zeros([2 2]);
points_pre_n_diff= zeros([2 2]);
ip= zeros([2 2]);
xp= zeros([1 2]);
xn= zeros([1 2]);
pointsp= zeros([2 2]);
pointsn= zeros([2 2]);
y56= zeros([1 2]);
num_secPoints=0;
NaNs=0;
points_pre_sum =zeros([2 1]);
num= 0;

% Convert calibrated motor displacements to absolute cable lengths.
r1= d*sqrt(2)/2-M1;
r2= d*sqrt(2)/2+M2;
r3= d*sqrt(2)/2-M3;
r4= d*sqrt(2)/2+M4;
r = [r1;r2;r3;r4;r1];

% Adjacent pairs: (1,2), (2,3), (3,4), (4,1). Choose local +x.
for i=1:1:4
    y_calc= (r(i)^2+y_c^2-r(i+1)^2)/(2*y_c);
    y(i)=y_calc;
    x(i) = sqrt(r(i)^2-y_calc^2);
end
points =[x;y];

% Anchor origins and local axes; +x points into the square.
CS0 = [0;0];
CS1 = [0;d];
CS2 = [d;d];
CS3 = [d;0];
r_d = [CS0,CS1,CS2,CS3,CS0];
y12=((r_d(:,2)-r_d(:,1))/norm(r_d(:,2)-r_d(:,1)));
x12=[0,1;-1,0]*y12;
y23=((r_d(:,3)-r_d(:,2))/norm(r_d(:,3)-r_d(:,2)));
x23=[0,1;-1,0]*y23;
y34=((r_d(:,4)-r_d(:,3))/norm(r_d(:,4)-r_d(:,3)));
x34=[0,1;-1,0]*y34;
y41=((r_d(:,1)-r_d(:,4))/norm(r_d(:,1)-r_d(:,4)));
x41=[0,1;-1,0]*y41;
xx=[x12 x23 x34 x41];
yy=[y12 y23 y34 y41];

% Transform to the common frame and omit points containing NaN.
for i= 1:1:4
    points_pre(:,i) = r_d(:,i) + [xx(:,i) yy(:,i)]*points(:,i);
    if sum(isnan(points_pre(:,i)))>0
        NaNs=NaNs+1;
    else
        points_pre_sum = points_pre(:,i) +points_pre_sum;
        num_secPoints= num_secPoints+ 1;
    end
end
points_pre_mean= points_pre_sum/(num_secPoints);

% Diagonal-pair calculations are retained but do not contribute to P.
y_c56=892;
for i=1:1:2
    y_calc56= (r(i)^2+y_c56^2-r(i+2)^2)/(2*y_c56);
    y56(i)=y_calc56;
    xp(i) = sqrt(r(i)^2-y_calc56^2);
    xn(i) = -sqrt(r(i)^2-y_calc56^2);
end
pointsp =[xp;y56];
pointsn =[xn;y56];
y13=((r_d(:,3)-r_d(:,1))/norm(r_d(:,3)-r_d(:,1)));
x13=[0,1;-1,0]*y13;
y24=((r_d(:,4)-r_d(:,2))/norm(r_d(:,4)-r_d(:,2)));
x24=[0,1;-1,0]*y24;
xx=[x13 x24];
yy=[y13 y24];
for i=1:1:2
    points_pre_p(:,i) = r_d(:,i) + [xx(:,i) yy(:,i)]*pointsp(:,i);
    points_pre_n(:,i) = r_d(:,i) + [xx(:,i) yy(:,i)]*pointsn(:,i);
    points_pre_p_diff(:,i) = abs(points_pre_mean-points_pre_p(:,i));
    points_pre_n_diff(:,i) = abs(points_pre_mean-points_pre_n(:,i));
end
for i=1:1:2
    if sum(isnan(points_pre_p_diff(:,i)))>0 && sum(isnan(points_pre_n_diff(:,i)))>0
        ip(:,i)=[0;0];
    elseif sum(isnan(points_pre_p_diff(:,i)))>0
        ip(:,i)=points_pre_n(:,i);
        num=num+1;
    elseif sum(isnan(points_pre_n_diff(:,i)))>0
        ip(:,i)=points_pre_p(:,i);
        num=num+1;
    elseif points_pre_p_diff(:,i)>=points_pre_n_diff(:,i)
        ip(:,i)=points_pre_n(:,i);
        num=num+1;
    else
        ip(:,i)=points_pre_p(:,i);
        num=num+1;
    end
end
NumSecPoints=num_secPoints+num;

% Return only the mean of the retained adjacent-pair intersections.
points_mean_final=points_pre_mean;
