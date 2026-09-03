
%TESTING ON TEXTBOOK CALCULATION: https://www.hlevkin.com/hlevkin/90MathPhysBioBooks/Mechanics/Curtis_OrbitamMechForEngineeringStudents.pdf
[semi_major_axis, eccentricity, inclination, ascending_node, periapsis, true_anomaly] = propogateOrbitalElements((9600e3 + 21000e3)/2, 0.37255, 0, 0, 0, 0, 10800);
%correct output of 3.3371 radians

%TESTING ON MATLAB LIBRARY: https://www.mathworks.com/help/aerotbx/ug/propagateorbit.html
%Shoudl scale this up to J2
function testOnMatlab(axis, e0, inc0, RAAN0, argp0, nu0)
% 1 min increments of testing, over 2 hours
sampleTime = 60;
startTime = datetime(2025,10,22,12,0,0); 
endTime = datetime(2025,10,22,14,0,0); 
time = startTime:seconds(sampleTime):endTime;

%Testing on Matlab aerospace toolbox functions
[position, velocity] = propagateOrbit(time, axis, e0, inc0, RAAN0, argp0, nu0, PropModel="two-body-keplerian"); 
[a, ecc, incl, RAAN, argp, nu, truelon, arglat, lonper] = ijk2keplerian(position, velocity); 

true_anamoly_vector = [];
%loop over every minute, to get the next prediction
for j=0:120
    [a1, ecc1, incl1, RAAN1, argp1, nu1] = propogateOrbitalElements(axis, e0, inc0, RAAN0, argp0, nu0, j*sampleTime);
    true_anamoly_vector(j+1) = nu1;
    
end
%get total error between the verified function and mine
error = abs(wrapTo360(true_anamoly_vector) - wrapTo360(nu));
mean(error);
end

%testing various e
testOnMatlab(1e7, 0.7, 70, 10, 120, 1); %mean error e-8
testOnMatlab(1e7, 0.5, 70, 10, 120, 1); %mean error e-8
testOnMatlab(1e7, 0.10, 70, 10, 120, 1); %mean error e-9
testOnMatlab(1e7, 0.9, 70, 10, 120, 1); %mean error e-9
testOnMatlab(1e7, 0.95, 70, 10, 120, 1); %mean error e-9
testOnMatlab(1e7, 0.05, 70, 10, 120, 1); %mean error e-8

%testing various true anamolys
testOnMatlab(1e7, 0.3, 150, 10, -40, 1); %mean error e-8
testOnMatlab(1e7, 0.3, 150, 10, -40, -180); %mean error e-8
testOnMatlab(1e7, 0.3, 150, 10, -40, 90); %mean error e-8
testOnMatlab(1e7, 0.3, 150, 10, -40, 240); %mean error e-8

%testing various a
testOnMatlab(1e10, 0.7, 78, 13, 70, 28); %mean error e-9
testOnMatlab(1e3, 0.5, 70, 10, 200, 59); %mean error e-8
testOnMatlab(1e12, 0.10, 70, 10, 40, 192); %mean error e-8
testOnMatlab(1e6, 0.9, 70, 10, 10, -25); %mean error e-8
testOnMatlab(1e8, 0.95, 70, 10, 120, 320); %mean error e-8
testOnMatlab(1e7, 0.05, 70, 10, 80, 143); %mean error e-8