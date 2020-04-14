%-------------------------------------------------------
% simulates motion: motion is in form of
% [first rotation(rad); forward dist; second rotation(rad)]
%-------------------------------------------------------
function m = generateMotion(t,deltaT)

if (deltaT>1.0)
    error('deltaT assumed to by < 1.0');
end

n=t/deltaT;

index=mod(n,floor(1/deltaT)*5);

if index == 0
    m = [0; deltaT*100; 0];
elseif index == 1*floor(1/deltaT)
    m = [0; deltaT*100; 0];
elseif index == 2*floor(1/deltaT)
    m = [deg2rad(45); deltaT*100; deg2rad(45)];
elseif index == 3*floor(1/deltaT)
    m = [0; deltaT*100; 0];
elseif index == 4*floor(1/deltaT)
    m = [deg2rad(45); 0; deg2rad(45)];
else
    m = [0; deltaT*100; 0];
end
