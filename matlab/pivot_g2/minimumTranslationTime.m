function duration = minimumTranslationTime(lengthValue,startSpeed,endSpeed,robot)
%MINIMUMTRANSLATIONTIME Thoi gian toi thieu voi gioi han v, tang/giam toc.
if ~isfinite(lengthValue)||lengthValue<0||startSpeed<0||endSpeed<0|| ...
        startSpeed>robot.maxLinearSpeed+1e-12|| ...
        endSpeed>robot.maxLinearSpeed+1e-12
    duration=inf;return;
end
if lengthValue<=1e-12
    if startSpeed<=1e-12&&endSpeed<=1e-12,duration=0;else,duration=inf;end
    return;
end
a=robot.maxLinearAcceleration;d=robot.maxLinearDeceleration;
if endSpeed>startSpeed
    minimumLength=(endSpeed^2-startSpeed^2)/(2*a);
else
    minimumLength=(startSpeed^2-endSpeed^2)/(2*d);
end
if minimumLength>lengthValue+1e-12,duration=inf;return;end
maximum=robot.maxLinearSpeed;
accelerationLength=(maximum^2-startSpeed^2)/(2*a);
decelerationLength=(maximum^2-endSpeed^2)/(2*d);
if accelerationLength+decelerationLength<=lengthValue
    cruiseLength=lengthValue-accelerationLength-decelerationLength;
    duration=(maximum-startSpeed)/a+cruiseLength/maximum+ ...
        (maximum-endSpeed)/d;
else
    peakSquared=(2*a*d*lengthValue+d*startSpeed^2+a*endSpeed^2)/(a+d);
    peak=sqrt(max(0,peakSquared));
    if peak+1e-12<max(startSpeed,endSpeed),duration=inf;return;end
    duration=(peak-startSpeed)/a+(peak-endSpeed)/d;
end
end
