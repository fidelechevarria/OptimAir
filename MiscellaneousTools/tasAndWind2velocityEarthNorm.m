function [velocityEarthNorm] = tasAndWind2velocityEarthNorm(TAS, pitch, yaw, windVelocityEarth)
    % Calculate velocity flat-Earth components
    velocityEarth = [TAS * cos(pitch) * cos(yaw) + windVelocityEarth(1);
                     TAS * cos(pitch) * sin(yaw) + windVelocityEarth(2);
                     TAS * sin(pitch) + windVelocityEarth(3)];
    velocityEarthNorm = sqrt(velocityEarth(1)^2+velocityEarth(2)^2+velocityEarth(3)^2);
end