function [TAS] = velocityEarthAndWind2tas(velocityEarthNorm, pitch, yaw, windVelocityEarth)
    % Calculate velocity flat-Earth components
    TAS_components = [velocityEarthNorm * cos(pitch) * cos(yaw) - windVelocityEarth(1);
                      velocityEarthNorm * cos(pitch) * sin(yaw) - windVelocityEarth(2);
                      velocityEarthNorm * sin(pitch) - windVelocityEarth(3)];
    TAS = sqrt(TAS_components(1)^2+TAS_components(2)^2+TAS_components(3)^2);
end