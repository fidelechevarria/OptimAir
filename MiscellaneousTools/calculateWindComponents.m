function [windVelocityEarth] = calculateWindComponents(configuration)
    windVelocityEarth = [configuration.dynamics.windVel * cos(configuration.dynamics.windElevation) * cos(configuration.dynamics.windHeading);
                         configuration.dynamics.windVel * cos(configuration.dynamics.windElevation) * sin(configuration.dynamics.windHeading);
                         configuration.dynamics.windVel * sin(configuration.dynamics.windElevation)];
end

