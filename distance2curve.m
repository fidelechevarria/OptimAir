function [distance,closestPointInCurve] = distance2curve(curve,point)

    x = curve(:,1);
    y = curve(:,2);
    z = curve(:,3);

    distances = sqrt((x-point(1)).^2+...
                    (y-point(2)).^2+...
                    (z-point(3)).^2);
    
    [distance,pointIndex] = min(distances);
    closestPointInCurve = [x(pointIndex) y(pointIndex) z(pointIndex)];
    
end

