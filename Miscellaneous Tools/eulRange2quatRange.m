
function quatRange = eulRange2quatRange(rollRange, headingRange)

N = 101;
heading = linspace(deg2rad(headingRange(1)),deg2rad(headingRange(2)),N);
roll = linspace(deg2rad(rollRange(1)),deg2rad(rollRange(2)),N);

quat = zeros(N,N,4);
for i = 1:N
    for j = 1:N
        quat(i,j,:) = eul2quat([heading(j) 0 roll(i)],'ZYX');
    end
end

quatMaxVals = zeros(4,1);
quatMinVals = zeros(4,1);
quatRange = zeros(2,4);
for i = 1:4
    quatMaxVals(i) = max(max(quat(:,:,i)));
    quatMinVals(i) = min(min(quat(:,:,i)));
    quatRange(:,i) = [quatMaxVals(i) quatMinVals(i)];
end

end