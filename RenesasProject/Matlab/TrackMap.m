track = imread('E:\Krassen Angelov\Programming\Renesas\RenesasProject\SolidWorks\Project\Rendering\OurRenesasTrackModel\OnlyTheFollowingLineOmega.JPG');
tmp = im2double(track);
zAxis = 3;
xAxis = 1566;
yAxis = 783;
for z = 1:zAxis 
    for x = 1:xAxis
        for y = 1:yAxis
            if tmp(y, x, z) > 0.9
                tmp(y, x, z) = 1;
            else
                tmp(y, x, z) = 0;
            end
        end
    end
end  
data = and(tmp(:,:,1),tmp(:,:,2));
and(data(:,:),tmp(:,:,3));
image(data)
data = double(data)

% sensorCoordinateZ = [4 174 173 172 171 170 169 168 167]
% sensorCoordinateX = [3 305 304 303 302 301 300 299 298]



