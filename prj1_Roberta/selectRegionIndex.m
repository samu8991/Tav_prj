function [nonzeroI] = selectRegionIndex(nonzerox, nonzeroy, xl, xh, yl, yh)
nonzeroI = [];

x = size(nonzerox);
if(x(1) ~= 0)
for i = 1 : x(2)
    if(nonzerox(i) > xh || nonzerox(i)<xl || nonzeroy(i) > yh || nonzeroy(i) < yl)
        nonzerox(i) = 0;
        nonzeroy(i) = 0;
    end
end

[nonzeroI, ~] = nonzero(cat(1,nonzerox, nonzeroy), x(2), 2);
end
end

