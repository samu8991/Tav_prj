function [nonzero] = nonZeroWindow(matrix, xl, xh, yl, yh)
%NONZEROWINDOW Summary of this function goes here
%   Detailed explanation goes here
nonzero = [];
c = 1;
for i = yl : yh
    for j = xl : xh
        if(matrix(i, j) == 255 || matrix(i, j) == 1)
            nonzero(c) = j;
            c = c + 1;
        end
    end
end

end

