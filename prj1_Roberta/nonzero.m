function [nonzerox,nonzeroy] = nonzero(matrix, w, h)
%NONZERO Summary of this function goes here
%   Detailed explanation goes here
c = 1;
nonzerox = [];
nonzeroy = [];

for i = 1 : h
    for j = 1 : w
        if(matrix(i, j) ~= 0)
            nonzerox(c) = j;
            nonzeroy(c) = i;
            c = c + 1;
        end
    end
end
end

