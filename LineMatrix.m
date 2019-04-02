function [ m ] = LineMatrix(m1, m2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    Ones = ones(1,length(m2));
    m = m1(1).*Ones;
    for i = 2:length(m1)
        m = [m; m1(i).*Ones];
    end
end

