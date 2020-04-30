function x = wedge(X)
% wedge: se(3) -> R^6
x = [X(1:3,4); unskew(X(1:3,1:3))]
end