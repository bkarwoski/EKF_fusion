function x = wedge(X)
%NOTE: I don't think this function is needed -bk
% wedge: se(3) -> R^6
x = [X(1:3,4); unskew(X(1:3,1:3))];
end