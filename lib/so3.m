% vecotor to so(3)

function twisted = so3(omega)
    twisted = [0, -omega(3), omega(2) ...
                omega(3), 0, -omega(1) ...
                -omega(2), omega(1), 0];   
end