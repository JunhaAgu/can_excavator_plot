function result = bitOperation(byte)
    % [byte(8) byte(7) byte(6) byte(5) byte(4) byte(3) byte(2) byte(1)]
    byte = uint8(byte);
    sign = bitshift(byte(8),-7);
    
    exp_temp = uint16(byte(8));
    exponent = bitshift(exp_temp, 4) + uint16(bitshift(byte(7),-4));
    exp = 2^(double(exponent-1023));
    
    b7 = uint8(byte(7));
    b7_low = bitshift(bitshift(b7,4),-4);
    
%     sig = bitshift(b7_low,48) + bitshift(byte(6), 40) + bitshift(byte(5), 32) + bitshift(byte(4), 24) + bitshift(byte(3), 16) ...
%         + bitshift(byte(2), 8) + byte(1);
%     sig = 1+ sig/2^52;
    
    sig = 1 + double(b7_low)*2^-4 + double(byte(6))*2^-12 + double(byte(5))*2^-20 + double(byte(4))*2^-28 + double(byte(3))*2^-36 ...
        + double(byte(2))*2^-44 + double(byte(1))*2^-52;
    
    result = exp * sig;
    
    if sign>1
        result = -result;
    end
end