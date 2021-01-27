function data = decodeFromEx(byte, data, angle_scale, angle_offset)

for i=1: size(data,2)
    data(1,i) =  0.1 * ( byte(2,i)*16^2 + byte(1,i) );
    data(2,i) =  0.1 * ( byte(4,i)*16^2 + byte(3,i) );
    data(3,i) =  0.1 * ( byte(6,i)*16^2 + byte(5,i) );
    data(4,i) =  0.1 * ( byte(8,i)*16^2 + byte(7,i) );
    
    data(5,i) =  0.1 * ( byte(10,i)*16^2 + byte(9,i) );
    data(6,i) =  0.1 * ( byte(12,i)*16^2 + byte(11,i) );
    data(7,i) =  0.1 * ( byte(14,i)*16^2 + byte(13,i) );
    data(8,i) =  0.1 * ( byte(16,i)*16^2 + byte(15,i) );
    
    data(9,i) =  pi/180*(angle_scale * ( byte(19,i)*16^4 + byte(18,i)*16^2 + byte(17,i) ) + angle_offset );
    data(10,i) =  pi/180*(angle_scale * ( byte(22,i)*16^4 + byte(21,i)*16^2 + byte(20,i) ) + angle_offset );
    data(11,i) =  pi/180*(angle_scale * ( byte(25,i)*16^4 + byte(24,i)*16^2 + byte(23,i) ) + angle_offset );
    data(12,i) =  pi/180*(angle_scale * ( byte(28,i)*16^4 + byte(27,i)*16^2 + byte(26,i) ) + angle_offset );
    data(13,i) =  pi/180*(angle_scale * ( byte(31,i)*16^4 + byte(30,i)*16^2 + byte(29,i) ) + angle_offset );
    data(14,i) =  pi/180*(angle_scale * ( byte(34,i)*16^4 + byte(33,i)*16^2 + byte(32,i) ) + angle_offset );
end

end