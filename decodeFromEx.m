function data = decodeFromEx(byte, data, angle_scale, angle_offset)

Boom_Angle_buf = zeros(1,4);
Arm_Angle_buf = zeros(1,4);
Bkt_Angle_buf = zeros(1,4);
Swing_Angle_buf = zeros(1,4);

Boom_Rate_buf = zeros(1,3);
Arm_Rate_buf = zeros(1,3);
Bkt_Rate_buf = zeros(1,3);
Swing_Rate_buf = zeros(1,3);

for i=1: size(data,2)
    data(1,i) =  0.1 * ( byte(2,i)*16^2 + byte(1,i) );
    data(2,i) =  0.1 * ( byte(4,i)*16^2 + byte(3,i) );
    data(3,i) =  0.1 * ( byte(6,i)*16^2 + byte(5,i) );
    data(4,i) =  0.1 * ( byte(8,i)*16^2 + byte(7,i) );
    
    data(5,i) =  0.1 * ( byte(10,i)*16^2 + byte(9,i) );
    data(6,i) =  0.1 * ( byte(12,i)*16^2 + byte(11,i) );
    data(7,i) =  0.1 * ( byte(14,i)*16^2 + byte(13,i) );
    data(8,i) =  0.1 * ( byte(16,i)*16^2 + byte(15,i) );
    
%     data(9,i) =  pi/180*(angle_scale * ( byte(19,i)*16^4 + byte(18,i)*16^2 + byte(17,i) ) + angle_offset );
%     data(10,i) =  pi/180*(angle_scale * ( byte(22,i)*16^4 + byte(21,i)*16^2 + byte(20,i) ) + angle_offset );
%     data(11,i) =  pi/180*(angle_scale * ( byte(25,i)*16^4 + byte(24,i)*16^2 + byte(23,i) ) + angle_offset );
%     data(12,i) =  pi/180*(angle_scale * ( byte(28,i)*16^4 + byte(27,i)*16^2 + byte(26,i) ) + angle_offset );
%     data(13,i) =  pi/180*(angle_scale * ( byte(31,i)*16^4 + byte(30,i)*16^2 + byte(29,i) ) + angle_offset );
%     data(14,i) =  pi/180*(angle_scale * ( byte(34,i)*16^4 + byte(33,i)*16^2 + byte(32,i) ) + angle_offset );
    
    % [Degree]
    data(9,i) =  (angle_scale * ( byte(19,i)*16^4 + byte(18,i)*16^2 + byte(17,i) ) + angle_offset );
    data(10,i) =  (angle_scale * ( byte(22,i)*16^4 + byte(21,i)*16^2 + byte(20,i) ) + angle_offset );
    data(11,i) =  (angle_scale * ( byte(25,i)*16^4 + byte(24,i)*16^2 + byte(23,i) ) + angle_offset );
    data(12,i) =  (angle_scale * ( byte(28,i)*16^4 + byte(27,i)*16^2 + byte(26,i) ) + angle_offset );
    data(13,i) =  (angle_scale * ( byte(31,i)*16^4 + byte(30,i)*16^2 + byte(29,i) ) + angle_offset );
    data(14,i) =  (angle_scale * ( byte(34,i)*16^4 + byte(33,i)*16^2 + byte(32,i) ) + angle_offset );
    
    Boom_Angle_buf(1) = data(11,i);
    Arm_Angle_buf(1) = data(12,i);
    Bkt_Angle_buf(1) = data(13,i);
    Swing_Angle_buf(1) = data(14,i);
    
    Boom_Rate_buf = SLIDE_WINDOW_RATE_BUF(Boom_Rate_buf);
    Boom_Rate_buf(1) = BUTTERWORTH(Boom_Angle_buf, Boom_Rate_buf);
    data(15,i) = Boom_Rate_buf(1);
    
    Arm_Rate_buf = SLIDE_WINDOW_RATE_BUF(Arm_Rate_buf);
    Arm_Rate_buf(1) = BUTTERWORTH(Arm_Angle_buf, Arm_Rate_buf);
    data(16,i) = Arm_Rate_buf(1);
    
    Bkt_Rate_buf = SLIDE_WINDOW_RATE_BUF(Bkt_Rate_buf);
    Bkt_Rate_buf(1) = BUTTERWORTH(Bkt_Angle_buf, Bkt_Rate_buf);
    data(17,i) = Bkt_Rate_buf(1);
    
    Swing_Rate_buf = SLIDE_WINDOW_RATE_BUF(Swing_Rate_buf);
    Swing_Rate_buf(1) = BUTTERWORTH(Swing_Angle_buf, Swing_Rate_buf);
    data(18,i) = Swing_Rate_buf(1);
    
    Boom_Angle_buf = SLIDE_WINDOW_ANGLE_BUF(Boom_Angle_buf);
    Arm_Angle_buf = SLIDE_WINDOW_ANGLE_BUF(Arm_Angle_buf);
    Bkt_Angle_buf = SLIDE_WINDOW_ANGLE_BUF(Bkt_Angle_buf);
    Swing_Angle_buf = SLIDE_WINDOW_ANGLE_BUF(Swing_Angle_buf);
    
%     data(19,i) = byte(42,i)*16^14+byte(41,i)*16^12+byte(40,i)*16^10+byte(39,i)*16^8+byte(38,i)*16^6+byte(37,i)*16^4+byte(36,i)*16^2+byte(35,i);
%     data(20,i) = byte(50,i)*16^14+byte(49,i)*16^12+byte(48,i)*16^10+byte(47,i)*16^8+byte(46,i)*16^6+byte(45,i)*16^4+byte(44,i)*16^2+byte(43,i);
%     data(21,i) = byte(58,i)*16^14+byte(57,i)*16^12+byte(56,i)*16^10+byte(55,i)*16^8+byte(54,i)*16^6+byte(53,i)*16^4+byte(52,i)*16^2+byte(51,i);

%     hexStr = [num2str(dec2hex(byte(42))) num2str(dec2hex(byte(41))) num2str(dec2hex(byte(40))) num2str(dec2hex(byte(39))) num2str(dec2hex(byte(38))) num2str(dec2hex(byte(37))) num2str(dec2hex(byte(36))) num2str(dec2hex(byte(35)))];
%     data(19,i) = hex2num(hexStr); %Easting
%     data(19,i) = typecast(uint8(byte(35:42))', 'double'); %Easting
    data(19,i) = bitOperation(byte(35:42)); % Easting
%     hexStr = [num2str(dec2hex(byte(50))) num2str(dec2hex(byte(49))) num2str(dec2hex(byte(48))) num2str(dec2hex(byte(47))) num2str(dec2hex(byte(46))) num2str(dec2hex(byte(45))) num2str(dec2hex(byte(44))) num2str(dec2hex(byte(43)))];
%     data(20,i) = hex2num(hexStr); %Northing
%     data(20,i) = typecast(uint8(byte(43:50))', 'double'); %Northing
    data(20,i) = bitOperation(byte(43:50)); % Northing
%     hexStr = [num2str(dec2hex(byte(58))) num2str(dec2hex(byte(57))) num2str(dec2hex(byte(56))) num2str(dec2hex(byte(55))) num2str(dec2hex(byte(54))) num2str(dec2hex(byte(53))) num2str(dec2hex(byte(52))) num2str(dec2hex(byte(51)))];
%     data(21,i) = hex2num(hexStr); %Elevation
%     data(21,i) = typecast(uint8(byte(51:58))', 'double'); % Elevation
    data(21,i) = bitOperation(byte(51:58)); % Elevation
    
    data(22,i) = (1e-7)*(byte(62,i)*16^6+byte(61,i)*16^4+byte(60,i)*16^2+byte(59,i)) - 210; % Latitude
    data(23,i) = (1e-7)*(byte(66,i)*16^6+byte(65,i)*16^4+byte(64,i)*16^2+byte(63,i)) - 210; % Longitude
    data(24,i) = (0.125)*(byte(68,i)*16^2+byte(67,i)) - 2500; % Sea_Level

end

end