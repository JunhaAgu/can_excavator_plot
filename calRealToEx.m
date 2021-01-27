function data = calRealToEx(byte, data)
 % scale:   [rad] 0.000144 <-> [deg] 0.0083
 % offset:  [rad] -4.7124  <-> [deg] -270 
for i=1:size(byte,2)
   data(1,i) = 0.000144*(byte(2,i)*16^2 + byte(1,i)) -4.7124;
   data(2,i) = 3.06e-5*(byte(4,i)*16^2 + byte(3,i)) +2;
   data(3,i) = 3.82e-5*(byte(6,i)*16^2 + byte(5,i)) +2;
   data(4,i) = 3.06e-5*(byte(8,i)*16^2 + byte(7,i)) +1.5;
   data(5,i) = 9.16e-5*(byte(10,i)*16^2 + byte(9,i)) -3;
   data(6,i) = 9.16e-5*(byte(12,i)*16^2 + byte(11,i)) -3;
   data(7,i) = 9.16e-5*(byte(14,i)*16^2 + byte(13,i)) -3;
   data(8,i) = 9.16e-5*(byte(16,i)*16^2 + byte(15,i)) -3;
   data(9,i) = 3.86*(byte(18,i)*16^2 + byte(17,i)) -126309;
   data(10,i) = 39.72*(byte(20,i)*16^2 + byte(19,i)) -855617;
   data(11,i) = 22.38*(byte(22,i)*16^2 + byte(21,i)) -463459;
   data(12,i) = 18.26*(byte(24,i)*16^2 + byte(23,i)) -384181;
   
   data(13,i) = 0.000144*(byte(26,i)*16^2 + byte(25,i)) -4.7124;
   data(14,i) = 3.06e-5*(byte(28,i)*16^2 + byte(27,i)) +2;
   data(15,i) = 3.82e-5*(byte(30,i)*16^2 + byte(29,i)) +2;
   data(16,i) = 3.06e-5*(byte(32,i)*16^2 + byte(31,i)) +1.5;
   data(17,i) = 9.16e-5*(byte(34,i)*16^2 + byte(33,i)) -3;
   data(18,i) = 9.16e-5*(byte(36,i)*16^2 + byte(35,i)) -3;
   data(19,i) = 9.16e-5*(byte(38,i)*16^2 + byte(37,i)) -3;
   data(20,i) = 9.16e-5*(byte(40,i)*16^2 + byte(39,i)) -3;
   data(21,i) = 3.86*(byte(42,i)*16^2 + byte(41,i)) -126309;
   data(22,i) = 39.72*(byte(44,i)*16^2 + byte(43,i)) -855617;
   data(23,i) = 22.38*(byte(46,i)*16^2 + byte(45,i)) -463459;
   data(24,i) = 18.26*(byte(48,i)*16^2 + byte(47,i)) -384181;
   
   data(25,i) = 00.000144*(byte(50,i)*16^2 + byte(49,i)) -4.7124;
   data(26,i) = 3.06e-5*(byte(52,i)*16^2 + byte(51,i)) +2;
   data(27,i) = 3.82e-5*(byte(54,i)*16^2 + byte(53,i)) +2;
   data(28,i) = 3.06e-5*(byte(56,i)*16^2 + byte(55,i)) +1.5;
   data(29,i) = 9.16e-5*(byte(58,i)*16^2 + byte(57,i)) -3;
   data(30,i) = 9.16e-5*(byte(60,i)*16^2 + byte(59,i)) -3;
   data(31,i) = 9.16e-5*(byte(62,i)*16^2 + byte(61,i)) -3;
   data(32,i) = 9.16e-5*(byte(64,i)*16^2 + byte(63,i)) -3;
   data(33,i) = 3.86*(byte(66,i)*16^2 + byte(65,i)) -126309;
   data(34,i) = 39.72*(byte(68,i)*16^2 + byte(67,i)) -855617;
   data(35,i) = 22.38*(byte(70,i)*16^2 + byte(69,i)) -463459;
   data(36,i) = 18.26*(byte(72,i)*16^2 + byte(71,i)) -384181;
end