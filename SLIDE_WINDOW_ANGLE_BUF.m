function arr = SLIDE_WINDOW_ANGLE_BUF(arr)

arr(4) = arr(3);
arr(3) = arr(2);
arr(2) = arr(1);

end