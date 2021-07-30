function result = BUTTERWORTH(abuf, rbuf)

result = 1.691 * rbuf(2) - 0.7327 * rbuf(3) ...
        + 1.04 * (abuf(1) - abuf(2)) + 2.09 * (abuf(2) - abuf(3)) + 1.04 * (abuf(3) - abuf(4));

end