def test(a, b, c):
    logic1 = ((a^b) & ~c)&1
    logic2 = (c& ~b & ~a)&1
    #return ~(logic1 | logic2)&1
    return a^b^c

for i in range(0, 8):
    a = i&1
    b = (i&2)>>1
    c = (i&4)>>2
    #d = (i&8)>>3
    print 'C1', a, 'C2', b, 'C3', c, 'Output: ', test(a,b,c)