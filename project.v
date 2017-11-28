/**************************************************************************/
/*  Stub for Project in EE382M - Dependable Computing
*/
/*
*/
/*  Do not change I/O names in main() module
*/
/*
*/
/**************************************************************************/
`define WIDTH 2

module main(A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2,X0,X1,X2,XC,XE0,XE1,
            Y0,Y1,Y2,YC,YE0,YE1);

input A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2;
output X0,X1,X2,XC,XE0,XE1,Y0,Y1,Y2,YC,YE0,YE1;

/*Your code here*/
wire [`WIDTH:0] A;
wire [`WIDTH:0] B;
wire [`WIDTH:0] Y;
wire [`WIDTH:0] X;

A = {A0, A1, A2};
B = {B0, B1, B2};
Y = {Y0, Y1, Y2};
X = {X0, X1, X2};

three_addr(A, B, Y, YC);
three_addr(A, B, X, XC);

   
endmodule

/*************************************************************
Check that the input is valid
sets e bit high if control or parity check fails
*/
module codeword_detect(a, b, c, p, e);
input [`WIDTH:0] a;
input [`WIDTH:0] b;
input [`WIDTH:0] c;
input p;
output [1:0] x;
output [1:0] y;

wire control, parity;

control_check(c, control);
parity_check(a, b, p, parity);

assign e = ~(control & parity);

endmodule
/*************************************************************
Check that C is valid - valid = 1
*/
module control_check(c, o);
input [`WIDTH:0] c;
output o;
wire one, two;
assign one = (c[0] ^ c[1]) & ~c[3];
assign two = c[2]&~c[1]&~c[0]
assign o = one | two;
endmodule

/*************************************************************
Check the parity of A, B, and P - the parity bit, 1 = valid
*/
module parity_check(a, b, p, o);
input [`WIDTH:0] a;
input [`WIDTH:0] b;
input p;
output o;

wire a_p, b_p, both_p

parity_tree(a, a_p);
parity_tree(b, b_p);

parity_tree({a_p, b_p, p}, o);

endmodule

/*************************************************************
Return parity of three bits
*/
module parity_tree(a, p);

input [`WIDTH:0] a;
output p

assign p = a[0] ^ a[1] ^ a[2];

endmodule


/*************************************************************
Compute three bit addition
*/
module three_addr( a, b, s, cout );/* 8 bit ripple carry adder made up of 8 one_bit_adder */

input [`WIDTH:0] a;
input [`WIDTH:0] b;


output[`WIDTH:0] s;
output           cout;

wire t1,t2,t3,t4,t5,t6,t7;
wire cin;
cin = 1'b0;
one_bit_adder a1(a[0],b[0],cin,s[0],t1);
one_bit_adder a2(a[1],b[1],t1,s[1],t2);
one_bit_adder a3(a[2],b[2],t2,s[2],cout);

endmodule

/*************************************************************
One bit adder
*/
module one_bit_adder(a0,b0,c0,s0,c1);

/* three inputs which are 1 bit each */
input a0;
input b0;
input c0;/* carry in */

/* two outputs which are 1 bit each */
output s0;/* sum */
output c1;/* carry out */

assign s0 = a0^b0^c0;
assign c1 = (a0&b0)|(b0&c0)|(c0&a0);

endmodule

