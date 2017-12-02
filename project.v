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
/*
module basic_tb();
reg A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2;
wire X0,X1,X2,XC,XE0,XE1,Y0,Y1,Y2,YC,YE0,YE1;

main m(A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2,X0,X1,X2,XC,XE0,XE1,Y0,Y1,Y2,YC,YE0,YE1);

initial begin
A0 = 1;
A1 = 0;
A2 = 0;
B0 = 0;
B1 = 0;
B2 = 0;
PAR = 1;
C0 = 1;
C1 = 0;
C2 = 0;
#100
B0 = 1;
PAR = 0;
C0 = 0;
C1 = 1;
#100
B1 = 1;
C1 = 0;
C2 = 1;
PAR = 1;
#100
$finish;
end

endmodule
*/
module main(A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2,X0,X1,X2,XC,XE0,XE1,
            Y0,Y1,Y2,YC,YE0,YE1);

input A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2;
output X0,X1,X2,XC,XE0,XE1,Y0,Y1,Y2,YC,YE0,YE1;

/*Your code here*/
wire [`WIDTH:0] A;
wire [`WIDTH:0] B;
wire [`WIDTH:0] C;
wire [`WIDTH:0] Y;
wire [`WIDTH:0] X;

wire [1:0] YE;
wire [1:0] XE;

wire cw_error;
wire [`WIDTH:0] negative_A;
wire [`WIDTH:0] negative_B;
wire [`WIDTH:0] ARG1;
wire [`WIDTH:0] ARG2;

assign A = {A2, A1, A0};
assign B = {B2, B1, B0};
assign C = {C2, C1, C0};
assign Y = {Y2, Y1, Y0};
assign X = {X2, X1, X0};
assign YE = {YE1, YE0};
assign XE = {XE1, XE0};

twos_comp tc1(A, negative_A);
twos_comp tc2(B, negative_B);

codeword_detect cw(A, B, {C2, C1, C0}, PAR, cw_error);

two_bit_two_one_mux m1(2'b11, 2'b01, cw_error, {YE1, YE0});
two_bit_two_one_mux m2(2'b11, 2'b01, cw_error, {XE1, XE0});

select_arg2 s1({C2, C1, C0}, B, negative_B, ARG2);
select_arg1 s2({C2, C1, C0}, A, negative_A, ARG1);

three_addr ta1(ARG1, ARG2, {Y2, Y1, Y0}, YC);
three_addr ta2(ARG1, ARG2, {X2, X1, X0}, XC);

endmodule

/*************************************************************
Return Two's Complement of a three bit number FIX
*/
module twos_comp(a, out);
input [`WIDTH:0] a;
output [`WIDTH:0] out;

wire [`WIDTH:0]negative;
wire cout;

assign negative = ~a;
three_addr ta(negative, 3'b001, out, cout);

endmodule

/*************************************************************
if c[1] is high, negative b, else b
*/
module select_arg2(c, a, a_neg, out);
input [`WIDTH:0] a;
input [`WIDTH:0] a_neg;
input [`WIDTH:0] c;
output [`WIDTH:0] out;

assign out = (c[1]) ? a_neg:a;

endmodule

/*************************************************************
if c[2] is high, negative a, else a
*/
module select_arg1(c, a, a_neg, out);
input [`WIDTH:0] a;
input [`WIDTH:0] a_neg;
input [`WIDTH:0] c;
output [`WIDTH:0] out;

assign out = (c[2]) ? a_neg:a;

endmodule

/*************************************************************
Two bit two to one mux
*/
module two_bit_two_one_mux(a, b, sel, out);
input [1:0] a;
input [1:0] b;
input sel;
output [1:0] out;

assign out = (sel)? a:b;

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
output e;

wire control, parity;

control_check c1(c, control);
parity_check c2(a, b, p, parity);

assign e = ~(control & (~parity));

endmodule
/*************************************************************
Check that C is valid - valid = 1
*/
module control_check(c, o);
input [`WIDTH:0] c;
output o;
wire one, two;
assign one = (c[0] ^ c[1]) & ~c[2];
assign two = c[2]&~c[1]&~c[0];
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
wire a_p, b_p, both_p;
parity_tree p1(a, a_p);
parity_tree p2(b, b_p);
parity_tree p3({a_p, b_p, p}, o);
endmodule

/*************************************************************
Return parity of three bits
*/
module parity_tree(a, p);

input [`WIDTH:0] a;
output p;

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

first_bit_adder a1(a[0],b[0],s[0],t1);
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

module first_bit_adder(a0, b0, s0, c1);
input a0;
input b0;
output s0;/* sum */
output c1;/* carry out */

assign s0 = a0^b0;
assign c1 = a0&b0;
endmodule



