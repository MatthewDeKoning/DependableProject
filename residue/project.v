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
//residue test
reg [3:0] A;
reg [3:0] B;
wire [1:0] O;
wire [1:0] O2;
wire [2:0] SUM;
wire Cout;
input_residue i1(A, B, O);
three_addr a1({A[2], A[1], A[0]}, {B[2], B[1], B[0]}, SUM, Cout);
residue r1({Cout,SUM}, O2);

initial begin
A = 0;
B = 3; 
#200
A = 1;
#200
A = 2;
#200
A = 3;
#200
A = 4;
#200
A = 5;
#200
A = 6;
#200
A = 7;
#200
A = 0;
B = 7;
#200
A = 1;
#200
A = 2;
#200
A = 3;
#200
A = 4;
#200
A = 5;
#200
A = 6;
#200
A = 7;
#200
$finish;
end
//Main test
reg A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2;
wire X0,X1,X2,XC,XE0,XE1,Y0,Y1,Y2,YC,YE0,YE1;

main m(A0,A1,A2,B0,B1,B2,PAR,C0,C1,C2,X0,X1,X2,XC,XE0,XE1,Y0,Y1,Y2,YC,YE0,YE1);

initial begin
A0 = 0;
A1 = 1;
A2 = 0;
B0 = 0;
B1 = 0;
B2 = 0;
PAR = 0;
C0 = 1;
C1 = 0;
C2 = 0;
#200
B0 = 1;
PAR = 1;
C0 = 0;
C1 = 1;
#200
B1 = 1;
C1 = 0;
C2 = 1;
PAR = 0;
#200
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

wire E1, E2, E3, E4, E5, E6;
//wire E1, E2, E3, E4;
wire carry1, carry2, carry3, carry4;
//wire PA1, PA2, PC, PO;

wire cw_error1;
wire cw_error2;
wire [`WIDTH:0] negative_A_FIRST;
wire [`WIDTH:0] negative_B_FIRST;
wire [`WIDTH:0] negative_A_SECOND;
wire [`WIDTH:0] negative_B_SECOND;
wire [`WIDTH:0] negative_B_THIRD;
wire [`WIDTH:0] ARG1_FIRST;
wire [`WIDTH:0] ARG2_FIRST;
wire [`WIDTH:0] ARG1_SECOND;
wire [`WIDTH:0] ARG2_SECOND;
wire [`WIDTH:0] OUT1;
wire [`WIDTH:0] OUT2;
wire [`WIDTH:0] OUT3;
wire [`WIDTH:0] OUT4;
//wire [1:0] IR;
//wire [1:0] OR1;
//wire [1:0] OR2;


assign A = {A2, A1, A0};
assign B = {B2, B1, B0};
assign C = {C2, C1, C0};
assign Y = {Y2, Y1, Y0};
assign X = {X2, X1, X0};
assign YE = {YE1, YE0};
assign XE = {XE1, XE0};

codeword_detect cw1(A, B, {C2, C1, C0}, PAR, cw_error1);
codeword_detect cw2(A, B, {C2, C1, C0}, PAR, cw_error2);

twos_comp tc1_FIRST(A, negative_A_FIRST);
twos_comp tc2_FIRST(B, negative_B_FIRST);

twos_comp tc1_SECOND(A, negative_A_SECOND);
twos_comp tc2_SECOND(B, negative_B_SECOND);

select_arg2 s1_FIRST({C2, C1, C0}, B, negative_B_FIRST, ARG2_FIRST);
select_arg1 s2_FIRST({C2, C1, C0}, A, negative_A_FIRST, ARG1_FIRST);

select_arg2 s1_SECOND({C2, C1, C0}, B, negative_B_SECOND, ARG2_SECOND);
select_arg1 s2_SECOND({C2, C1, C0}, A, negative_A_SECOND, ARG1_SECOND);

comp_three_bits arg_c1(ARG2_FIRST, ARG2_SECOND, E5);
comp_three_bits arg_c2(ARG1_FIRST, ARG1_SECOND, E6);

three_addr ta1(ARG1_FIRST, ARG2_FIRST, OUT1, carry1);
three_addr ta2(ARG1_FIRST, ARG2_FIRST, OUT2, carry2);
three_addr ta3(ARG1_SECOND, ARG2_SECOND, OUT3, carry3);
three_addr ta4(ARG1_SECOND, ARG2_SECOND, OUT4, carry4);
//three_addr ta3(ARG1_FIRST, ARG2_FIRST, OUT3, carry3);
//three_addr ta4(ARG1_FIRST, ARG2_FIRST, OUT4, carry4);

comp_four_bits c1(OUT1, OUT2, carry1, carry2, E1);
comp_four_bits c2(OUT3, OUT4, carry3, carry4, E2);

assign E3 = cw_error1 | E1 | E5;
assign E4 = cw_error2 | E2 | E6;
//assign E3 = cw_error | E1;
//assign E4 = cw_error | E2;
assign Y0 = OUT1[0];
assign Y1 = OUT1[1];
assign Y2 = OUT1[2];
assign YC = carry1;
assign X0 = OUT3[0];
assign X1 = OUT3[1];
assign X2 = OUT3[2];
assign XC = carry3;

//input_residue i_r({1'b0, ARG1}, {1'b0, ARG2}, IR);
//residue r1({carry1, OUT1}, OR1);
//residue r2({carry3, OUT3}, OR2);

//comp_two_bits c3(IR, OR1, E5);
//comp_two_bits c4(IR, OR2, E6);

/*
parity_tree p1(ARG1, PA1);
parity_tree p2(ARG2, PA2);
parity_tree p3({C2, C1, C0}, PC);
parity_tree p4({PA1,PA2, PC}, PO);
*/

two_bit_two_one_mux m1(2'b11, 2'b01, E3, {YE1, YE0});
two_bit_two_one_mux m2(2'b11, 2'b01, E4, {XE1, XE0});

endmodule

module comp_two_bits(a, b, out);
input [1:0] a;
input [1:0] b;
output out;

wire one, two, three;

assign one = a[0] ^ b[0];
assign two = a[1] ^ b[1];
assign out = one | two;
endmodule
/*************************************************************
Three bit compare - one for error, zero for equal
*/
module comp_three_bits(a, b, out);
input [`WIDTH:0] a;
input [`WIDTH:0] b;
output out;

wire one, two, three;

assign one = a[0] ^ b[0];
assign two = a[1] ^ b[1];
assign three = a[2] ^ b[2];
assign out = one | two | three;
endmodule

/*************************************************************
Four bit compare - one for error, zero for equal
*/
module comp_four_bits(a, b, c1, c2, out);
input [`WIDTH:0] a;
input [`WIDTH:0] b;
input c1;
input c2;
output out;

wire one, two, three, four;

assign one = a[0] ^ b[0];
assign two = a[1] ^ b[1];
assign three = a[2] ^ b[2];
assign four = c1 ^ c2;
assign out = one | two | three | four;
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

assign e = ~(control & parity);

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
wire a_p, b_p;
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



module input_residue(A, B, O);
input [3:0] A;
input [3:0] B;
output [1:0] O;
wire [1:0] AR;
wire [1:0] BR;
wire [1:0] RR;
wire cout;

residue r1(A, AR);
residue r2(B, BR);
two_bit_adder a1(AR, BR, RR, 1'b0, cout);
residue r3({1'b0, cout, RR}, O);
endmodule

module residue(A, O);
input [3:0] A;
output [1:0] O;

wire cout;
wire not_used;
wire select;
wire [1:0] output1;
wire [1:0] output2;
two_bit_adder a1({A[3], A[2]}, {A[1], A[0]}, output1, 1'b0, cout);
two_bit_adder a2(output1, 2'b00, output2, cout, not_used);
assign select = output2[0]&output2[1];
assign O = (select)? 2'b00: output2;
endmodule

module two_bit_adder(A, B, S, Cin, Cout);
input [1:0] A;
input [1:0] B;
input Cin;
output [1:0] S;
output Cout;
wire C1;

one_bit_adder a1(A[0], B[0], Cin, S[0], C1);
one_bit_adder a2(A[1], B[1], C1, S[1], Cout);

endmodule

module first_bit_adder(a0, b0, s0, c1);
input a0;
input b0;
output s0;/* sum */
output c1;/* carry out */

assign s0 = a0^b0;
assign c1 = a0&b0;
endmodule



