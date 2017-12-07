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
#100
B0 = 1;
PAR = 1;
C0 = 0;
C1 = 1;
#100
B1 = 1;
C1 = 0;
C2 = 1;
PAR = 0;
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

wire arg_ERROR_FIRST, arg_ERROR_FIRST_N;
wire arg_ERROR_SECOND, arg_ERROR_SECOND_N;
wire arg1_Error_FIRST, arg2_Error_FIRST;
wire arg1_Error_SECOND, arg2_Error_SECOND;
wire carry1, carry2, carry3, carry4;

wire cw_error1;
wire cw_error2;
wire [`WIDTH:0] negative_A_FIRST;
wire [`WIDTH:0] negative_B_FIRST;
wire [`WIDTH:0] negative_A_SECOND;
wire [`WIDTH:0] negative_B_SECOND;
wire [`WIDTH:0] negative_A_THIRD;
wire [`WIDTH:0] negative_B_THIRD;
wire [`WIDTH:0] negative_A_FOURTH;
wire [`WIDTH:0] negative_B_FOURTH;
wire [`WIDTH:0] ARG1_FIRST;
wire [`WIDTH:0] ARG2_FIRST;
wire [`WIDTH:0] ARG1_SECOND;
wire [`WIDTH:0] ARG2_SECOND;
wire [`WIDTH:0] ARG1_THIRD;
wire [`WIDTH:0] ARG2_THIRD;
wire [`WIDTH:0] ARG1_FOURTH;
wire [`WIDTH:0] ARG2_FOURTH;
wire [`WIDTH:0] OUT1;
wire [`WIDTH:0] OUT2;
wire [`WIDTH:0] OUT3;
wire [`WIDTH:0] OUT4;

wire two_rail_FIRST, two_rail_FIRST_N, two_rail_SECOND, two_rail_SECOND_N;

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
twos_comp tc3_SECOND(A, negative_A_SECOND);
twos_comp tc4_SECOND(B, negative_B_SECOND);
twos_comp tc5_FIRST(A, negative_A_THIRD);
twos_comp tc6_FIRST(B, negative_B_THIRD);
twos_comp tc7_SECOND(A, negative_A_FOURTH);
twos_comp tc8_SECOND(B, negative_B_FOURTH);

select_arg2 s1_FIRST({C2, C1, C0}, B, negative_B_FIRST, ARG2_FIRST);
select_arg1 s2_FIRST({C2, C1, C0}, A, negative_A_FIRST, ARG1_FIRST);
select_arg2 s3_SECOND({C2, C1, C0}, B, negative_B_SECOND, ARG2_SECOND);
select_arg1 s4_SECOND({C2, C1, C0}, A, negative_A_SECOND, ARG1_SECOND);
select_arg2 s5_FIRST({C2, C1, C0}, B, negative_B_THIRD, ARG2_THIRD);
select_arg1 s6_FIRST({C2, C1, C0}, A, negative_A_THIRD, ARG1_THIRD);
select_arg2 s7_SECOND({C2, C1, C0}, B, negative_B_FOURTH, ARG2_FOURTH);
select_arg1 s8_SECOND({C2, C1, C0}, A, negative_A_FOURTH, ARG1_FOURTH);

comp_three_bits arg_c1(ARG2_FIRST, ARG2_THIRD, arg2_Error_FIRST);
comp_three_bits arg_c2(ARG1_FIRST, ARG1_THIRD, arg1_Error_FIRST);
comp_three_bits arg_c3(ARG2_SECOND, ARG2_FOURTH, arg2_Error_SECOND);
comp_three_bits arg_c4(ARG1_SECOND, ARG1_FOURTH, arg1_Error_SECOND);
assign arg_ERROR_FIRST = arg2_Error_FIRST | arg1_Error_FIRST;
assign arg_ERROR_SECOND = arg2_Error_SECOND | arg1_Error_SECOND;
assign arg_ERROR_FIRST_N = ~arg_ERROR_FIRST;
assign arg_ERROR_SECOND_N = ~arg_ERROR_SECOND;

three_addr ta1(ARG1_FIRST, ARG2_FIRST, OUT1, carry1);
three_addr_complement ta2(ARG1_FIRST, ARG2_FIRST, OUT2, carry2);
three_addr ta3(ARG1_SECOND, ARG2_SECOND, OUT3, carry3);
three_addr_complement ta4(ARG1_SECOND, ARG2_SECOND, OUT4, carry4);

two_rail_tree tree_1(OUT1, OUT2, carry1, carry2, two_rail_FIRST, two_rail_FIRST_N);
two_rail_tree tree_2(OUT3, OUT4, carry3, carry4, two_rail_SECOND, two_rail_SECOND_N);


basic_two_rail two_rail_1(two_rail_FIRST, two_rail_FIRST_N, cw_error1, arg_ERROR_FIRST_N, YE0, YE1);
basic_two_rail two_rail_2(two_rail_SECOND, two_rail_SECOND_N, cw_error2, arg_ERROR_SECOND_N, XE0, XE1);

assign Y0 = OUT1[0];
assign Y1 = OUT1[1];
assign Y2 = OUT1[2];
assign YC = carry1;
assign X0 = OUT3[0];
assign X1 = OUT3[1];
assign X2 = OUT3[2];
assign XC = carry3;

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
*************************************************************/
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
*************************************************************/
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
*************************************************************/
module twos_comp(a, out);
input [`WIDTH:0] a;
output [`WIDTH:0] out;

wire [`WIDTH:0] negative;
wire cout;

assign negative = ~a;
three_addr ta(negative, 3'b001, out, cout);

endmodule

/*************************************************************
if c[1] is high, negative b, else b
*************************************************************/
module select_arg2(c, a, a_neg, out);
input [`WIDTH:0] a;
input [`WIDTH:0] a_neg;
input [`WIDTH:0] c;
output [`WIDTH:0] out;

assign out = (c[1]) ? a_neg:a;

endmodule

/*************************************************************
if c[2] is high, negative a, else a
*************************************************************/
module select_arg1(c, a, a_neg, out);
input [`WIDTH:0] a;
input [`WIDTH:0] a_neg;
input [`WIDTH:0] c;
output [`WIDTH:0] out;

assign out = (c[2]) ? a_neg:a;

endmodule

/*************************************************************
Two bit two to one mux
*************************************************************/
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
*************************************************************/
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
*************************************************************/
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
*************************************************************/
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
*************************************************************/
module parity_tree(a, p);

input [`WIDTH:0] a;
output p;

assign p = a[0] ^ a[1] ^ a[2];

endmodule


/*************************************************************
Compute three bit addition
*************************************************************/
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
Complement Compute three bit addition
*************************************************************/
module three_addr_complement( a, b, s, cout );/* 8 bit ripple carry adder made up of 8 one_bit_adder */

input [`WIDTH:0] a;
input [`WIDTH:0] b;


output[`WIDTH:0] s;
output           cout;

wire t1,t2,t3,t4,t5,t6,t7;
wire [`WIDTH:0] sum_orig;
wire carry_orig;

first_bit_adder a1(a[0],b[0],sum_orig[0],t1);
one_bit_adder a2(a[1],b[1],t1,sum_orig[1],t2);
one_bit_adder a3(a[2],b[2],t2,sum_orig[2],carry_orig);

assign s[`WIDTH:0] = ~sum_orig[`WIDTH:0];
assign cout = ~carry_orig;

endmodule

/*************************************************************
One bit adder
*************************************************************/
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

/*************************************************************
Two bit adder
*************************************************************/
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

/*************************************************************
First Bit Adder
*************************************************************/
module first_bit_adder(a0, b0, s0, c1);
input a0;
input b0;
output s0;/* sum */
output c1;/* carry out */

assign s0 = a0^b0;
assign c1 = a0&b0;
endmodule

/*************************************************************
Basic Two Rails
*************************************************************/
module basic_two_rail(a, aN, b, bN, out, outN);
input a;
input aN;
input b;
input bN;
output out;
output outN;

wire ab;
wire aNbN;
wire aNb;
wire abN;

assign ab = a&b;
assign aNbN = aN&bN;
assign aNb = aN&b;
assign abN = a&bN;

assign out = ab | aNbN;
assign outN = aNb | abN;
endmodule

/*************************************************************
Two Rails Tree
*************************************************************/
module two_rail_tree(a, b, c1, c2, out, outN);
input [`WIDTH:0] a;
input [`WIDTH:0] b;
input c1;
input c2;
output out;
output outN;

wire intermediate_a;
wire intermediate_aN;
wire intermediate_b;
wire intermediate_bN;

basic_two_rail two_rail_1(a[0], b[0], a[1], b[1], intermediate_a, intermediate_aN);
basic_two_rail two_rail_2(a[2], b[2], c1, c2, intermediate_b, intermediate_bN);

basic_two_rail two_rail_3(intermediate_a, intermediate_aN, intermediate_b, intermediate_bN, out, outN);
endmodule