`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Okan Sen
// 
// Create Date: 11/28/2019 11:56:08 AM
// Design Name: 
// Module Name: alutest
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module alu(input  logic [31:0] a, b, 
           input  logic [2:0]  alucont, 
           output logic [31:0] result,
           output logic zero,
	       output logic bgres);
	
	always_comb
    begin
		case (alucont)

            3'b010: result = a + b; // ADD
            3'b110: result = a - b; // SUB
            3'b000: result = a & b; // AND
            3'b001: result = a | b; // OR
            3'b111: result = a < b ? 32'b1: 32'b0; // SLT
            3'b100: result = a < b ? 32'b0: 32'b1; //SLT reverse
            default: result = 0;
		endcase
    end
	assign zero = (result == 0);
	assign bgres = (result);
  
endmodule

   
