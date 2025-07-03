# Abreviaturas y definiciones
- **FPGA**: Field Programmable Gate Arrays

# Referencias
[0] David Harris y Sarah Harris. *Digital Design and Computer Architecture. RISC-V Edition.* Morgan Kaufmann, 2022. ISBN: 978-0-12-820064-3

[1] Features, 1., & Description, 3. (s/f). SNx4HC14 Hex Inverters with Schmitt-Trigger Inputs. Www.ti.com. https://www.ti.com/lit/ds/symlink/sn74hc14.pdf?ts=1709130609427&ref_url=https%253A%252F%252Fwww.google.com%252F

[2] Molina, C. (s.f)

# Pipeline
Un procesador tipo pipeline es un sistema que puede realizar más de un tarea al mismo tiempo, debido a que, se separan por registros permitiendo guardar la infromación en cada una de las etapas y enviarlas a las siguientes etapas. Como cada una de las etapas necesita un registro, el tamaño de hardware aumenta, además de aumentar su complejidad a comparación de un uniciclo o multiciclo, no obstante, se obtiene una mejoria en su rendimiento y velocidad del sistema. El controlador del sistema es una parte indispensable del proceso, porque es el encargado de enviar los datos correctos a cada una de los partes del pipeline. Dependiendo del pipeline este puede ser mas complejo realizando tarea como jumps, xor, beq, jal, entre otras instrucciones.

## Descripción de módulos utilizados y descripción de su funcionamiento:

###Pasos para la implementación del pipeline:

Etapa IF (Instruction Fetch)
Esta etapa hace:
-Selección del próximo PC (mux_4_1)
-Registro del PC actual (pc_register)
-Cálculo de pc +4 (adder)
-Acceso a la memoria de instrucciones (inst_mem)

Pasos:

####1) Crear el registro inter-etapa IF/ID 
Este registro va a guardar el resultado de IF para que la siguiente etapa (ID) lo use en el siguiente ciclo de reloj.

####2)Etapa ID/EX
Se crea un registro inter-etapa ID/EX, donde se guardan los datos necesarios para la etapa EX, se usan esos nuevos tags en la etapa EX.
2.1: Se agrega el registro ID/EX
2.2: Se modifican conexiones nuevas

####3)Etapa EX/MEM
Se crea el registro EX/EM que almacena los resultados de la etapa EX, para que luego la etapa MEM los use.
3.1 Se agrega el registro EX/MEM
3.2 Se conecta data_mem con los nuevos registros

####4)Etapa MEM/WB 
Se guarda en el registro MEM/WB los datos que vienen de la memoria o la Alu, y se pasan a la etapa final (escritura en registros). 
4.1 Se agrega una modificación extra al registro ID/EX 
4.2 Se crea el registro MEM/WB
4.3 Se agregan o conectan las nuevas señales

###Pasos para la prueba de la implementación Pipeline:

####1) Crear un .mem con instruccciones (program.mem)
####2) Se modifica inst_mem.sv para leer .mem
####3) Crear un testbench (top_pipeline_tb.sv)

## Modulos usados
### Mux: que permite la salida de uno u otra entrada parametrizados.

module mux_4_1 #(parameter WIDTH=32) (

  input logic  [WIDTH-1:0] A,
  input logic  [WIDTH-1:0] B,
  input logic  [WIDTH-1:0] C,
  input logic  [WIDTH-1:0] D,
  input logic  [1:0]       sel,
  
  output logic [WIDTH-1:0] out

);
  
  always_comb begin
    case(sel)
      'h0: out = A;
      'h1: out = B;
      'h2: out = C;
      'h3: out = D;
    endcase
  end
  
endmodule

### Adder simple: para la suma del PC y cambios de en registro/memoria
```
module adder #(parameter WIDTH=32) (
	
  input logic  [WIDTH-1:0] A,
  input logic  [WIDTH-1:0] B,
  
  output logic [WIDTH-1:0] out

);
  
  assign out = A + B;
  
endmodule

### Registro: simple y parametrizable para guardar las direcciones y sus datos

module register #(parameter WIDTH=32) (
	
  input logic			   clk,
  input logic              rst,
  input logic  [WIDTH-1:0] data_in,
  input logic              wr,
  
  output logic [WIDTH-1:0] data_out

);
  
  always_ff @(posedge clk) begin
    if      (rst) data_out <= 0;
    else if (wr)  data_out <= data_in;
    else          data_out <= data_out;
  end
  
endmodule
```
### Registro RISC-V: se usa para los documentos del RISC-V, instrucciones del registro y entendimiento futuro del control.
```
module reg_file #(parameter WIDTH=32, parameter DEPTH=5) (
	
  input logic  	   	      clk,
  input logic             rst,
  input logic [WIDTH-1:0] write_data,
  input logic [DEPTH-1:0] write_register, //addr
  input logic             wr,
  input logic [DEPTH-1:0] read_register_1, //addr
  input logic [DEPTH-1:0] read_register_2, //addr
  input logic             rd,
  
  output logic [WIDTH-1:0] read_data_1,
  output logic [WIDTH-1:0] read_data_2

);
  
  logic [WIDTH-1:0] registers [2**DEPTH];
  logic [WIDTH-1:0] reg_zero;
  
  assign reg_zero = 0;
  
  always_ff @(negedge clk or posedge rst) begin
    if (rst) begin
      for(int i=1; i < 2**DEPTH;i++) begin
        $display("Here!!!!!");
        registers[i] <= 0;
      end
    end
    else if ((wr) && (write_register != 0)) registers[write_register] <= write_data;
  end
  
  always_comb begin
    if (rd) begin
        if   (read_register_1 == 0) read_data_1 = reg_zero;
        else                        read_data_1 = registers[read_register_1];
        if   (read_register_2 == 0) read_data_2 = reg_zero;
        else                        read_data_2 = registers[read_register_2];
    end
    else begin
      read_data_1 = 'hz;
      read_data_2 = 'hz;
    end
  end
  
endmodule
```
### Alu: Para el uso de aritmetico o logico de algunas instrucciones como el add o el and.
```
module alu #(parameter WIDTH=32) (
	
  input logic  [WIDTH-1:0] data_in_1,
  input logic  [WIDTH-1:0] data_in_2,
  input logic  [2:0]       func3,
  input logic  [6:0]       func7,
  input logic  [6:0]       opcode,
  
  output logic [WIDTH-1:0] data_out,
  //output logic           carry,
  output logic             zero,
  output logic             comparison

);
  
  parameter ALI_OP    = 7'b0010011;
  parameter AL_OP     = 7'b0110011;
  parameter MEM_WR_OP = 7'b0100011;
  parameter MEM_RD_OP = 7'b0000011;
  parameter BR_OP     = 7'b1100011;
  parameter JALR      = 7'b1100111;
  parameter LUI       = 7'b0110111;
  parameter AUIPC     = 7'b0010111;
  
  
  always_comb begin
    case(opcode)
      ALI_OP: begin
        case(func3)
          3'b000: data_out = signed'(data_in_1) + signed'(data_in_2); //addi
          3'b001: data_out = data_in_1 << data_in_2; //slli
          3'b010: data_out = (signed'(data_in_1) < signed'(data_in_2)) ? 1 : 0; //slti
          3'b011: data_out = (unsigned'(data_in_1) < unsigned'(data_in_2)) ? 1 : 0; //sltiu
          3'b100: data_out = data_in_1 ^ data_in_2; //xori
          3'b101: begin
            if(func7 == 7'b0000000) data_out = data_in_1 >> data_in_2; //srli
            if(func7 == 7'b0100000) data_out = signed'(data_in_1) >>> data_in_2; //srai
          end
          3'b110: data_out = data_in_1 | data_in_2; //ori
          3'b111: data_out = data_in_1 & data_in_2; //andi
        endcase
        comparison = 0;
      end
      AL_OP: begin
        case(func3)
          3'b000: begin
            if(func7 == 7'b0000000) data_out = signed'(data_in_1) + signed'(data_in_2); //add
            if(func7 == 7'b0100000) data_out = signed'(data_in_1) - signed'(data_in_2); //sub
          end
          3'b001: data_out = data_in_1 << data_in_2; //sll
          3'b010: data_out = (signed'(data_in_1) < signed'(data_in_2)) ? 1 : 0; //slt
          3'b011: data_out = (unsigned'(data_in_1) < unsigned'(data_in_2)) ? 1 : 0; //sltu
          3'b100: data_out = data_in_1 ^ data_in_2; //xor
          3'b101: begin
            if(func7 == 7'b0000000) data_out = data_in_1 >> data_in_2; //srl
            if(func7 == 7'b0100000) data_out = signed'(data_in_1) >>> data_in_2; //sra
          end
          3'b110: data_out = data_in_1 | data_in_2; //or
          3'b111: data_out = data_in_1 & data_in_2; //and
        endcase
        comparison = 0;
      end
      MEM_WR_OP: begin
        data_out = signed'(data_in_1) + signed'(data_in_2); //addr for sb, sh, sw
        comparison = 0;
      end
      MEM_RD_OP: begin
        data_out = signed'(data_in_1) + signed'(data_in_2); //addr for lb, lh, lw, lbu, lhu
        comparison = 0;
      end
      BR_OP: begin
        case(func3)
          3'b000: comparison = (signed'(data_in_1) == signed'(data_in_2)) ? 1 : 0; //beq
          3'b001: comparison = (signed'(data_in_1) == signed'(data_in_2)) ? 0 : 1; //bne
          3'b100: comparison = (signed'(data_in_1) <  signed'(data_in_2)) ? 1 : 0; //blt
          3'b101: comparison = (signed'(data_in_1) >= signed'(data_in_2)) ? 1 : 0; //bge
          3'b110: comparison = (unsigned'(data_in_1) <  unsigned'(data_in_2)) ? 1 : 0; //bltu
          3'b111: comparison = (unsigned'(data_in_1) >= unsigned'(data_in_2)) ? 1 : 0; //bgeu
        endcase
      end
      JALR: begin
        //jal instruction doesn't need the ALU
        data_out = signed'(data_in_1) + signed'(data_in_2); //jalr
        comparison = 0;
      end
      LUI: begin
        data_out = data_in_2; //lui
        comparison = 0;
      end
      AUIPC: begin
        data_out = signed'(data_in_1) + signed'(data_in_2); //auipc
        comparison = 0;
      end
      default: data_out = 0;
    endcase
  end
  
  assign zero = (data_out == 0) ? 1 : 0; //zero logic
  
endmodule
```
### Data Memory File: Data Memory. Uso de memoria del sistema registra y borra conforme entran datos.
```
module data_mem #(parameter WIDTH=32, parameter DEPTH=20) (

  input logic  	    	   clk,
  input logic              rst,
  input logic  [WIDTH-1:0] data_in,
  input logic  [DEPTH-1:0] addr,
  input logic              wr,
  input logic              rd,
  input logic              one_byte, //for sb, lb, lbu
  input logic              two_bytes, //for sh, lh, lhu
  input logic              four_bytes, //for sw, lw
  
  output logic [WIDTH-1:0] data_out

);
  
  logic [7:0] memory [2**DEPTH];

  always_ff @(negedge clk) begin
    if (rst) begin
      for(int i=1; i< 2**DEPTH;i++) begin
        memory[i] <= 0;
      end
    end
    else begin
      if (wr) begin
        if (one_byte) begin
          memory[addr]   <= data_in[7:0];
        end
        if (two_bytes) begin
          memory[addr]   <= data_in[7:0];
          memory[addr+1] <= data_in[16:8];
        end
        if (four_bytes) begin
          memory[addr]   <= data_in[7:0];
          memory[addr+1] <= data_in[15:8];
          memory[addr+2] <= data_in[23:16];
          memory[addr+3] <= data_in[31:24];
        end
      end
    end
  end
  
  always_comb begin
    if (rd) begin
      if (one_byte) begin
        data_out <= {24'b0,memory[addr]};
      end
      if (two_bytes) begin
        data_out <= {16'b0,memory[addr+1],memory[addr]};
      end
      if (four_bytes) begin
        data_out <= {memory[addr+3],memory[addr+2],memory[addr+1],memory[addr]};
      end
    end
  end
  
endmodule

### //Instruction Memory File: Instruction Memory

module inst_mem #(parameter WIDTH=32, parameter DEPTH=16) (
 
`ifdef MULTICYCLE
  input logic  	    	   clk,
`endif
  input logic              rst,
  input logic  [WIDTH-1:0] data_in,
  input logic  [DEPTH-1:0] addr,
  input logic              wr,
  input logic              rd,
  
  output logic [WIDTH-1:0] data_out

);
  
  logic [7:0] memory [2**DEPTH];
  
  initial $readmemh("program.mem", memory);
  
`ifdef MULTICYCLE 
  always_ff @(posedge clk) if (wr) memory[addr] <= data_in;
`endif
  
  assign data_out = rd ? {memory[addr+3],memory[addr+2],memory[addr+1],memory[addr]} : 'hz;
  
endmodule
```
### Immediate Generator: su uso es exclusivo para las instrucciones que necesitan un numero natural (numero inmediato) para su realización como el addi.
```
module imm_gen #(parameter WIDTH=32) (
	
  input logic  [WIDTH-1:0] instr,
  
  output logic [WIDTH-1:0] data_out

);
  
  parameter ALI_OP    = 7'b0010011;
  parameter MEM_WR_OP = 7'b0100011;
  parameter MEM_RD_OP = 7'b0000011;
  parameter BR_OP     = 7'b1100011;
  parameter JALR      = 7'b1100111;
  parameter JAL       = 7'b1101111;
  parameter LUI       = 7'b0110111;
  parameter AUIPC     = 7'b0010111;
  
  logic [6:0] opcode;
  logic [6:0] func3;
  
  assign opcode = instr[6:0];
  assign func3 = instr[14:12];
  
  always_comb begin
    case(opcode)
      ALI_OP: begin
        if   (func3 != (001 || 101)) data_out = {{19{instr[31]}},instr[31:20]};
        else                         data_out = {27'b0,instr[24:20]};
      end
      MEM_WR_OP: begin
        data_out = {{19{instr[31]}},instr[31:25],instr[11:7]};
      end
      MEM_RD_OP: begin
        data_out = {{19{instr[31]}},instr[31:20]};
      end
      BR_OP: begin
        data_out = {{18{instr[31]}},instr[31],instr[7],instr[30:25],instr[11:8],1'b0};
      end
      JALR: begin
        data_out = {{19{instr[31]}},instr[31:20]};
      end
      JAL: begin
        data_out = {{12{instr[31]}},instr[31],instr[19:12],instr[20],instr[30:21],1'b0};
      end
      LUI: begin
        data_out = {instr[31:12],12'b0};
      end
      AUIPC: begin
        data_out = {instr[31:12],12'b0};
      end
      default: data_out = 0;
    endcase
  end
  
endmodule
```

### Control Decoder: Basicamente es el que controla todo el pipeline y su parte. Al entrar una istrucción este manda cada control a las partes para que cada ciclo de reloj le indique mediante los registro una orden. Asi se mantiene la instrucción por cada ciclo de reloj.

```
module control_deco #(parameter WIDTH=32, parameter INST_SIZE = 32) (
	
  input logic  [WIDTH-1:0] instr,
  input logic              comparison,
  
  output logic [1:0]	   if_mux_sel,
  output logic 			   ex_mux_sel,
  output logic [1:0]	   wb_mux_sel,
  output logic             reg_file_rd,
  output logic             reg_file_wr,
  output logic			   mem_read,
  output logic			   mem_write,
  output logic             one_byte,
  output logic			   two_bytes,
  output logic			   four_bytes
);
  
  parameter ALI_OP    = 7'b0010011;
  parameter AL_OP     = 7'b0110011;
  parameter MEM_WR_OP = 7'b0100011;
  parameter MEM_RD_OP = 7'b0000011;
  parameter BR_OP     = 7'b1100011;
  parameter JALR      = 7'b1100111;
  parameter JAL       = 7'b1101111;
  parameter LUI       = 7'b0110111;
  parameter AUIPC     = 7'b0010111;
  
  logic [6:0] opcode;
  logic [6:0] func3;
  
  assign opcode = instr[6:0];
  assign func3  = instr[14:12];
  
  always_comb begin
    case(opcode)
      ALI_OP: begin
        		if_mux_sel = 0;
        		ex_mux_sel = 1;
        		wb_mux_sel = 0;
        		reg_file_rd = 1;
        		reg_file_wr = 1;
        		mem_read    = 0;
        		mem_write   = 0;
                one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		  end
      AL_OP: begin
        		if_mux_sel = 0;
        		ex_mux_sel = 0;
        		wb_mux_sel = 0;
        		reg_file_rd = 1;
        		reg_file_wr = 1;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		 end
      MEM_WR_OP: begin
                	if_mux_sel = 0;
        			ex_mux_sel = 1;
        			wb_mux_sel = 0;
        			reg_file_rd = 1;
        			reg_file_wr = 0;
        			mem_read    = 0;
        			mem_write   = 1;
        			if (func3 == 000) begin
                      one_byte     = 1;
                      two_bytes    = 0;
                      four_bytes   = 0;
          			end
        			else if (func3 == 001) begin
                      one_byte     = 0;
                      two_bytes    = 1;
                      four_bytes   = 0;
          			end
                    else begin
                      one_byte     = 0;
                      two_bytes    = 0;
                      four_bytes   = 1;
          			end
      		     end
      MEM_RD_OP: begin
                	if_mux_sel = 0;
        			ex_mux_sel = 1;
        			wb_mux_sel = 1;
        			reg_file_rd = 1;
        			reg_file_wr = 1;
        			mem_read    = 1;
        			mem_write   = 0;
        			if ((func3 == 000) || (func3 == 100)) begin
                      one_byte     = 1;
                      two_bytes    = 0;
                      four_bytes   = 0;
          			end
        			else if ((func3 == 001) || (func3 == 101)) begin
                      one_byte     = 0;
                      two_bytes    = 1;
                      four_bytes   = 0;
          			end
                    else begin
                      one_byte     = 0;
                      two_bytes    = 0;
                      four_bytes   = 1;
          			end
      		     end
      BR_OP:  begin
        		if_mux_sel = (comparison == 1) ? 1 : 0;
        		ex_mux_sel = 0;
        		wb_mux_sel = 0;
        		reg_file_rd = 1;
        		reg_file_wr = 0;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		  end
      JALR: begin
        		if_mux_sel = 2;
        		ex_mux_sel = 1;
        		wb_mux_sel = 2;
        		reg_file_rd = 1;
        		reg_file_wr = 1;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		end
      JAL: begin
        		if_mux_sel = 1;
        		ex_mux_sel = 1;
        		wb_mux_sel = 2;
        		reg_file_rd = 1;
        		reg_file_wr = 1;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		end
      LUI: begin
        		if_mux_sel = 0;
        		ex_mux_sel = 1;
        		wb_mux_sel = 2;
        		reg_file_rd = 1;
        		reg_file_wr = 1;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		end
      AUIPC: begin
        		if_mux_sel = 0;
        		ex_mux_sel = 1;
        		wb_mux_sel = 3;
        		reg_file_rd = 1;
        		reg_file_wr = 1;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		end
      default: begin
        		if_mux_sel = 0;
                ex_mux_sel = 0;
        		wb_mux_sel = 0;
        		reg_file_rd = 0;
        		reg_file_wr = 0;
        		mem_read    = 0;
        		mem_write   = 0;
        		one_byte    = 0;
                two_bytes   = 0;
                four_bytes  = 0;
      		   end
    endcase
  end
  
endmodule
```

### Mux 2x1: Un mux pero de dos entradas.
```
module mux_2_1 #(parameter WIDTH=32) (

  input logic  [WIDTH-1:0] A,
  input logic  [WIDTH-1:0] B,
  input logic              sel,
  
  output logic [WIDTH-1:0] out

);
  
  /*esto de abajo es equivalente a 
  if(sel) out = A;
  else    out = B;
  */
  always_comb out = (!sel) ? A : B; //out = al primer argumento (A) si la condición antes del signo de pregunta es verdadera, si no out = al segundo argumento (B) 
  
endmodule
```
Finalmente
