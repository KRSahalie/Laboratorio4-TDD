# Abreviaturas y definiciones
- **FPGA**: Field Programmable Gate Arrays

# Referencias
[0] David Harris y Sarah Harris. *Digital Design and Computer Architecture. RISC-V Edition.* Morgan Kaufmann, 2022. ISBN: 978-0-12-820064-3

[1] Features, 1., & Description, 3. (s/f). SNx4HC14 Hex Inverters with Schmitt-Trigger Inputs. Www.ti.com. https://www.ti.com/lit/ds/symlink/sn74hc14.pdf?ts=1709130609427&ref_url=https%253A%252F%252Fwww.google.com%252F

# Ejercicio 1: FIFO
La solución al ejercicio consiste en implementar una FIFO a partir del IP Generator, con un ancho de palabra de 8 bits y un largo de 512 palabras. Además se activan señales de salida para indicar la FIFO llena, FIFO vacía y cantidad de palabras presentes.

## Módulos utilizados y descripción de su funcionamiento:

### A. Módulos utilizados (generados) 
### 1. IP de FIFO (fifo_generator)
Esta FIFO (First In, First Out) de 8 bits y 512 palabras, generada mediante un IP core, es un bloque de memoria temporal que permite almacenar datos de manera ordenada: el primer dato en entrar es el primero en salir.

```SystemVerilog
  // Instancia FIFO
  fifo_generator fifo_inst (
      .clk(clk_16MHz),
      .srst(srst),
      .din(din),
      .wr_en(wr_en),
      .rd_en(rd_en),
      .dout(dout),
      .full(full),
      .empty(empty),
      .data_count(data_count)
  );
```

#### Señales:
- **clk:** Señal de reloj para la operación de la FIFO
- **srst:** Reset síncrono. Pone la FIFO en estado inicial
- **din:** Entrada de datos de 8 bits
- **wr_en:** Habilita escritura
- **rd_en:** Habilita lectura
- **dout::** Salida de datos de 8 bits
- **full:** Indica que la FIFO está llena
- **empty:** Indica que la FIFO está vacía
- **data_count:** Número de palabras actualmente almacenadas en la FIFO (monitor de ocupación)


### 2. PLL Reductor de Frecuencia (clk_wiz_0)
Este módulo utiliza un PLL (Phase-Locked Loop) generado mediante IP para reducir la frecuencia de un reloj de entrada de 100 MHz a 16 MHz.

```SystemVerilog
//Clock instance
clk_wiz_0 clk_pll    (
    // Clock out ports
    .clk_16MHz(clk_16MHz),     // output clk_out1
    .reset(srst), // input reset
    .clk_100MHz(clk_100MHz)
    );      // input clk_in1
```
#### Señales:
- **clk_100MHz:** Entrada del reloj principal de 100 MHz
- **clk_16MHz:** Salida del reloj generado por el PLL a 16 MHz
- **reset:** Señal de reinicio del PLL, normalmente asociada al reset general del sistema

### B. Testbench

```SystemVerilog
`timescale 1ns/1ps

module tb_fifo;

  reg clk_16MHz;
  reg srst;
  reg wr_en;
  reg rd_en;
  reg [7:0] din;
  wire [7:0] dout;
  wire full;
  wire empty;
  wire [8:0] data_count;

  // Instancia del PLL (simulada aquí con reloj generado)
  initial clk_16MHz = 0;
  always #31.25 clk_16MHz = ~clk_16MHz; // 16 MHz => periodo 62.5ns

  // Instancia FIFO
  fifo_generator fifo_inst (
      .clk(clk_16MHz),
      .srst(srst),
      .din(din),
      .wr_en(wr_en),
      .rd_en(rd_en),
      .dout(dout),
      .full(full),
      .empty(empty),
      .data_count(data_count)
  );

  initial begin
    // Reset inicial
    srst = 1;
    wr_en = 0;
    rd_en = 0;
    din = 8'd0;
    #300; // un par de ciclos de reloj
    srst = 0;

    // Escritura de datos 0 a 9
    repeat (10) begin
        #100;
      @(posedge clk_16MHz);
      if (!full) begin
        wr_en = 1;
            #100;
        din = din + 1;
        $display("Escribiendo: %0d en tiempo %0t", din, $time);
            #100;
      end else begin
        wr_en = 0;
      end
      @(posedge clk_16MHz);
      wr_en = 0;
    end
    
    // Pausa breve antes de empezar lectura
    repeat (5) @(posedge clk_16MHz);
    #300;
    // Lectura de datos hasta que FIFO quede vacía
    while (!empty) begin
      rd_en = 1;
          #100;
      @(posedge clk_16MHz);
      $display("Leyendo en tiempo %0t, valor: %0d", $time, dout);
          #100;
      rd_en = 0;
    end
    #100;
    $finish;
  end

endmodule
```

Los resultados de la simulación se pueden visualizar en la siguiente imagen
![Testbench UART](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%201/Imagenes/TB1.1.png)

Como se observa en el testbench, cada dato de salida corresponde a su respectiva entrada, manteniendo el orden en que fueron ingresados (First In, First Out). Esto demuestra que la FIFO funciona correctamente, conservando la integridad y secuencia de los datos.

# Ejercicio 2: Interfaz UART

La solución al ejercicio consiste en implementar una interfaz UART, dos registros de datos, un registro de control y una FSM que se encargue de la entrada y recepción de datos a parte de las propias de la interfaz.

## Módulos utilizados y descripción de su funcionamiento:

## A. Registros de Datos y Control. 
Se crean registros de datos y control para la FSM del protocolo UART. El registro de datos va a almacenar los datos que se reciben o envían a través del protocolo serial y el registro de control va a almacenar los bits que controlan la transmisión. 

### 1. Registros de Datos:
```SystemVerilog
module data_register #(
  parameter integer WIDTH = 8
)(
  input logic clk,
  input logic write_enable,
  input logic [WIDTH-1:0] data_in,
  output logic [WIDTH-1:0] data_out
);
```
### 2. Señales
Las señales básicas del módulo y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Reloj del sistema
- **write_enable:** Habilita la esccritura de datos
- **data_in:** Datos de entrada (recepción o transmisión dependiendo de la instanciación del registro).

#### Señales de Salida:
- **data_out:** Datos de salida (recepción o transmisión dependiendo de la instanciación del registro).

### 3. Uso y criterios de diseño
El módulo implementa un Shift Register simple de salida y entrada paralela. Está sincronizado al reloj y controlado por una señal de habilitación de escritura. 

### 4. Código 

```SystemVerilog
module data_register #(
  parameter integer WIDTH = 8
)(
  input logic clk,
  input logic write_enable,
  input logic [WIDTH-1:0] data_in,
  output logic [WIDTH-1:0] data_out
);

  logic [WIDTH-1:0] register = '0; // Inicialización del registro a 0

  always_ff @(posedge clk) begin
    if (write_enable) begin
      register <= data_in;
    end
  end

  assign data_out = register;

endmodule
```

### 5. Testbench

```SystemVerilog
`timescale 1ns / 1ps
module data_register_tb #(
  parameter integer WIDTH = 8  // Define an appropriate width for testing
);

  // Inputs to the register
  logic clk;
  logic write_enable;
  logic [WIDTH-1:0] data_in;

  // Outputs from the register
  logic [WIDTH-1:0] data_out;

  // DUT instantiation
  data_register #( .WIDTH(WIDTH) ) dut (
    .clk(clk),
    .write_enable(write_enable),
    .data_in(data_in),
    .data_out(data_out)
  );

  // Clock generation
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // Clock period of 10 ns
  end

  // Stimulus generation (data_in and write_enable)
  initial begin
    data_in = '0;
    write_enable = '0;

    // Wait for initial clock cycles
    #30; 

    // Test case 1: Single write
    @(posedge clk);
    write_enable <= 1;
    data_in <= 8'hA5; // Example data
    @(posedge clk); // Wait for the next clock edge
    write_enable <= 0; // Disable write
    @(posedge clk); // Wait for the next clock edge

    // Test case 2: Multiple writes with delay
    repeat (5) begin // Repeat 5 times
      @(posedge clk);
      write_enable <= $random % 2; // Randomly enable write (0 or 1)
      data_in <= $random & {WIDTH{1'b1}}; // Generate random data with the correct width
      @(posedge clk); // Wait for the next clock edge
    end

    // Test case 3: Hold value when write_enable is 0
    @(posedge clk);
    write_enable <= 1;
    data_in <= 8'h3C;
    @(posedge clk);
    write_enable <= 0; // Disable write
    @(posedge clk);
    // Expect data_out to hold the value 8'h3C
    @(posedge clk);
    // Expect data_out to still be 8'h3C
    if (data_out !== 8'h3C) begin
      $display("Test failed at time %0d: data_out should hold 0x3C, but got %h", $time, data_out);
    end

    // Test case 4: Sequential writes
    @(posedge clk);
    write_enable <= 1;
    data_in <= 8'h55;
    @(posedge clk);
    data_in <= 8'hAA;
    @(posedge clk);
    data_in <= 8'hFF;
    @(posedge clk);
    write_enable <= 0; // Disable write

    // Final delay before finishing the simulation
    #20; 

    $finish;
  end

  // Monitor signals
  initial begin
    $monitor("Time: %0d, clk: %b, write_enable: %b, data_in: %h, data_out: %h", 
             $time, clk, write_enable, data_in, data_out);
  end

endmodule
```

Los resultados de la simulación se pueden visualizar en la siguiente imagen.
![Testbench UART](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%202/Imagenes/Data%20register%20tb.png)

### 2. Registro de Control:
```SystemVerilog
module control_register (
    input logic clk,             
    input logic rx_bit,          
    input logic tx_bit,        
    output logic [1:0] stored_bits  
);
```
### 2. Señales
Las señales básicas del módulo y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Reloj del sistema
- **rx_bit:** Bit de recepción.
- **tx_bit:** Bit de transmisión.

#### Señales de Salida:
- **stored_bits :** Datos de salida (Actualización de bits de recepción o transmisión).

### 3. Uso y criterios de diseño
El módulo implementa un Shift Register simple de salida y entrada paralela sincronizada al reloj. 

### 4. Código 

```SystemVerilog
module control_register (
    input logic clk,             
    input logic rx_bit,        
    input logic tx_bit,          
    output logic [1:0] stored_bits 
);

    logic [1:0] stored_bits_reg = 2'b00; 

    // Proceso de captura de los bits de control
    always_ff @(posedge clk) begin
        stored_bits_reg <= {rx_bit, tx_bit};
    end

    // Asignación de los bits almacenados a la salida
    assign stored_bits = stored_bits_reg;

endmodule
```

### 5. Testbench

```SystemVerilog
`timescale 1ns / 1ps

module control_register_tb;

    // Parámetros
    parameter CLK_PERIOD = 10; // Periodo del reloj en unidades de tiempo

    // Señales
    logic clk;
    logic rx_bit;
    logic tx_bit;
    logic [1:0] stored_bits;

    // Instancia del módulo bajo prueba (DUT)
    control_register dut (
        .clk(clk),
        .rx_bit(rx_bit),
        .tx_bit(tx_bit),
        .stored_bits(stored_bits)
    );

    // Generación de reloj
    initial begin
        clk = 1'b0;
        forever #CLK_PERIOD clk = ~clk;
    end

    // Estímulo de entrada
    initial begin
        // Establecer valores iniciales
        rx_bit = 1'b0;
        tx_bit = 1'b0;

        // Esperar un ciclo de reloj antes de cambiar las entradas
        #CLK_PERIOD;

        // Prueba 1: Cambiar rx_bit y tx_bit
        rx_bit = 1'b1;
        tx_bit = 1'b0;

        // Esperar un ciclo de reloj
        #CLK_PERIOD;

        // Prueba 2: Cambiar rx_bit y tx_bit
        rx_bit = 1'b0;
        tx_bit = 1'b1;

        // Esperar un ciclo de reloj
        #CLK_PERIOD;

        // Prueba 3: Cambiar rx_bit y tx_bit
        rx_bit = 1'b1;
        tx_bit = 1'b1;

        // Esperar un ciclo de reloj
        #CLK_PERIOD;

        // Prueba 4: Mantener rx_bit y tx_bit
        rx_bit = 1'b0;
        tx_bit = 1'b0;

        // Esperar un ciclo de reloj
        #CLK_PERIOD;

        // Prueba 5: Cambiar rx_bit y tx_bit rápidamente
        #1;
        rx_bit = 1'b1;
        tx_bit = 1'b0;
        #1;
        rx_bit = 1'b0;
        tx_bit = 1'b1;
        #1;
        rx_bit = 1'b1;
        tx_bit = 1'b1;

        // Esperar algunos ciclos de reloj para ver los cambios
        #CLK_PERIOD;

        // Finalizar la simulación después de un tiempo suficiente
        #100;
        $finish;
    end

    // Monitorear las salidas
    always @(posedge clk) begin
        $display("Tiempo: %0t, stored_bits: %b", $time, stored_bits);
    end
endmodule
```

Los resultados de la simulación se pueden visualizar en la siguiente imagen.
![Testbench UART](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%202/Imagenes/Control%20Register%20tb.png)

## B. Módulo UART tx  


UART significa "Universal Asynchronous Receiver/Transmitter" (Receptor/Transmisor Universal Asincrónico) y es un componente fundamental en la comunicación serie entre dispositivos electrónicos. En su esencia, la UART es un hardware que convierte datos paralelos (generalmente provenientes de un procesador o microcontrolador) en datos serie para la transmisión, y viceversa para la recepción.

La comunicación UART es asincrónica, lo que significa que no se requiere un reloj compartido entre el transmisor y el receptor. En lugar de eso, se utiliza un protocolo que incluye bits de inicio y parada, junto con los datos mismos, para sincronizar la comunicación. Los datos se envían uno tras otro en forma de "frames" (tramas), generalmente de 8 bits de longitud, aunque pueden variar dependiendo de la configuración. La UART también puede incluir bits de paridad para verificar la integridad de los datos transmitidos.

Para el caso de la transmisión:

```VHDL
entity UART_tx is

    generic(
        BAUD_CLK_TICKS: integer := 10416); -- clk/baud_rate (100 000 000 / 9600 = 10416)

    port(
        clk            : in  std_logic;
        reset          : in  std_logic;
        tx_start       : in  std_logic;
        tx_data_in     : in  std_logic_vector (7 downto 0);
        tx_data_out    : out std_logic
        );
end UART_tx;

```
### 2. Señales
Las señales básicas del módulo y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Señal de reloj 
- **reset:** Señal de reset 
- **tx_start:** Señal de habilitación que controla el comienzo de la transmisión de datos
- **tx_data_in:** Datos de transmisión


#### Señales de Salida:
- **tx_data_out** Datos transmitidos de manera serial por la UART tx


### 3. Uso y criterios de diseño
El módulo de la UART tiene su propia FSM y registro interno de datos que controlan la transmisión con el protocolo serial.


## B. Módulo UART rx


### 1. Encabezado
```VHDL
entity UART_rx is

    generic(
        BAUD_X16_CLK_TICKS: integer := 651); -- (clk / baud_rate) / 16 => (100 000 000 / 9600) / 16 = 651.04

    port(
        clk            : in  std_logic;
        reset          : in  std_logic;
        rx_data_in     : in  std_logic;
        rx_data_out    : out std_logic_vector (7 downto 0)
        );
end UART_rx;

```
### 2. Señales
Las señales básicas del módulo y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Señal de reloj 
- **reset:** Señal de reset
- **rx_data_out:** Datos de entrada que representan los datos en recepción

#### Señales de Salida:
- **rx_data_out:** Datos recibidos de manera serial

### 3. Uso y criterios de diseño
Este módulo recibe datos en protocolo serial, también tiene su propia FSM interna que controla el proceso y un registro.


## C. Conexión e instanciación del módulo de transmisión UART tx y el módulo de recepción UART rx

Se crea un módulo que implementa ambos códigos de transmisión y recepción, de manera que pueda ser utilizado para transferencia y recepción de datos en el programa final, desde la FPGA al computador y viceversa.

### 1. Encabezado

```VHDL
entity UART is

    port(
        clk            : in  std_logic;
        reset          : in  std_logic; 
        tx_start       : in  std_logic; 

        data_in        : in  std_logic_vector (7 downto 0); 
        data_out       : out std_logic_vector (7 downto 0); 

        rx             : in  std_logic; 
        tx             : out std_logic 
        );
end UART;
```

### 2. Señales
Las señales básicas de ambos módulos y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Señal de reloj del sistema
- **reset:** Señal de reinicio del sistema
- **tx_start:** Indica el inicio de transmisión de datos
- **data_in:** Datos a transmitir (8 bits)
- **rx:** Señal de recepcion de datos

#### Señales de Salida:
- **data_out:** Datos a recibir (8 bits)
- **tx:** Señal de transmision de datos

### 3. Uso y criterios de diseño
Básicamente este módulo crea la interfaz UART, de esta manera se pueden utilizar los módulos de manera independiente para enviar o recibir datos en serie.

### 4. Código

```VDHL
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


use IEEE.NUMERIC_STD.ALL;



entity UART is

    port(
        clk            : in  std_logic; --Reloj del sistema
        reset          : in  std_logic; --Reinicio del sistema
        tx_start       : in  std_logic; --Indica el inicio de transmisión de datos

        data_in        : in  std_logic_vector (7 downto 0); -- Datos a transmitir (8 bits)
        data_out       : out std_logic_vector (7 downto 0); -- Datos a recibir (8 bits)

        rx             : in  std_logic; --Señal de recepcion de datos
        tx             : out std_logic --Señal de transmision de datos
        );
end UART;


architecture Behavioral of UART is
--UART DE TRANSMISIÓN
    component UART_tx
        port(
            clk            : in  std_logic;--Reloj
            reset          : in  std_logic;--Reset
            tx_start       : in  std_logic;--Inicia la transmisión de datos
            tx_data_in     : in  std_logic_vector (7 downto 0);--Datos de entrada que se transmitirán
            tx_data_out    : out std_logic --Señal de salida que indica estado de transmisión
            );
    end component;

--UART DE RECEPCION
    component UART_rx
        port(
            clk            : in  std_logic;--Reloj
            reset          : in  std_logic;--Reset
            rx_data_in     : in  std_logic;--Datos de entrada del dispositivo externo
            rx_data_out    : out std_logic_vector (7 downto 0)--Datos de salida que representan los datos recibidos
            );
    end component;

begin
--Conexión de señales
    transmitter: UART_tx
    port map(
            clk            => clk, --Señal de reloj
            reset          => reset, --Señal de reset
            tx_start       => tx_start, -- Señal de comienzo de transmisión
            tx_data_in     => data_in, -- Datos de transmisión
            tx_data_out    => tx -- Señal de transmisión
            );


    receiver: UART_rx
    port map(
            clk            => clk, -- Señal de reloj
            reset          => reset, -- Señal de reset 
            rx_data_in     => rx, -- Señal de recepción 
            rx_data_out    => data_out --Datos de recepción
            );


end Behavioral;
```
## C. Debouncer para el botón de control

Se implementa un módulo que utiliza 4 flip flops para estabilizar la señal del botón de transmisión.

### 1. Encabezado

```VHDL
--Modulo que usa 4 flips flops para evitar que los cambios rápidos en el botón se interpreten como múltiples pulsaciones
entity button_debounce is
    generic (
            COUNTER_SIZE : integer := 10_000
            );
    port ( clk        : in  std_logic;
           reset      : in  std_logic;
           button_in  : in  std_logic;
           button_out : out std_logic);
end button_debounce;
```

### 2. Señales
Las señales básicas de ambos módulos y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Señal de reloj del sistema
- **reset:** Señal de reinicio del sistema
- **button_in:** Señal de presión del botón

#### Señales de Salida:
- **button_out:** Señal de presión del botón estabilizada

### 3. Uso y criterios de diseño
Es un módulo simple que utiliza la combinación de 4 Flips Flops con un contador para procesar la señal del botón y estabilizarla evitando que se produzcan muchas señales a la vez haciendo que el código no funcione del modo correcto.

### 4. Código

```VDHL
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--Modulo que usa 4 flips flops para evitar que los cambios rápidos en el botón se interpreten como múltiples pulsaciones
entity button_debounce is
    generic (
            COUNTER_SIZE : integer := 10_000
            );
    port ( clk        : in  std_logic;
           reset      : in  std_logic;
           button_in  : in  std_logic;
           button_out : out std_logic);
end button_debounce;

architecture Behavioral of button_debounce is --4 Flip flop para limpiar la señal del botón

    signal flipflop_1       : std_logic := '0';     -- output of flip-flop 1
    signal flipflop_2       : std_logic := '0';     -- output of flip-flop 2
    signal flipflop_3       : std_logic := '0';     -- output of flip-flop 3
    signal flipflop_4       : std_logic := '0';     -- output of flip-flop 4
    signal count_start      : std_logic := '0';

begin

    input_flipflops: process(clk) --Proceso en donde se implementan dos flipflops , si son diferentes, indica que el boton se activó y el contador se establece en 1 por un ciclo del reloj
    begin
        if rising_edge(clk) then
            if (reset = '1') then
                flipflop_1 <= '0';
                flipflop_2 <= '0';
            else
                flipflop_1 <= button_in;
                flipflop_2 <= flipflop_1;
            end if;
        end if;
    end process input_flipflops;

    count_start <= flipflop_1 xor flipflop_2;

    pause_counter: process(clk)-- contador que se activa cuando el count_start cambia a 1, estabiliza el estado del botón con dos flipflops
        variable count: integer range 0 to COUNTER_SIZE := 0;
    begin
        if rising_edge(clk) then
            if (reset = '1') then
                count := 0;
                flipflop_3 <= '0';
            else
                if (count_start = '1') then
                    count := 0;
                elsif (count < COUNTER_SIZE) then
                    count := count + 1;
                else
                    flipflop_3 <= flipflop_2;
                end if;
            end if;
        end if;
    end process pause_counter;

    output_flipflop: process(clk) -- Retrasa un ciclo del reloj entre las señales del flipflop 3 y 4
    begin
        if rising_edge(clk) then
            if (reset = '1') then
                flipflop_4 <= '0';
            else
                flipflop_4 <= flipflop_3;
            end if;
        end if;
    end process output_flipflop;

    with flipflop_3 select -- buton_out se establece en 1 cuando el flip flop3 es 1 y el flip flop4 es 0, indicando que hay un cambio en el estado del boton
    button_out <= flipflop_3 xor flipflop_4 when '1',
                  '0'                       when others;

end Behavioral;

```


## D. Módulo TOP e Implementación
Al final se implementa la interfaz UART con una FSM, y los registros de datos y control, además se conectal las señales respectivas a la FPGA y se utiliza una terminal para UART en windows llamada Tera Term, para visualizar las señales de transmisión y recepción. 

### 1. Encabezado
```SystemVerilog
module UART_controller(
  input wire clk,             
  input wire reset,           
  input wire [7:0] data_in,   
  output wire [7:0] data_out, 
  input wire rx,             
  input wire tx,             
  output wire tx_line,       
  input wire rx_line         
);

```
### 2. Señales
Las señales básicas del módulo y su funcionamiento son:

#### Señales de Entrada:
- **clk:** Reloj del sistema
- **reset:** Reset del controlador
- **[7:0] data_in:** Datos para ser transmitidos
- **wire rx:** Señal de recepción de datos
- **wire tx:** Señal de transmisión de datos
- **wire rx_line:** Línea de recepción

#### Señales de Salida:
- **[7:0] data_out:** Datos de salida que se reciben a través de UART
- **wire tx_line:** Línea de transmisión

### 3. Uso y criterios de diseño
El módulo al implementarse utiliza la interfaz UART para controlar el paso de información de manera serial y bidireccional.

### 4. Código 

```SystemVerilog
module UART_controller(
  input wire clk,             // System clock
  input wire reset,           // Controller reset
  input wire [7:0] data_in,   // Input data to be transmitted via UART
  output wire [7:0] data_out, // Received data from UART
  input wire rx,              // Data reception signal
  input wire tx,              // Data transmission signal
  output wire tx_line,        // UART transmission line
  input wire rx_line          // UART reception line
);

  // Define states for the FSM
  typedef enum {IDLE, RECEIVE, TRANSMIT} state_type;
  reg [1:0] current_state, next_state;

  // Internal signals for controlling UART transmission and reception
  reg tx_start, rx_start;

  // Internal signal to indicate data ready
  wire rx_data_rdy;

  // Instantiating control and data registers
  wire [1:0] control_reg_out;
  wire [7:0] tx_data, rx_data;
  reg write_enable;

  // Instantiate the control register
  control_register control_reg(
    .clk(clk),
    .rx_bit(rx),
    .tx_bit(tx),
    .stored_bits(control_reg_out)
  );

  // Instantiate the data registers
  data_register #(.WIDTH(8)) tx_data_reg(
    .clk(clk),
    .write_enable(tx_start),
    .data_in(data_in),
    .data_out(tx_data)
  );

  data_register #(.WIDTH(8)) rx_data_reg(
    .clk(clk),
    .write_enable(rx_data_rdy),
    .data_in(rx_data),
    .data_out(data_out)
  );

  // UART module instantiation
  UART UART_transceiver(
    .clk(clk),             // System clock
    .reset(reset),         // Reset signal for UART
    .tx_start(tx_start),   // UART transmission start signal
    .data_in(tx_data),     // Input data to UART
    .data_out(rx_data),    // Received data from UART
    .rx_data_rdy(rx_data_rdy), // Data ready signal from UART receiver
    .rx(rx_line),          // UART reception signal
    .tx(tx_line)           // UART transmission signal
  );

  // Sequential process for state transition
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      current_state <= IDLE;
    end else begin
      current_state <= next_state;
    end
  end

  // Combinational process for state logic
  always @* begin
    next_state = current_state;
    tx_start = 1'b0;
    rx_start = 1'b0;

    case (current_state)
      IDLE: begin
        if (control_reg_out[0]) begin
          next_state = RECEIVE;
          rx_start = 1'b1;
        end else if (control_reg_out[1]) begin
          next_state = TRANSMIT;
          tx_start = 1'b1;
        end
      end
      RECEIVE: begin
        if (!control_reg_out[0]) begin
          next_state = IDLE;
        end
      end
      TRANSMIT: begin
        if (!control_reg_out[1]) begin
          next_state = IDLE;
        end
      end
    endcase
  end

endmodule
```

### 5. Implementación y prueba en FPGA
Se procede a sintetizar, implementar y generar el Bitstream para probar en la FPGA. El constraint utilizado es el siguiente:

```SystemVerilog
set_property PACKAGE_PIN V17 [get_ports {data_in[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[0]}]
set_property PACKAGE_PIN V16 [get_ports {data_in[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[1]}]
set_property PACKAGE_PIN W16 [get_ports {data_in[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[2]}]
set_property PACKAGE_PIN W17 [get_ports {data_in[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[3]}]
set_property PACKAGE_PIN W15 [get_ports {data_in[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[4]}]
set_property PACKAGE_PIN V15 [get_ports {data_in[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[5]}]
set_property PACKAGE_PIN W14 [get_ports {data_in[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[6]}]
set_property PACKAGE_PIN W13 [get_ports {data_in[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_in[7]}]
set_property PACKAGE_PIN U16 [get_ports {data_out[0]}]
set_property PACKAGE_PIN E19 [get_ports {data_out[1]}]
set_property PACKAGE_PIN U19 [get_ports {data_out[2]}]
set_property PACKAGE_PIN V19 [get_ports {data_out[3]}]
set_property PACKAGE_PIN W18 [get_ports {data_out[4]}]
set_property PACKAGE_PIN U15 [get_ports {data_out[5]}]
set_property PACKAGE_PIN U14 [get_ports {data_out[6]}]
set_property PACKAGE_PIN V14 [get_ports {data_out[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {data_out[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports reset]
set_property PACKAGE_PIN A18 [get_ports tx]
set_property IOSTANDARD LVCMOS33 [get_ports tx]
set_property PACKAGE_PIN B18 [get_ports rx]
set_property IOSTANDARD LVCMOS33 [get_ports rx]
set_property PACKAGE_PIN T18 [get_ports reset]
#set_property PACKAGE_PIN U18 [get_ports tx_start]
#set_property IOSTANDARD LVCMOS33 [get_ports tx_start]
set_property PACKAGE_PIN W5 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property PACKAGE_PIN U18 [get_ports tx_enable]
set_property IOSTANDARD LVCMOS33 [get_ports tx_enable]
create_clock -period 10.000 -name clk -waveform {0.000 5.000} [get_ports clk]
```

Seguidamente se prueba la transmisión:

Se conecta la FPGA y se programa con el bitstream creado. Se procede a enviar la señal con los switches correspondiendte al ASCII A:

![Envío de datos desde la FPGA](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%202/Imagenes/TX.jpeg)

![Dato enviado desde la FPGA a la terminal Tera Term](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%202/Imagenes/Serial%20TX.png)

Por último se prueba la recepción en la FPGA:

Se procede a enviar la señal con el teclado, la tecla es la B y debería verse el equivalente con los Leds en la FPGA:

![Envío de datos desde el Serial](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%202/Imagenes/Serial%20RX.png)

![Dato recibido en la FPGA](https://github.com/KRSahalie/Laboratorio3-TDD/blob/main/Ejercicio%202/Imagenes/RX.jpeg)

De esta manera se puede verificar que el envío y la recepción de datos funcionan correctamente. 














