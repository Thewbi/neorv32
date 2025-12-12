

# NEORV32 Documents

Notes and materials on the NEORV32 VHDL CPU
<!--
[ToC]
[[_TOC_]]
-->

<!--

Regenerating the TOC:

Install VSCode extension Markdown TOC by Joffrey Kern.

1. Open any markdown file
2. Open the command palette (Ctrl+Shift+P)
3. Type "Generate"
4. Choose "Generate TOC for markdown"

-->

<!-- vscode-markdown-toc -->
1. [Select the Mnemonic](#SelecttheMnemonic)
2. [Select the Encoding](#SelecttheEncoding)
3. [Extending the Execution Engine Microcode](#ExtendingtheExecutionEngineMicrocode)
4. [Strategies for Writing Applications](#StrategiesforWritingApplications)
5. [The Standard Approach using a Toolchain and Makefiles](#TheStandardApproachusingaToolchainandMakefiles)
6. [Setup the toolchain and the build environment](#Setupthetoolchainandthebuildenvironment)
7. [Compiling](#Compiling)
8. [Analysing the Assembly Listing](#AnalysingtheAssemblyListing)
9. [Initializing the C-runtime](#InitializingtheC-runtime)
10. [Symmetric Multi Processing, Hart 0 Check](#SymmetricMultiProcessingHart0Check)
11. [Summary](#Summary)
12. [Write a simple main() function](#Writeasimplemainfunction)
13. [Editing the Main Function](#EditingtheMainFunction)
14. [Inline Assembly](#InlineAssembly)
	14.1. [String Concatenation](#StringConcatenation)
	14.2. [Replacing Placeholders](#ReplacingPlaceholders)
	14.3. [In- and out-parameters](#In-andout-parameters)
	14.4. [Usage](#Usage)
15. [Extending the NEORV32 via the Custom Functions Unit (CFU)](#ExtendingtheNEORV32viatheCustomFunctionsUnitCFU)
	 15.1. [Merge/Pull Request that added the CFU](#MergePullRequestthataddedtheCFU)
	 15.2. [Questions](#Questions)
16. [Extending the NEORV32 via the Custom Functions Subsystem (CFS)](#ExtendingtheNEORV32viatheCustomFunctionsSubsystemCFS)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->

# Credit

All credit goes to Stephan Nolting. All the ideas stem from his designs.
I salute his ingenuity and creativity.

# Links

https://stnolting.github.io/neorv32/

# Tips

In VSCode, adding

```
#warning-ignore-all
```

to the top of a VHDL (or any) file will turn of error-squigglies
which makes it easier to read the file in case the linter is not
able to resolve all symbols correctly.

# Simulation

https://stnolting.github.io/neorv32/ug/#_simulating_the_processor
https://stnolting.github.io/neorv32/ug/#_ghdl_simulation

Install Msys2. Inside the Msys2 console, install GHDL.

```
cd /c/Users/lapto/dev/VHDL/neorv32/sim
sh ghdl.sh --stop-time=20ms --vcd=neorv32.vcd

sh ghdl.sh --stop-time=20us --vcd=neorv32.vcd

sh ghdl.sh --stop-time=20ns --vcd=neorv32.vcd
```

The shell script will compile (analyse, evaluate with GHDL) the
processor and start a simulation. The signal traces go into the
waveform file neorv32.vcd which you can open and analyze in
GTKWave, Surfer or other waveform viewers.

The CPU is simulated for the real-time period specified via
the --stop-time parameter. The simulation takes much longer than
the specified real time period. It is probably smart to first
try with 20us instead of 20ms. 20ns is not enough to get the
CPU to process relevant data.

# Execution Engine

The NEORV32 processor is separated into a frontend and a backend.
The frontend fetches instructions while the backend executes them.
Both units can work somewhat independently from each other and are
loosly coupled by a queue. This allows the frontend to fetch several
instructions without waiting for the backend.

The backend is a mixture of a pipelined architecture with separate
phases (FETCH, DECODE, EXECUTE, MEM_ACCESS, WRITE_BACK) and a
micro-code architecture.

The instruction from the frontend enters the execution engine during
the EX_DISPATCH microcode step. The exe_engine_nxt signal is used
to store the instruction. Also the Program Counter (PC) is updated.

```VHDL
exe_engine_nxt.ir    <= frontend_i.instr;                         -- instruction word
exe_engine_nxt.pc    <= exe_engine.pc2(XLEN-1 downto 1) & '0';    -- PC <= next PC
```

Sadly when simulating the design, the generated .vcd file will not
contain the internal signals which carry the instruction! This makes
it hard to debug what instructions the CPU executes during simulation.

# Finding executed instructions

Using only a GHDL simulation and a waveform viewer, it is hard to
figure out, which instructions are executed. The reason is that
the frontend will perform instruction fetches which are to a degree
independant from the instructions that the backend will execute.
The frontend has two instruction prefetch buffers.

The backend will execute instructions in the order in which the
assembly code orders them, there is no reordering. The instructions
enter the backend via the a structure called 'frontend'. I have not
yet understood how a simulator such as GHDL decides, which signals
it will write to the .vcd file and which are hidden. The frontend
structure is not written into the .vcd file.

To trace the instructions, as a debug measure, it is possible to add
artificial signals into the design strictly so that GHDL writes
them to the .vcd file.

Making these changes:

*rtl\core\neorv32_cpu.vhd*

```VHDL
  -- Control Unit (Back-End / Instruction Execution) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: entity neorv32.neorv32_cpu_control
    generic map (
        ...
    )
    port map (

        ...

        -- [debug] - make instruction visible
        debug_valid_i   => frontend.valid,       -- bus signals are valid
        debug_instr_i   => frontend.instr,       -- instruction

        ...

    );
```

*rtl\core\neorv32_cpu_control.vhd*

```VHDL
entity neorv32_cpu_control is
    generic (
        ...
    );
    port (

        ...

        -- [debug] - make instruction visible
        debug_valid_i   : in std_ulogic;                          -- bus signals are valid
        debug_instr_i   : in std_ulogic_vector(31 downto 0);      -- instruction

        ...

    );
end neorv32_cpu_control;
```

GHDL will output the following trace:

![image info](res/images/DebuggingInstructions.png)

REMARK: The surfer trace viewer (https://surfer-project.org/) is used
in the image above.

This trace shows that the frontend prefetches instructions in non-intuitive
ways. For example the compressed instruction 0x00002b27 is fetched, then
0x80028293 is fetched and then 0x00002b27 is fetched again!

The execution engine is displayed in the lower traces thanks to the
debug-signals inserted above.

The *debug_instr* trace contains the instruction that the execution engine
sees and the *debug_valid* trace dictates if the execution engine is triggered
to consume the instruction. If the valid signal is low, the execution engine
ignores the instruction.

The execution engine traces show which instructions are executed by the CPU.
For example the second fetch of the compressed instruction 0x00002b27 is ignored
by the execution engine as the *debug_valid* signal is low.

# Adding custom instructions

Lets add a custom instruction to the NEORV32 RISC-V CPU.

##  1. <a name='SelecttheMnemonic'></a>Select the Mnemonic

The RV32I instructions and their encoding is defined on page 586 of the unprivileged
RISC-V specification.

![image info](res/images/RV32_Instructions.png)

Lets mimic the add instruction and instead of adding two registers, add the
first register value to the second register value and on top of that add the
constant value 1 to the result.

Lets mimic the add instruction and instead of adding two registers, add the
first register to itself.

Or count the number of bits that are set to 1 in an immediate and return that
count into the target register.

```
add1 rd, rs1, rs2
```

add1 will perform the operation

```
rd = rs1 + rs2 + 1
```

##  2. <a name='SelecttheEncoding'></a>Select the Encoding

The encoding of add is R-Type because it is a register (R) instruction.

| component | value | remark |
| --------- | ----- | -------------- |
| opcode    | 0b0110011 | lowest two bits are 11 because this is not a compressed instruction |
| funct7    | 0b0000000 | |
| funct3    | 0b000 | |

The opcode 0b0110011 is defined to be an ALU-operation within the NEORV32 source code.

*rtl\core\neorv32_package.vhd*

```VHDL
constant opcode_alu_c    : std_ulogic_vector(6 downto 0) := "0110011"; -- ALU operation
```

In general, func3 is used to control the operation that the ALU will perform. The ALU can
add and subtract (whereas subtraction is an addition with an sign-inverted/negated operand),
shift left or right, XOR, OR and AND.

Amongst the ALU operations, a func3 value of 0b000 is defined to be a sub or a add ALU-operation.

```VHDL
constant funct3_sadd_c   : std_ulogic_vector(2 downto 0) := "000"; -- sub/add
```

Which leaves us with funct7 for adding codes for custom instructions.

funct7 of 0b0000000 is used for add. funct7 of 0b0100000 is used for sub.

Note that the second highest bit is what activates subtraction inside the NEORV32 execution engine:

*rtl\core\neorv32_cpu_control.vhd*
```VHDL
-- addition/subtraction control --
if (funct3_v(2 downto 1) = funct3_slt_c(2 downto 1)) or -- SLT(I), SLTU(I)
    ((funct3_v = funct3_sadd_c) and (opcode(5) = '1') and (exe_engine.ir(instr_funct7_msb_c-1) = '1')) then -- SUB
    ctrl_nxt.alu_sub <= '1';
end if;
```

This means, any funct7 with the scond highest bit set is not of use for the add1 instruction.

Let's use funct7 of 0b1000000 for add1.

##  3. <a name='ExtendingtheExecutionEngineMicrocode'></a>Extending the Execution Engine Microcode

Next, since the Execution Engine checks instructions before executing them, we need to add the add1 instruction to the check.

*rtl\core\neorv32_cpu_control.vhd*

```VHDL
-- is base rv32i/e ALU[I] instruction (excluding shifts)? --
if ((opcode(5) = '0') and (funct3_v /= funct3_sll_c) and (funct3_v /= funct3_sr_c)) or -- base ALUI instruction (excluding SLLI, SRLI, SRAI)
    ((opcode(5) = '1') and (((funct3_v = funct3_sadd_c) and (funct7_v = "0000000")) or -- add
                            ((funct3_v = funct3_sadd_c) and (funct7_v = "0100000")) or -- sub
                            ((funct3_v = funct3_sadd_c) and (funct7_v = "1000000")) or -- add1
                            ((funct3_v = funct3_slt_c)  and (funct7_v = "0000000")) or
                            ((funct3_v = funct3_sltu_c) and (funct7_v = "0000000")) or
                            ((funct3_v = funct3_xor_c)  and (funct7_v = "0000000")) or
                            ((funct3_v = funct3_or_c)   and (funct7_v = "0000000")) or
                            ((funct3_v = funct3_and_c)  and (funct7_v = "0000000")))) then -- base ALU instruction (excluding SLL, SRL, SRA)
    ctrl_nxt.rf_wb_en    <= '1'; -- valid RF write-back (won't happen if exception)
    exe_engine_nxt.state <= EX_DISPATCH;
else -- [NOTE] illegal ALU[I] instructions are handled as multi-cycle operations that will time-out as no ALU co-processor responds
    ctrl_nxt.alu_cp_alu  <= '1'; -- trigger ALU[I] opcode-space co-processor
    exe_engine_nxt.state <= EX_ALU_WAIT;
end if;
```

In the listing above, a line for the add1 instruction has been added to the check.

TODO: changes for add1
TODO: changes for debugging alu operations and alu_inc

# Writing an application that uses the new instruction

Next, we need to create machine code that we can copy into rtl\core\neorv32_application_image.vhd
because this is the ROM that is synthesized into the NEORV32 CPU for simulating and the NEORV32
processor will execute the code stored in this ROM when the simulation starts.

##  4. <a name='StrategiesforWritingApplications'></a>Strategies for Writing Applications

In order to create machine code, there is a quick and dirty way and a standard approach.

The quick and dirty way is to assemble your machine code by hand according to the RISC-V encoding
defined in the RISC-V non priviledged specification. Then paste you machine code into
rtl\core\neorv32_application_image.vhd

##  5. <a name='TheStandardApproachusingaToolchainandMakefiles'></a>The Standard Approach using a Toolchain and Makefiles

The standard approach is to follow the structure outlined in the examples located inside the
sw\example folder.

The examples use a Makefile in each of the example folders which sets flags and then includes
the common Makefile which is used for all examples. The common makefile is sw\common\common.mk

Inside the common.mk makefile, all the targets are defined. A target is a certain function that
you can execute using the build system. One target is the 'all' target which performs all build
steps. You trigger a target by adding it as a parameter to the make command.

```
make all
```

The makefiles uses the RISC-V GNU GCC toolchain to build machine code. A toolchain
is a term for a set of tools that you can use for compiling higher languages into assemly (gcc)
and for turning the assembly into machine code (as). Also tools that decompile output files back to
assembly are available (objdump). The toolchain contains a plethora of tools that I have never even
used yet! It is worthwhile to familiarize yourself with the applications available inside the
toolchain's bin folder to become a better developer if you are interested in open source development.

The RISC-V GNU GCC toolchain that we will be using is going to be a toolchain for cross compiling.
Cross compiling is when the toolchain outputs machine code that the system on which the tools are
executed, cannot run. Instead we use one computer system to write code for another computer system.

In our case, you will most likely (we assume) be working on a x86 or a ARM
system. Your target is RISC-V for the NEORV32. With that, you are going to need a cross compiler
toolchain. FYI, native toolchains produce machine code for the architecture that also runs the
toolchain itself.

##  6. <a name='Setupthetoolchainandthebuildenvironment'></a>Setup the toolchain and the build environment

The NEORV32 repository does not come with the cross compiler toolchain prepackaged. Therefore download a
precompiled gcc toolchain from https://xpack-dev-tools.github.io/riscv-none-elf-gcc-xpack/
more precicesly from the release page of the xpack project: https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases

Next, install Msys2 if you have not already and open a Msys2 64 bit console.
Inside the console, export the *NEORV32_HOME* environment variable.

```
NEORV32_HOME=/C/Users/lapto/dev/VHDL/neorv32
```

Adjust the paths to your local situation. Copy and pasting my paths from this tutorial without
adjustments would be a mistake!

Also add the toolchain's bin folder to the PATH environment variable

```
PATH=/c/Users/lapto/Downloads/xpack-riscv-none-elf-gcc-14.2.0-3-win32-x64/xpack-riscv-none-elf-gcc-14.2.0-3/bin:$PATH
```

Enter one of the software example folders.

```
cd /c/Users/lapto/dev/VHDL/neorv32/sw/example/demo_cfu
```

Next let the makefile check if the cross compiling toolchain is setup correctly.
To do this, execute the check target.

```
make check
```

The output should be 'Toolchain check OK'. The check target will also compile the
image_gen tool. The image_gen tool produces new neorv32_application_image.vhd files.

The next step will be to compile the example.

##  7. <a name='Compiling'></a>Compiling

```
make all
```

The 'all' target creates the machine code, stores the machine code into several file
formats. It also generates a new neorv32_application_image.vhd and copies it
into the VHDL source code folder so you can immedately start a simulation with the
new code.

```
$ make all
Memory utilization:
   text    data     bss     dec     hex filename
   6324       0    1292    7616    1dc0 main.elf
Generating neorv32_exe.bin
Executable size in bytes:
6336
Generating neorv32_raw_exe.hex
Generating neorv32_raw_exe.bin
Generating neorv32_raw_exe.coe
Generating neorv32_raw_exe.mem
Generating neorv32_raw_exe.mif
Generating neorv32_application_image.vhd
Installing application image to ../../../rtl/core/neorv32_application_image.vhd
```

One important output file is the .elf file. .elf stands for executable and linkable format.
.elf files may contain executable applications and also libraries that can be
linked to become part of applications. .elf is used by Linux to transfer machine code
from the harddrive into RAM for execution in a new process. The respective file format
on windows would be the PE file format. The GCC toolchain cannot create PE files.

NEORV32 does not use the .elf file but the cross compile toolchain contains the
objdump tool which extracts RISC-V assembly code from the .elf file. This is important
for use here since the original source code of the example is written in C but we
want to analyze the application in assembly code in the following.

Convert the .elf file into an assembly listing:

```
riscv-none-elf-objdump --disassemble main.elf > listing.asm
```

##  8. <a name='AnalysingtheAssemblyListing'></a>Analysing the Assembly Listing

The assembly listing is ridiculously large. The most important function is the
main function because it contains the actual source code that the user has written
using the C programming language. Basically this is the payload that the user is
interested in.

```
00000334 <main>:
     334:	fc010113          	addi	sp,sp,-64
     338:	02112e23          	sw	ra,60(sp)
     33c:	02812c23          	sw	s0,56(sp)
     340:	02912a23          	sw	s1,52(sp)
     344:	03212823          	sw	s2,48(sp)
     348:	03312623          	sw	s3,44(sp)
     34c:	03412423          	sw	s4,40(sp)
     350:	03512223          	sw	s5,36(sp)
     354:	03612023          	sw	s6,32(sp)
     358:	091000ef          	jal	be8 <neorv32_rte_setup>
     35c:	fff50537          	lui	a0,0xfff50
     360:	0f1000ef          	jal	c50 <neorv32_uart_available>
     364:	1e050463          	beqz	a0,54c <main+0x218>
     368:	000055b7          	lui	a1,0x5
     36c:	00000613          	li	a2,0
     370:	b0058593          	addi	a1,a1,-1280 # 4b00 <__neorv32_rom_size+0xb00>
     374:	fff50537          	lui	a0,0xfff50
     378:	115000ef          	jal	c8c <neorv32_uart_setup>
     37c:	000015b7          	lui	a1,0x1
     380:	34058593          	addi	a1,a1,832 # 1340 <__fini_array_end>
     384:	fff50537          	lui	a0,0xfff50
     388:	419000ef          	jal	fa0 <neorv32_uart_printf>
     38c:	000015b7          	lui	a1,0x1

     ...
```

The main function is not the first set of instructions that are executed!
Instead, the application starts with initializing the C-runtime first.

The instructions mainly come from the code that the toolchain generates for the
C-programming language. C has a C-runtime which is a set all functions that solve
common problems such as strlen() but also acts as a layer that ports the C language
to a target by implementing memory handling through malloc() and free() for example.

Some of the C-runtime functions are provided by the compiler which means it is
provided by the gcc cross compiler toolchain. To initialize the C-runtime, a assembly
file from the NEORV32 repostory is used. This means this file is not part of the
cross compiler toolchain! You can see that the entire software system is rather complex.

In order for this layer of C-runtime to work, it needs to be initialized. This means
that some sections of RAM are filled with certain values so that all variable used
by the C-runtimes have valid values. Before main() is executed, the application starts
with the initialization of the C-runtime, then jumps to the main() function.

crt stands for C-runtime.

##  9. <a name='InitializingtheC-runtime'></a>Initializing the C-runtime

Let's look at the disassembly for the C runtime initialization.

```

main.elf:     file format elf32-littleriscv


Disassembly of section .text:

00000000 <__crt0_entry>:
       0:	f14020f3          	csrr	ra,mhartid
       4:	80002217          	auipc	tp,0x80002
       8:	ffb20213          	addi	tp,tp,-5 # 80001fff <__crt0_ram_last>
       c:	ff027113          	andi	sp,tp,-16
      10:	80000197          	auipc	gp,0x80000
      14:	7f018193          	addi	gp,gp,2032 # 80000800 <__global_pointer>
      18:	000022b7          	lui	t0,0x2
      1c:	80028293          	addi	t0,t0,-2048 # 1800 <_ctype_+0x50>
      20:	30029073          	csrw	mstatus,t0
      24:	00000317          	auipc	t1,0x0
      28:	18030313          	addi	t1,t1,384 # 1a4 <__crt0_trap>
      2c:	30531073          	csrw	mtvec,t1
      30:	30401073          	csrw	mie,zero
      34:	00002397          	auipc	t2,0x2
      38:	88038393          	addi	t2,t2,-1920 # 18b4 <__crt0_copy_data_src_begin>
      3c:	80000417          	auipc	s0,0x80000
      40:	fc440413          	addi	s0,s0,-60 # 80000000 <time_dec_sw>
      44:	80000497          	auipc	s1,0x80000
      48:	fbc48493          	addi	s1,s1,-68 # 80000000 <time_dec_sw>
      4c:	80000517          	auipc	a0,0x80000
      50:	fb450513          	addi	a0,a0,-76 # 80000000 <time_dec_sw>
      54:	80000597          	auipc	a1,0x80000
      58:	4b858593          	addi	a1,a1,1208 # 8000050c <__crt0_bss_end>
      5c:	00000613          	li	a2,0
      60:	00000693          	li	a3,0
      64:	00000713          	li	a4,0
      68:	00000793          	li	a5,0
      6c:	00000813          	li	a6,0
      70:	00000893          	li	a7,0
      74:	00000913          	li	s2,0
      78:	00000993          	li	s3,0
      7c:	00000a13          	li	s4,0
      80:	00000a93          	li	s5,0
      84:	00000b13          	li	s6,0
      88:	00000b93          	li	s7,0
      8c:	00000c13          	li	s8,0
      90:	00000c93          	li	s9,0
      94:	00000d13          	li	s10,0
      98:	00000d93          	li	s11,0
      9c:	00000e13          	li	t3,0
      a0:	00000e93          	li	t4,0
      a4:	00000f13          	li	t5,0
      a8:	00000f93          	li	t6,0

000000ac <__crt0_smp_check>:
      ac:	02008a63          	beqz	ra,e0 <__crt0_smp_primary>
      b0:	00000797          	auipc	a5,0x0
      b4:	01878793          	addi	a5,a5,24 # c8 <__crt0_smp_wakeup>
      b8:	30579073          	csrw	mtvec,a5
      bc:	30446073          	csrsi	mie,8
      c0:	30046073          	csrsi	mstatus,8
      c4:	0d80006f          	j	19c <__crt0_sleep>

000000c8 <__crt0_smp_wakeup>:
      c8:	fff44737          	lui	a4,0xfff44
      cc:	00872103          	lw	sp,8(a4) # fff44008 <__crt0_ram_last+0x7ff42009>
      d0:	00c72603          	lw	a2,12(a4)
      d4:	fff40737          	lui	a4,0xfff40
      d8:	00072223          	sw	zero,4(a4) # fff40004 <__crt0_ram_last+0x7ff3e005>
      dc:	05c0006f          	j	138 <__crt0_main_entry>

000000e0 <__crt0_smp_primary>:
      e0:	00838e63          	beq	t2,s0,fc <__crt0_bss_clear>

000000e4 <__crt0_data_copy>:
      e4:	00945c63          	bge	s0,s1,fc <__crt0_bss_clear>
      e8:	0003a783          	lw	a5,0(t2)
      ec:	00f42023          	sw	a5,0(s0)
      f0:	00438393          	addi	t2,t2,4
      f4:	00440413          	addi	s0,s0,4
      f8:	fedff06f          	j	e4 <__crt0_data_copy>

000000fc <__crt0_bss_clear>:
      fc:	00b55863          	bge	a0,a1,10c <__crt0_bss_clear_end>
     100:	00052023          	sw	zero,0(a0)
     104:	00450513          	addi	a0,a0,4
     108:	ff5ff06f          	j	fc <__crt0_bss_clear>

0000010c <__crt0_bss_clear_end>:
     10c:	00001417          	auipc	s0,0x1
     110:	23440413          	addi	s0,s0,564 # 1340 <__fini_array_end>
     114:	00001497          	auipc	s1,0x1
     118:	22c48493          	addi	s1,s1,556 # 1340 <__fini_array_end>

0000011c <__crt0_constructors>:
     11c:	00945a63          	bge	s0,s1,130 <__crt0_constructors_end>
     120:	00042083          	lw	ra,0(s0)
     124:	000080e7          	jalr	ra
     128:	00440413          	addi	s0,s0,4
     12c:	ff1ff06f          	j	11c <__crt0_constructors>

00000130 <__crt0_constructors_end>:
     130:	00000617          	auipc	a2,0x0
     134:	20460613          	addi	a2,a2,516 # 334 <main>

00000138 <__crt0_main_entry>:
     138:	80000197          	auipc	gp,0x80000
     13c:	6c818193          	addi	gp,gp,1736 # 80000800 <__global_pointer>
     140:	0ff0000f          	fence
     144:	0000100f          	fence.i
     148:	30029073          	csrw	mstatus,t0
     14c:	00000513          	li	a0,0
     150:	00000593          	li	a1,0
     154:	000600e7          	jalr	a2

00000158 <__crt0_main_exit>:
     158:	30401073          	csrw	mie,zero
     15c:	34051073          	csrw	mscratch,a0
     160:	00000517          	auipc	a0,0x0
     164:	04450513          	addi	a0,a0,68 # 1a4 <__crt0_trap>
     168:	30551073          	csrw	mtvec,a0
     16c:	f1402473          	csrr	s0,mhartid
     170:	02041463          	bnez	s0,198 <__crt0_destructors_end>
     174:	00001417          	auipc	s0,0x1
     178:	1cc40413          	addi	s0,s0,460 # 1340 <__fini_array_end>
     17c:	00001497          	auipc	s1,0x1
     180:	1c448493          	addi	s1,s1,452 # 1340 <__fini_array_end>
```

Disclaimer: I do not pretend to understand exactly what is happening here in all detail.

Let's check the most important parts.

Firstly, when the C-runtime is done initializing, it calls the main() function.

```
154:	000600e7          	jalr	a2
```

The register a2 has been loaded with the address of the main() function a few
lines earlier.

```
00000130 <__crt0_constructors_end>:
     130:	00000617          	auipc	a2,0x0
     134:	20460613          	addi	a2,a2,516 # 334 <main>
```

At the very beginning, at address 0x00000000, we find the code contained in
the sw\common\crt0.S file. crt stands for C-runtime. This means this assembly
file contains all the C-runtime initialization. Remark: This file is provided
by the NEORV32 repository and not by the cross compiler toolchain for some reason.

This means that the crt0.s file is linked into the executable at the beginning
of the address space.

It is easier to read the original assembly code than to read the disassembled
assembly because the original source file contains human readable symbols and
comments. So let's look at the crt0.s instead of the disassembly listing.

Here is a excerpt from sw\common\crt0.S

```
// ************************************************************************************************
// Register setup.
// ************************************************************************************************
.option push
.option norelax
  csrr  x1, mhartid                     // get ID of this core

  la    x4, __crt0_ram_last             // last address of RAM, stack pointer (sp) starts here
  andi  x2, x4, 0xfffffff0              // align stack to 16-bytes according to the RISC-V ABI (#1021)
  la    x3, __global_pointer            // global pointer "gp"

  li    x5, 0x00001800                  // mstatus.mpp = machine-mode
  csrw  mstatus, x5
  la    x6, __crt0_trap                 // configure early-boot trap handler
  csrw  mtvec, x6
  csrw  mie, zero                       // disable all interrupt sources

  la    x7,  __crt0_copy_data_src_begin // .data: start of copy-source (in .rodata)
  la    x8,  __crt0_copy_data_dst_begin // .data: start of actual data region
  la    x9,  __crt0_copy_data_dst_end   // .data: end of actual data region
  la    x10, __crt0_bss_start           // .bss: start address
  la    x11, __crt0_bss_end             // .bss: end address (not part of bss)
.option pop

  // initialize remaining registers
  addi  x12, zero, 0
  addi  x13, zero, 0
  addi  x14, zero, 0
  addi  x15, zero, 0
#ifndef __riscv_32e
  addi  x16, zero, 0
  addi  x17, zero, 0
  addi  x18, zero, 0
  addi  x19, zero, 0
  addi  x20, zero, 0
  addi  x21, zero, 0
  addi  x22, zero, 0
  addi  x23, zero, 0
  addi  x24, zero, 0
  addi  x25, zero, 0
  addi  x26, zero, 0
  addi  x27, zero, 0
  addi  x28, zero, 0
  addi  x29, zero, 0
  addi  x30, zero, 0
  addi  x31, zero, 0
#endif
```

Firstly, the id of the hart that runs the assembly is stored into the register x1.
The term hart is RISC-V lingo for a CPU core in a multi-core system.

Then the stack pointer is set up.

The global pointer is set up. The global pointer
is used to point to some global data in RAM. Sometimes, text strings or other data
is located into RAM by the compiler for later access when the code is executed. The
global pointer is set such global data in RAM.

The hart is set to machine mode which is the privileged mode as opposed to the
user mode which is not privileged.

In machine mode, interrupts are turned off.

Then the registers are initalized. Some registers get addresses loaded with the la instruction.
The rest of the registers are set to 0. Interestingly, the compiler does even respect
the embedded variant of RISC-V since it will stop initializing registers after x15
if the symbol __riscv_32e is defined, because the embedded variants of RISC-V only
contain the first 16 registers as opposed to all 32 registers!

##  10. <a name='SymmetricMultiProcessingHart0Check'></a>Symmetric Multi Processing, Hart 0 Check

Firstly, RISC-V, like any other multi processor system, will run the software on all harts.
To give the user full control over what the harts do, the C-runtime contains code that
puts harts to sleep. Next follows the explanation how this is accomplished.

The system software will check if it is run on the first hart which has the hart id 0.
Depending on the current hart id, the same application will show different behaviour!

As is the habit on multi core systems, system-software will start executing on all cores but
it will keep running on core 0 only!

For the rest of the cores, the system software is executed for a very short time until it
hits the core 0 check. It will fail the core 0 check on other harts and it will then pend
on a software interrupt.

This means that all cores other than core 0 will pend. Pending means that they will go
to sleep until the software interrupt is triggered to wake up those cores.

The system software running on core 0 may decide to wake up the other cores or it may decide
to let the cores sleep which effectively turns the multi-core system into a single-core system.

> Prithee lull the Old one back to it's ancient slumber

*The maiden, Demon's Souls by FromSoft / Bandai Namco*

SMP stands for symetric multi processing.

Here is the code that makes the harts sleep:

```
// ************************************************************************************************
// SMP setup - wait for configuration if we are not core 0.
// ************************************************************************************************
__crt0_smp_check:
  beqz  x1, __crt0_smp_primary              // proceed with normal boot-up if we are core 0

  // setup machine software interrupt
  la    x15,     __crt0_smp_wakeup
  csrw  mtvec,   x15                        // install interrupt handler
  csrsi mie,     1 << 3                     // only enable software interrupt source
  csrsi mstatus, 1 << 3                     // enable machine-level interrupts
  j     __crt0_sleep                        // wait for interrupt in sleep mode

  // machine software interrupt handler
__crt0_smp_wakeup:
  li    x14, 0xfff44000                     // CLINT.MTIMECMP base address
  lw    x2,  8(x14)                         // MTIMECMP[1].lo = stack top (sp)
  lw    x12, 12(x14)                        // MTIMECMP[1].hi = entry point

  // acknowledge booting
  li    x14,  0xfff40000                    // CLINT.MSWI base address
  sw    zero, 4(x14)                        // clear MSWI[1]

  j     __crt0_main_entry                   // start at entry point

__crt0_smp_primary:
```

We can see that the beqz command (Branch equal zero) will skip the entire sleep code on hart 0
since each core will store it's id into x1 as we have analyzed earlier. This is why the register
x1 is used for the core 0 check.

The sleep code jumps to the *__crt0_sleep* label for sleeping. When the interrupt wakes up the
core, the code returns on the *__crt0_smp_wakeup* label. Eventually, the sleep code will jump
to the *__crt0_main_entry* symbol. From the *__crt0_main_entry*, the main() function is called.
This means that system software on the other harts will start executing the same main function
as hart 0 does eventually after wakeup.

```
// ************************************************************************************************
// Setup arguments and call main function.
// ************************************************************************************************
  la    x12, main             // primary core's (core0) entry point (#1169)
__crt0_main_entry:
  la    x3, __global_pointer  // re-initialize global pointer "gp" (to prevent a race condition during SMP boot)
  fence                       // synchronize loads/stores
  fence.i                     // synchronize instruction fetch

  csrw  mstatus, x5           // re-initialize
  addi  x10, zero, 0          // x10 = a0 = argc = 0
  addi  x11, zero, 0          // x11 = a1 = argv = 0
  jalr  x1, x12               // call actual main function; put return address in ra
```

##  11. <a name='Summary'></a>Summary

Lets provide a small summary of the points discussed so far.

We have setup the toolchain and we have compiled one of the example applications.

Instead of immediately starting with the code in the main() function, the C-runtime is
initialized first. After initializing stack and global pointers and registers, a core
0 check is performed and the system software pends on all cores othe than hart 0. The
system software on hart 0 has the power to wake up the other cores how it sees fit.
A single threaded application is created when the other cores are not woken up.

The take away is that your average hello world application will only
run on hart 0 because usually example applications will not wake up the other cores
by triggering software interrupts.

All other harts will sleep. The simulation of the NEORV32 will only show activity on hart 0.
You should not expect any activity on the other cores unless you trigger the software interrupt
to wake up the other harts.

##  12. <a name='Writeasimplemainfunction'></a>Write a simple main() function

Copy the folder sw\example\hello_world to a new folder. Call that new folder sw\example\add1

Check to see if the Makefile buildsystem picks up the new folder. Open a Msys2 MINGW64 console.

```
cd /c/Users/lapto/dev/VHDL/neorv32/sw/example
NEORV32_HOME=/C/Users/lapto/dev/VHDL/neorv32
PATH=/c/Users/lapto/Downloads/xpack-riscv-none-elf-gcc-14.2.0-3-win32-x64/xpack-riscv-none-elf-gcc-14.2.0-3/bin:$PATH
```

Adjust the paths to your local situation. Copy and pasting my paths from this tutorial without
adjustments would be a mistake!

Execute the all target.

```
make all
```

If the new add1 project is picked up by the toolchain, there should be output similar to:

```
$ make all
make[1]: Entering directory '/c/Users/lapto/dev/VHDL/neorv32/sw/example/add1'
Memory utilization:
   text    data     bss     dec     hex filename
   5052       0     256    5308    14bc main.elf
Generating neorv32_exe.bin
Executable size in bytes:
5064
Generating neorv32_raw_exe.hex
Generating neorv32_raw_exe.bin
Generating neorv32_raw_exe.coe
Generating neorv32_raw_exe.mem
Generating neorv32_raw_exe.mif
Generating neorv32_application_image.vhd
Installing application image to ../../../rtl/core/neorv32_application_image.vhd
make[1]: Leaving directory '/c/Users/lapto/dev/VHDL/neorv32/sw/example/add1'
```

##  13. <a name='EditingtheMainFunction'></a>Editing the Main Function

As a framework, use a most basic implementation of main.

```C
#include <neorv32.h>

int main() {
    return 0;
}
```

Check if it compiles still. (Expected output is the same as above)

```
cd /c/Users/lapto/dev/VHDL/neorv32/sw/example/add1
make all
```

##  14. <a name='InlineAssembly'></a>Inline Assembly

The next big hurdle is to make the C compiler output the machine code
for our custom add1 instruction.

Thinking this requirement through, the
compiler cannot possibly output the add1 instruction since the add1
instruction is not part of the RISC-V nonpriviledged specification and
hence the C-Compiler will not output this instruction because to the
compiler it simply does not exist!

The only chance to emit this instruction is to manually output the
machine code. Luckily the compiler has keywords for direct assembly output.
The process is called *inline assembly*.

GCC has the *asm ()* keyword (https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html)
for *inline assembly*.

In it's simplest form, you can output literal assembly code or
assembler instructions from within C code:

```C
asm ("nop")

asm (".word 0x1234")
```

If the assembly code has side effects only instead of manipulating data,
then the compiler might not anticipate the side effects and it might decide
to remove the assembly statements altogether because they are not part of
any data flow that the compiler can identify.

If it is required that the assembly code is executed precisely as written
down, then the compiler is not allowed to optimize the statements.

Obviously the compiler should not throw away or optmizie our
assembly statements.

In both cases, if deletion or optimization should be turned off for the
assembly code, the *volatile* keyword needs to be used.

```
asm volatile ("nop")
```

Remember that the compiler and assembler do not recognize
the new *add1* mnemonic and therefore direct machine code for the instruction
needs to be provided by the software engineer. As the compiler and
assembler will not perform encoding of the new, custom instruction for use, the idea
is to perform the encoding manually and emit machine code in form of hex numbers.

Using the *.word* assembler instruction makes it possible to directly insert
machine code into the assembly. This is required because of the reason
discussed above.

```
asm volatile (
    ".word

    ...
```

NEORV32 has a set of macros that perform instruction encoding using the .word
approach outlined above.

Here is the source code of the R-Type (register type) instruction encoding macro.
All credit goes to Stephan Nolting as always in all this tutorial.

```C
/**********************************************************************//**
 * @name R-type instruction format, RISC-V-standard
 **************************************************************************/
#define CUSTOM_INSTR_R_TYPE(funct7, rs2, rs1, funct3, opcode) \
({                                                            \
  uint32_t __return;                                          \
  asm volatile (                                              \
    ".word (                                                  \
      (((" #funct7 ") & 0x7f) << 25) |                        \
      (((  reg_%2   ) & 0x1f) << 20) |                        \
      (((  reg_%1   ) & 0x1f) << 15) |                        \
      (((" #funct3 ") & 0x07) << 12) |                        \
      (((  reg_%0   ) & 0x1f) <<  7) |                        \
      (((" #opcode ") & 0x7f) <<  0)                          \
    );"                                                       \
    : [rd] "=r" (__return)                                    \
    : "r" (rs1),                                              \
      "r" (rs2)                                               \
  );                                                          \
  __return;                                                   \
})
```

This is mind-bowling stuff especially if one has never used inline assembly.
The syntax is quite cryptic so let's go ahead and discuss the macro one step at a time.

Going forward, GCC inline assembly is flexible and it allows us to
insert data that is read from parameters.

###  14.1. <a name='StringConcatenation'></a>String Concatenation

Firstly, if there are local variables or parameters that should be pasted into the inline
assembly by value, then prefixing the variable names with a hash character '#'
causes the asm () keyword to concatenate the values of the variables into the
assembly code.

This feature is used in the R-Type macro above for the parameters funct7, funct3 and
opcode. This makes sense since funct7, funct3 and opcode are literal values to be
used as is during encoding and hence they are just concatenated into the output.

###  14.2. <a name='ReplacingPlaceholders'></a>Replacing Placeholders

The next feature is to paste values where placeholders are provided. This is very
similar to the format string of the printf() function family. With the printf()
functions, the first parameters is a so-called format string containing placeholders
followed by variables and values that are replaced where the placeholders are.

This replacement consists of three parts.

1. A format string
1. A way to provide placeholders along with an order (0, 1, 2, ...)
1. A way to provide the values which also clearly defines an order so that the correct placeholders
are replaced

The format string of inline assembly is the code inside the asm () keyword itself.

The placeholders are specified using the percent sign '%' followed by natural numbers starting
with 0 for the first parameter, 1 for the second ...

The actual parameters are specified after the format string in a colon-separated list.
A colon is the ':' character.

In the R-Type encoding macro above, the local __return variable is the actual parameter
number 0 (%0). The second actual parameter is the rs1 parameter and the third actual parameter
is the rs2 parameter.

Inside the format string, the placedholders are used as follows:

```C
reg_%0
reg_%1
reg_%2
```

It is hopefully clear by now that the first actual parameter will replce %0 in the format string.
Same goes for the other parameters in the order specified by the colon-separated list.

###  14.3. <a name='In-andout-parameters'></a>In- and out-parameters

The inline assembly *asm ()* keyword allows the programmer to make a distincion between variables
that go into the inline assembly as inputs and variables that get results assigned once the
inline assembly is executed.

This is dark compiler magic. I cannot even fathom how this feature is implemented.
In fact, the only thing I can do is to tell you that the *"=r"* string in the colon-separated
parameter list denotes output parameters whereas the *"r"* string denotes input-parameters.

In the RISC-V encoding schemes, the destination register is always generalized by the *rd* name
and input register 1 is called *rs1* and input register 2 is called *rs2*

In the encoding macro for R-Type instructions, *rd* is choosen by the compiler since the
local variable __return is used as an output parameter (=r). Whichever register __return is
placed into, that register will be used as *rd* in the statement above.

The input register *rs1* and *rs2* are hand crafted by the macro. The compiler has no say
in selecting *rs1* and *rs2*, instead it is forced to use the values specified via in
parameters (r).

This is the explanation of the encoding Macro! It allows you to encode custom RISC-V
instructions into the machine code that the compiler outputs. This works even although
the compiler and assembler have no clue about the custom mnemonics that you have defined
as you conduct computer science experiments or work on custom extensions that are not ratified yet.

###  14.4. <a name='Usage'></a>Usage

To use the macro, look at this example:

```C
/**********************************************************************//**
 * Single-precision floating-point addition
 *
 * @param[in] rs1 Source operand 1.
 * @param[in] rs2 Source operand 2.
 * @return Result.
 **************************************************************************/
inline float __attribute__ ((always_inline)) riscv_intrinsic_fadds(float rs1, float rs2) {

  float_conv_t opa, opb, res;
  opa.float_value = rs1;
  opb.float_value = rs2;

  res.binary_value = CUSTOM_INSTR_R_TYPE(0b0000000, opb.binary_value, opa.binary_value, 0b000, 0b1010011);
  return res.float_value;
}
```

The function carries the term intrinsic in it's name. The name intrinsic is used for
glue code that makes assembly or hardware based functionality usable from within the
C programming language.

The intrinsic above outputs the custom instruction and when executed returns the result
from the *rd* register back to the C code as the return value of the function.

A second approach is to only use the CUSTOM_INSTR_R_TYPE macro without processing the
return value.

```C
/**********************************************************************//**
 * @name Low-level CFU custom instruction prototypes ("intrinsics").
 * Note that each instruction provides a uint32_t return value.
 **************************************************************************/
/**@{*/
/** R-type CFU custom instruction (CUSTOM-0 opcode) */
#define neorv32_cfu_r_instr(funct7, funct3, rs1, rs2) CUSTOM_INSTR_R_TYPE(funct7, rs2, rs1, funct3, 0b0001011)
/** I-type CFU custom instruction (CUSTOM-1 opcode) */
#define neorv32_cfu_i_instr(funct3, imm12, rs1) CUSTOM_INSTR_I_TYPE(imm12, rs1, funct3, 0b0101011)
/**@}*/
```

The #define preprocessor instruction defines the symbol neorv32_cfu_r_instr which basically is
replaced with the CUSTOM_INSTR_R_TYPE macro.

Next, there are even more define preprocessor statements for each individual custom function.

```C
/**********************************************************************//**
 * @name Define macros for easy CFU instruction wrapping
 **************************************************************************/
/**@{*/
#define xtea_key_write(i, data)     neorv32_cfu_i_instr(0b001, i, data)
#define xtea_key_read(i)            neorv32_cfu_i_instr(0b000, i, 0   )
#define xtea_hw_init(sum)           neorv32_cfu_r_instr(0b0000000, 0b100, sum, 0 )
#define xtea_hw_enc_v0_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b000, v0,  v1)
#define xtea_hw_enc_v1_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b001, v0,  v1)
#define xtea_hw_dec_v0_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b010, v0,  v1)
#define xtea_hw_dec_v1_step(v0, v1) neorv32_cfu_r_instr(0b0000000, 0b011, v0,  v1)
#define xtea_hw_illegal_inst()      neorv32_cfu_r_instr(0b0000000, 0b111, 0,   0 )
/**@}*/
```

These custom functions are used in code:

```C
// set XTEA-CFU key storage (via CFU CSRs)
  xtea_key_write(0, key[0]);
  xtea_key_write(1, key[1]);
  xtea_key_write(2, key[2]);
  xtea_key_write(3, key[3]);
```

The simplest approach would probably be to call the MACRO directly from the main function.

```C
#include <neorv32.h>

int main() {

    CUSTOM_INSTR_R_TYPE(0b1000000, 1, 2, 0b000, 0b0110011);

    return 0;
}
```

Now rebuild the code.

```
NEORV32_HOME=/C/Users/lapto/dev/VHDL/neorv32
PATH=/c/Users/lapto/Downloads/xpack-riscv-none-elf-gcc-14.2.0-3-win32-x64/xpack-riscv-none-elf-gcc-14.2.0-3/bin:$PATH
cd /c/Users/lapto/dev/VHDL/neorv32/sw/example/add1
make all
```

Convert the .elf file into an assembly listing

```
riscv-none-elf-objdump --disassemble main.elf > listing.asm
```

or just use the existing main.asm which is created by the build.system
automatically.

The main function looks like this:

```
000001e4 <main>:
#include <neorv32.h>

int main() {

    CUSTOM_INSTR_R_TYPE(0b1000000, 1, 2, 0b000, 0b0110011);
 1e4:	00200793          	li	a5,2
 1e8:	00100713          	li	a4,1
 1ec:	80e787b3          	.word	0x80e787b3

    return 0;
 1f0:	00000513          	li	a0,0
 1f4:	00008067          	ret

```

The compiler has in fact loaded the hardcoded values 1 and 2 into registers
a5 and a4. This means the registers a5 and a4 are encoded into the .word
instruction.

Check that the hex data in the .word instruction matches the encoding you expect!

It needs to be the funct7 of the custom add1 instruction!


# CFU vs. CFS - Extending the NEORV32 CPU

There are two processor-internal options for custom hardware now: the Custom Functions Subsystem (CFS) and the Custom Functions Unit (CFU).

> CFU Complexity. The CFU is *not* intended for complex and CPU-independent functional units that implement complete accelerators (like full block-based AES encryption). These kind of accelerators should be implemented as memory-mapped co-processor via the Custom Functions Subsystem (CFS) to allow CPU-independent operation. A comparative survey of all NEORV32-specific hardware extension/customization options is provided in the user guide section Adding Custom Hardware Modules.

*Custom Functions Subsystem (CFS):* The CFS is a memory-mapped peripheral that is accessed using load/store instructions. It is intended for complex accelerators that - once triggered - perform some "long" processing in a CPU-independent manner (like a complete AES encryption). The CFS also provides the option to implement custom interfaces as it has direct access to special top entity signals.

*Custom Functions Unit (CFU):* The CFU is located right inside the CPU's pipeline. It is intended for custom instructions that implement certain functionality, which is not supported by the official (and supported) RISC-V ISA extensions. These instructions should be rather simple data transformations (like bit-reversal, summing elements in a vector, elementary AES operations, ...) rather than implementing a complete algorithm (even if this is also supported) since the CFU instructions are absolutely CPU-dependent and will stall the core until completed.

##  15. <a name='ExtendingtheNEORV32viatheCustomFunctionsUnitCFU'></a>Extending the NEORV32 via the Custom Functions Unit (CFU)

https://stnolting.github.io/neorv32/ug/#_custom_functions_subsystem
https://stnolting.github.io/neorv32/#_custom_functions_unit_cfu

> The Custom Functions Unit (CFU) is a functional unit that is integrated right into the CPU’s pipeline. It allows to implement custom RISC-V instructions. This extension option is intended for rather small logic that implements operations, which cannot be emulated in pure software in an efficient way. Since the CFU has direct access to the core’s register file it can operate with minimal data latency.

https://stnolting.github.io/neorv32/ug/#_comparative_summary

###  15.1. <a name='MergePullRequestthataddedtheCFU'></a>Merge/Pull Request that added the CFU

The pull request explains the CFU best.

https://github.com/stnolting/neorv32/pull/264

With this PR the NEORV32 now provides an option to add custom RISC-V instructions.

This PR adds a Custom Functions Unit (CFU) wrapped in the Zxcfu ISA extension, which is a NEORV32-specific custom ISA extension. The extension's name follows the RISC-V naming scheme:

Z = this is a sub-extension
x = the second letter behind the Z defines the "parent-extension" where this sub-extension belongs to: in this case it belongs to the X "custom extensions" extension (platform-specific extension that is not defined by the RISC-V spec.)
cfu = name of the extension (Custom Functions Unit)
The CFU is implemented as a new hardware module (rtl/core/neorv32_cpu_cp_cfu.vhd) that is integrated right into the CPU's ALU. Thus, the CFU has direct access to the core's register file, which provides minimal data transfer latency. A special OPCODE, which has been officially reserved for custom extensions by the RISC-V spec, is used to build custom instructions. The custom instructions supported by the CFU use the R2-type format that provides two source registers, one destinations register and a 10-bit immediate (split into two bit-fields:

The funct7 and funct3 bit-fields can be used to pass immediates to the CFU for certain computations (for example offsets, addresses, shift-amounts, ...) or they can be used to select the actual custom instruction to be executed (allowing up to 1024 different instructions).

Software can utilize the custom instruction by using the provides intrinsics (defined in sw/lib/include/neorv32_cpu_cfu.h. These pre-defined functions implicitly set the funct3 bit field. Each intrinsic can be treated as "normal C function" (see #263). A simple demo program using the default CFU hardware is available in sw/example/demo_cfu.

```
// custom instruction prototypes
neorv32_cfu_cmd0(funct7, rs1, rs2); // funct3 = 000
neorv32_cfu_cmd1(funct7, rs1, rs2); // funct3 = 001
neorv32_cfu_cmd2(funct7, rs1, rs2); // funct3 = 010
neorv32_cfu_cmd3(funct7, rs1, rs2); // funct3 = 011
neorv32_cfu_cmd4(funct7, rs1, rs2); // funct3 = 100
neorv32_cfu_cmd5(funct7, rs1, rs2); // funct3 = 101
neorv32_cfu_cmd6(funct7, rs1, rs2); // funct3 = 110
neorv32_cfu_cmd7(funct7, rs1, rs2); // funct3 = 111
```

This new feature was highly inspired by @google's CFU-Playground (https://github.com/google/CFU-Playground) - thanks again to @umarcor for showing me that framework. With some logic plumbing it should be possible to install the CFUs from the CFU-Playground into the NEORV32.

###  15.2. <a name='Questions'></a>Questions

Q: How does the NEORV32 CPU know which CFU to start or to call?
A: There is only a single CFU module. The module is integrated into the ALU as a coprocessor. Whenever a instruction is executed that uses the predefined custom-0 and custom-1 opcodes (See https://stnolting.github.io/neorv32/#_cfu_instruction_formats), the CFU is called.

Q: How does the CFU know which instruction has been executed?
A: All instruction using the opcodes custom-0 (0001011), used for CFU R-Type Instructions and custom-1 (0101011), used for CFU I-Type Instructions are processed by the ALU. Inside the ALU, the CFU entity is instantiated and it gets the funct7, funct3, funct12, rs1 and rs2 port mapped. The flag ctrl_i.alu_cp_cfu decides about if the CFU is activated and starts processing the data or not. ctrl_i.alu_cp_cfu is set during the EX_EXECUTE microcode step of the CPU control (rtl\core\neorv32_cpu_control.vhd)

```
-- CFU: custom RISC-V instructions --
when opcode_cust0_c | opcode_cust1_c =>
ctrl_nxt.alu_cp_cfu  <= '1'; -- trigger CFU co-processor
exe_engine_nxt.state <= EX_ALU_WAIT; -- will be aborted via monitor
                                        -- timeout if CFU is not implemented
```

opcode_cust0_c and opcode_cust1_c are the custom-0 and custom-1 opcodes mentioned above.

```
-- official custom RISC-V opcodes - free for custom instructions --
  constant opcode_cust0_c  : std_ulogic_vector(6 downto 0) := "0001011"; -- custom-0 (NEORV32 CFU)
  constant opcode_cust1_c  : std_ulogic_vector(6 downto 0) := "0101011"; -- custom-1 (NEORV32 CFU)
```

This means, when the EX_EXECUTE microcode step sees the custom opcodes, then it activates the CFU coprocessor inside the ALU. The CFU entity will then process the instructions and look at funct7 and funct3.

There are 1024 (1016 ???) possible custom CFU instructions possible based on the amount of combinations of funct7 and funct3. (2^7 * 2^3 = 1016). The idea is that there will not be one separate CFU entity per extension but instead, every extension will share the same entity.

Currently the code contains a sample implementation of XTEA. XTEA uses about eight custom instructions. The XTEA CFU constains instructions to set and get keys, to perform encryption and decryption.






##  16. <a name='ExtendingtheNEORV32viatheCustomFunctionsSubsystemCFS'></a>Extending the NEORV32 via the Custom Functions Subsystem (CFS)

https://stnolting.github.io/neorv32/ug/#_custom_functions_subsystem
https://stnolting.github.io/neorv32/#_custom_functions_subsystem_cfs

### The interface of CFS

CFS is basically a way to define a co-processor.
The CPU and the CFS co-processor communicate via an interface.

The interface consists of

* bus requests that enter the CFS co-processor and bus responses that exit the CFS co-processor.
* An interrupt line from the CFS co-processor towards the CPU.
* And a 255 bit bus called cfs_in_i into the co-processor and a output bus out of the co-processor called cfs_out_o.

### Bus Requests

The bus requests are defined as such:

```
-- bus request --
type bus_req_t is record
    meta  : std_ulogic_vector(2 downto 0); -- access meta information
    addr  : std_ulogic_vector(31 downto 0); -- access address
    data  : std_ulogic_vector(31 downto 0); -- write data
    ben   : std_ulogic_vector(3 downto 0); -- byte enable
    stb   : std_ulogic; -- request strobe, single-shot
    rw    : std_ulogic; -- 0 = read, 1 = write
    amo   : std_ulogic; -- set if atomic memory operation
    amoop : std_ulogic_vector(3 downto 0); -- type of atomic memory operation
    burst : std_ulogic; -- set if part of burst access
    lock  : std_ulogic; -- set if exclusive access request
    -- out-of-band signals --
    fence : std_ulogic; -- set if fence(.i) operation, single-shot
end record;

bus_req_i : in  bus_req_t; -- bus request
```

and

```
-- bus response --
type bus_rsp_t is record
    ack  : std_ulogic; -- set if access acknowledge, single-shot
    err  : std_ulogic; -- set if access error, valid if ack = 1
    data : std_ulogic_vector(31 downto 0); -- read data, valid if ack = 1
end record;

bus_rsp_o : out bus_rsp_t; -- bus response
```

The request has an address and data and an rw flag with identifies it as either a read or a write operation.

The response has a value field that carries data in case of read operations. If the request is a write request, then the data has to be zero!

```
-- bus access --
      bus_rsp_o.data <= (others => '0'); -- the output HAS TO BE ZERO if there is no actual
                                         -- (read) access
```

When the CPU wants to execute a request, it will strobe for a single cycle. The CFS co-processor will acknowledge on the next cycle by setting ack high for a cycle. If the co-processor needs more time to process the request, it will acknowledge on a later cycle! If the CPU sees no ACK, then it will eventually time out! It is unclear how long the timeout is!

err is pulled lo if there is no error.

```
-- transfer/access acknowledge --
      bus_rsp_o.ack <= bus_req_i.stb; -- send ACK right after the access request
      bus_rsp_o.err <= '0'; -- set high together with bus_rsp_o.ack if there is an access error
```

Besides the bus_req_i and bus_rsp_o objects, there are two 255 bit busses called cfs_in_i and cfs_out_o. They are also referred to as conduits. I do not know if the data transfer over the conduits also needs request and acknowledge in the bus request and bus response or if the conduits are completely independant of the bus request/response.

The example just sets the out conduit to zero:

```
cfs_out_o <= (others => '0');
```

This means the example does not even make use of the conduits.

### Writing Software that uses CFS

First, use the CFS-HAL function defined in neorv32_cfs.c/h called int neorv32_cfs_available(void) to determined, if the CFS has been enabled in the configuration and therefore has even been synethsized. Once verified that it is synthesized, the usage can continue.

As outlined in the NEORV32 datasheet (Section "CFS Software Access"), the official way to access the CFS is to write into the memory mapped registers of the CFS module directly from the application:

Source: https://stnolting.github.io/neorv32/ > CFS Software Access > Listing 15. CFS Software Access Example

```
// C-code CFS usage example
NEORV32_CFS->REG[0] = (uint32_t)some_data_array(i); // write to CFS register 0
int temp = (int)NEORV32_CFS->REG[20]; // read from CFS register 20
```

Writing to the register directly will cause a memory read or write to/from a specific address. Internally inside the NEORV32 CPU, this will cause a bus access.

The bus is implemented in such a way that a read/write request to a CFS register is translated into a bus_req_t object which was described earlier as part of the interface of the CFS entity.

The file rtl\core\neorv32_bus.vhd contains the bus implementation. the neorv32_bus_io_switch entity is the basic switching table that will connect users to memory mapped hardware behind the scenes just like on of those classic telephone switching tables back in the day of analog phone switching where thick cinch cables had been used to connect the caller to the callee.

Firstly, the CFS entity is instaniated inside rtl\core\neorv32_top.vhd

```
neorv32_cfs_enabled:
if IO_CFS_EN generate
    neorv32_cfs_inst: entity neorv32.neorv32_cfs
    port map (
    clk_i       => clk_i,
    rstn_i      => rstn_sys,
    bus_req_i   => iodev_req(IODEV_CFS),
    bus_rsp_o   => iodev_rsp(IODEV_CFS),
    irq_o       => firq(FIRQ_CFS),
    cfs_in_i    => cfs_in_i,
    cfs_out_o   => cfs_out_o
    );
end generate;
```

bus_req_i is connected to iodev_req(IODEV_CFS).

Inside rtl\core\neorv32_top.vhd, dev_11_req_o is assigned to iodev_req(IODEV_CFS).

```
dev_11_req_o => iodev_req(IODEV_CFS),     dev_11_rsp_i => iodev_rsp(IODEV_CFS),
```

dev_11_req_o is a port of the neorv32_bus_io_switch as can be seen in rtl\core\neorv32_bus.vhd on line 547.

Looking at the label bus_request_gen in rtl\core\neorv32_bus.vhd, if a request for an address falls in the range of a certain entry in the device base list, then the request is forwarded to that device base list entry.

```
if (main_req.addr(addr_hi_c downto addr_lo_c) = dev_base_list_c(i)(addr_hi_c downto addr_lo_c)) then
    dev_req(i).stb <= main_req.stb; -- propagate transaction strobe if address match
```

Entry 11 of the device base list is used by the CFS.

neorv32_bus_io_switch is a generic entity. This means, during it's initialization, values need to be supplied for it's generic parameters. These concrete values contain the base addresses for all the device base list entries.

For example the generic parameter DEV_11_BASE for the CFS is then used like this:

```
-- list of device base addresses --
  type dev_base_list_t is array (0 to num_devs_c-1) of std_ulogic_vector(31 downto 0);
  constant dev_base_list_c : dev_base_list_t := (
    DEV_00_BASE, DEV_01_BASE, DEV_02_BASE, DEV_03_BASE, DEV_04_BASE, DEV_05_BASE, DEV_06_BASE, DEV_07_BASE,
    DEV_08_BASE, DEV_09_BASE, DEV_10_BASE, DEV_11_BASE, DEV_12_BASE, DEV_13_BASE, DEV_14_BASE, DEV_15_BASE,
    DEV_16_BASE, DEV_17_BASE, DEV_18_BASE, DEV_19_BASE, DEV_20_BASE, DEV_21_BASE, DEV_22_BASE, DEV_23_BASE,
    DEV_24_BASE, DEV_25_BASE, DEV_26_BASE, DEV_27_BASE, DEV_28_BASE, DEV_29_BASE, DEV_30_BASE, DEV_31_BASE
  );
```

This means if during instantiation a address is given for the DEV_11_BASE, then this address is used for the CFS subsystem and requests are routed to the CFS.

The instatiation of neorv32_bus_io_switch is contained in rtl\core\neorv32_top.vhd

```
-- IO Switch ------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_bus_io_switch_inst: entity neorv32.neorv32_bus_io_switch
    generic map (
      INREG_EN  => true,
      OUTREG_EN => true,
      DEV_SIZE  => iodev_size_c,
      DEV_00_EN => bootrom_en_c,    DEV_00_BASE => base_io_bootrom_c,
      DEV_01_EN => false,           DEV_01_BASE => (others => '0'), -- reserved
      DEV_02_EN => false,           DEV_02_BASE => (others => '0'), -- reserved
      DEV_03_EN => false,           DEV_03_BASE => (others => '0'), -- reserved
      DEV_04_EN => false,           DEV_04_BASE => (others => '0'), -- reserved
      DEV_05_EN => false,           DEV_05_BASE => (others => '0'), -- reserved
      DEV_06_EN => false,           DEV_06_BASE => (others => '0'), -- reserved
      DEV_07_EN => false,           DEV_07_BASE => (others => '0'), -- reserved
      DEV_08_EN => false,           DEV_08_BASE => (others => '0'), -- reserved
      DEV_09_EN => false,           DEV_09_BASE => (others => '0'), -- reserved
      DEV_10_EN => IO_TWD_EN,       DEV_10_BASE => base_io_twd_c,
      DEV_11_EN => IO_CFS_EN,       DEV_11_BASE => base_io_cfs_c,
      DEV_12_EN => IO_SLINK_EN,     DEV_12_BASE => base_io_slink_c,
      DEV_13_EN => IO_DMA_EN,       DEV_13_BASE => base_io_dma_c,
      DEV_14_EN => false,           DEV_14_BASE => (others => '0'), -- reserved
      DEV_15_EN => false,           DEV_15_BASE => (others => '0'), -- reserved
      DEV_16_EN => io_pwm_en_c,     DEV_16_BASE => base_io_pwm_c,
      DEV_17_EN => IO_GPTMR_EN,     DEV_17_BASE => base_io_gptmr_c,
      DEV_18_EN => IO_ONEWIRE_EN,   DEV_18_BASE => base_io_onewire_c,
      DEV_19_EN => IO_TRACER_EN,    DEV_19_BASE => base_io_tracer_c,
      DEV_20_EN => IO_CLINT_EN,     DEV_20_BASE => base_io_clint_c,
      DEV_21_EN => IO_UART0_EN,     DEV_21_BASE => base_io_uart0_c,
      DEV_22_EN => IO_UART1_EN,     DEV_22_BASE => base_io_uart1_c,
      DEV_23_EN => IO_SDI_EN,       DEV_23_BASE => base_io_sdi_c,
      DEV_24_EN => IO_SPI_EN,       DEV_24_BASE => base_io_spi_c,
      DEV_25_EN => IO_TWI_EN,       DEV_25_BASE => base_io_twi_c,
      DEV_26_EN => IO_TRNG_EN,      DEV_26_BASE => base_io_trng_c,
      DEV_27_EN => IO_WDT_EN,       DEV_27_BASE => base_io_wdt_c,
      DEV_28_EN => io_gpio_en_c,    DEV_28_BASE => base_io_gpio_c,
      DEV_29_EN => IO_NEOLED_EN,    DEV_29_BASE => base_io_neoled_c,
      DEV_30_EN => io_sysinfo_en_c, DEV_30_BASE => base_io_sysinfo_c,
      DEV_31_EN => OCD_EN,          DEV_31_BASE => base_io_ocd_c
    )
    port map (
      clk_i        => clk_i,
      rstn_i       => rstn_sys,
      main_req_i   => io_req,
      main_rsp_o   => io_rsp,
      dev_00_req_o => iodev_req(IODEV_BOOTROM), dev_00_rsp_i => iodev_rsp(IODEV_BOOTROM),
      dev_01_req_o => open,                     dev_01_rsp_i => rsp_terminate_c, -- reserved
      dev_02_req_o => open,                     dev_02_rsp_i => rsp_terminate_c, -- reserved
      dev_03_req_o => open,                     dev_03_rsp_i => rsp_terminate_c, -- reserved
      dev_04_req_o => open,                     dev_04_rsp_i => rsp_terminate_c, -- reserved
      dev_05_req_o => open,                     dev_05_rsp_i => rsp_terminate_c, -- reserved
      dev_06_req_o => open,                     dev_06_rsp_i => rsp_terminate_c, -- reserved
      dev_07_req_o => open,                     dev_07_rsp_i => rsp_terminate_c, -- reserved
      dev_08_req_o => open,                     dev_08_rsp_i => rsp_terminate_c, -- reserved
      dev_09_req_o => open,                     dev_09_rsp_i => rsp_terminate_c, -- reserved
      dev_10_req_o => iodev_req(IODEV_TWD),     dev_10_rsp_i => iodev_rsp(IODEV_TWD),
      dev_11_req_o => iodev_req(IODEV_CFS),     dev_11_rsp_i => iodev_rsp(IODEV_CFS),
      dev_12_req_o => iodev_req(IODEV_SLINK),   dev_12_rsp_i => iodev_rsp(IODEV_SLINK),
      dev_13_req_o => iodev_req(IODEV_DMA),     dev_13_rsp_i => iodev_rsp(IODEV_DMA),
      dev_14_req_o => open,                     dev_14_rsp_i => rsp_terminate_c, -- reserved
      dev_15_req_o => open,                     dev_15_rsp_i => rsp_terminate_c, -- reserved
      dev_16_req_o => iodev_req(IODEV_PWM),     dev_16_rsp_i => iodev_rsp(IODEV_PWM),
      dev_17_req_o => iodev_req(IODEV_GPTMR),   dev_17_rsp_i => iodev_rsp(IODEV_GPTMR),
      dev_18_req_o => iodev_req(IODEV_ONEWIRE), dev_18_rsp_i => iodev_rsp(IODEV_ONEWIRE),
      dev_19_req_o => iodev_req(IODEV_TRACER),  dev_19_rsp_i => iodev_rsp(IODEV_TRACER),
      dev_20_req_o => iodev_req(IODEV_CLINT),   dev_20_rsp_i => iodev_rsp(IODEV_CLINT),
      dev_21_req_o => iodev_req(IODEV_UART0),   dev_21_rsp_i => iodev_rsp(IODEV_UART0),
      dev_22_req_o => iodev_req(IODEV_UART1),   dev_22_rsp_i => iodev_rsp(IODEV_UART1),
      dev_23_req_o => iodev_req(IODEV_SDI),     dev_23_rsp_i => iodev_rsp(IODEV_SDI),
      dev_24_req_o => iodev_req(IODEV_SPI),     dev_24_rsp_i => iodev_rsp(IODEV_SPI),
      dev_25_req_o => iodev_req(IODEV_TWI),     dev_25_rsp_i => iodev_rsp(IODEV_TWI),
      dev_26_req_o => iodev_req(IODEV_TRNG),    dev_26_rsp_i => iodev_rsp(IODEV_TRNG),
      dev_27_req_o => iodev_req(IODEV_WDT),     dev_27_rsp_i => iodev_rsp(IODEV_WDT),
      dev_28_req_o => iodev_req(IODEV_GPIO),    dev_28_rsp_i => iodev_rsp(IODEV_GPIO),
      dev_29_req_o => iodev_req(IODEV_NEOLED),  dev_29_rsp_i => iodev_rsp(IODEV_NEOLED),
      dev_30_req_o => iodev_req(IODEV_SYSINFO), dev_30_rsp_i => iodev_rsp(IODEV_SYSINFO),
      dev_31_req_o => iodev_req(IODEV_OCD),     dev_31_rsp_i => iodev_rsp(IODEV_OCD)
    );
```

This massive instantiation enables the CFS if configured. It will assign the base address and at the same time map the correct bus request and response signals to the respective ports:

```
dev_11_req_o => iodev_req(IODEV_CFS),     dev_11_rsp_i => iodev_rsp(IODEV_CFS),
```



### The example

The provided CFS example

