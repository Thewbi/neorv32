# NEORV32 Documents

Notes and material on the NEORV32 VHDL CPU

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
CPU processing relevant data.

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

```
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

neorv32_cpu.vhd 

```
  -- Control Unit (Back-End / Instruction Execution) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: entity neorv32.neorv32_cpu_control
    generic map (
        ...
    )
    port map (

        ...

        -- [debug] - make instruction visible
        wbi_valid_i   => frontend.valid,       -- bus signals are valid
        wbi_instr_i   => frontend.instr,       -- instruction

        ...

    );
```

neorv32_cpu_control.vhd

```
entity neorv32_cpu_control is
    generic (
        ...
    );
    port (

        ...

        -- [debug] - make instruction visible
        wbi_valid_i   : in std_ulogic;                          -- bus signals are valid
        wbi_instr_i   : in std_ulogic_vector(31 downto 0);      -- instruction

        ...

    );
end neorv32_cpu_control;
```

GHDL will output the following trace:

![image info](res/images/DebuggingInstructions.png)

This trace shows that the frontend prefetches instructions in non-intuitive ways. For example the compressed instruction 0x00002b27 is fetched, then 0x80028293 is fetched and then 0x00002b27 is fetched again!

The execution engine is displayed in the lower traces thanks to the debug-signals inserted above.

The instr traces contains the instruction that the execution engine sees and the valid trace dictates if the execution engine is triggered to consume the instruction. If the valid signal is low, the execution engine ignores the instruction.

The execution engine traces show which instructions are executed by the CPU.
