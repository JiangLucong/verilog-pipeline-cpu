# verilog-pipeline-cpu

用 **Verilog** 设计并实现的 **16 位五级流水线微处理器**（IF / ID / EX / MEM / WB），自定义指令集，并完整处理数据冒险与控制冒险。

---

## 架构

- **流水线**：取指（IF）→ 译码（ID）→ 执行（EX）→ 访存（MEM）→ 写回（WB）。
- **数据通路**：16 位字长，8 个通用寄存器，独立的指令存储器与数据存储器（哈佛结构）。
- **冒险处理**
  - **数据冒险**：EX/MEM/WB 到 ID 的**前向通路（data forwarding）**；对 load-use 相关插入**流水线停顿（stall）**。
  - **控制冒险**：跳转 / 分支在 EX 解析后**冲刷（flush）**错误取指并重定向 PC；分支所需标志位提前旁路（flag bypass）。

## 指令集（5-bit opcode）

| 类别 | 指令 |
|---|---|
| 控制 | `NOP` `HALT` |
| 访存 | `LOAD` `STORE` `LDIH`(load immediate high) |
| 算术 | `ADD` `ADDI` `SUB` `SUBI` `ADDC` `SUBC` `CMP` |
| 逻辑 | `AND` `OR` `XOR` |
| 移位 | `SLL` `SRL` `SLA` `SRA` |
| 跳转 | `JUMP` `JMPR`(jump register) |
| 分支 | `BZ` `BNZ` `BN` `BNN` `BC` `BNC` |
| **自定义** | **`LOOP`** —— 自行扩展的**硬件循环指令**，在硬件层面完成计数与回跳 |

## 文件

```
src/
├── pcpu.v          # 五级流水线 CPU 核心（数据通路、冒险处理、ALU、控制 FSM）
├── processor.v     # 顶层：CPU + 指令/数据存储器
├── t_processor.v   # 测试平台（testbench）
├── pcpu.scr        # Synopsys VCS 仿真脚本
├── imem.ini        # 指令存储器初始化（测试程序）
└── dmem.ini        # 数据存储器初始化
```

## 仿真

测试平台用 `$readmemh` 从 `imem.ini` / `dmem.ini` 载入程序与数据，运行结束后把数据存储器转储到 `dmem.mem`。

```bash
cd src
# Synopsys VCS：
vcs -f pcpu.scr && ./simv
# 或 Icarus Verilog：
iverilog -o sim t_processor.v processor.v && vvp sim
```

> 由于 `processor.v` 中使用了 `` `include "pcpu.v" ``，且存储器初始化文件按工作目录相对路径读取，请在 `src/` 目录下运行仿真。

## 许可证

[MIT](LICENSE)。
