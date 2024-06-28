const std = @import("std");
const processor = @import("cpu.zig");
const Z80State = processor.Z80State;
const RegisterPairs = processor.RegisterPairs1;

// Z80 instructions are represented in memory as byte sequences of the form (items in brackets are optional):
// [prefix byte,]  opcode  [,displacement byte]  [,immediate data]
// - OR -
// two prefix bytes,  displacement byte,  opcode
//
// 7 6 5 4 3 2 1 0
// x x y y y z z z
// ─┬─ ──┬── ──┬──
//  │    │     └───── zzz
//  │    └─────────── yyy = ppq
//  └────────────────  xx
//
pub const OpCode = packed struct {
    z: u3, // 0-1-2
    y: u3, // 3-4-5
    x: u2, // 6-7
};

// Z80 Flags:
//   7 6 5 4 3 2  1 0
//   S Z Y H X PV N C
//   │ │ │ │ │  │ │ │
//   │ │ │ │ │  │ │ └─  C: Carry (add) / Borrow (sub)
//   │ │ │ │ │  │ └───  N: Addition / Subtraction
//   │ │ │ │ │  └───── PV: Parity (P) / Two complement signed overflow (V)
//   │ │ │ │ └────────  X: Result's 3rd bit (undocumented)
//   │ │ │ └──────────  H: Half carry / Half borrow
//   │ │ └────────────  Y: Result's 5th bit (undocumented)
//   │ └──────────────  Z: Zero
//   └────────────────  S: Sign
// See also: https://github.com/hoglet67/Z80Decoder/wiki/Undocumented-Flags
//
const SF = 128;
const YF = 32;
const HF = 16;
const XF = 8;
const PVF = 4;

const InstructionFn = *const fn (*Z80State, *const OpCode) void;

const AddressingMode = enum {
    register, // r
    indirect, // (HL)
    indexed, // (IX+d) or (IY+d)
    immediate, // n
    extended, // (nn)
};

const BitwiseOp = enum { AND, XOR, OR };

/// see: https://web.archive.org/web/20170121033813/http://www.cs.umd.edu:80/class/spring2003/cmsc311/Notes/Comb/overflow.html
inline fn overflow8(result: u8, lhs: u8, rhs: u8) u8 {
    return ((((lhs ^ rhs) & (lhs ^ result)) >> (8 - 3)) & PVF);
}

/// see: Hacker's delight Chapter 5-2 "Parity", for an in-depth explanation
fn hasEvenParity8(x: u8) bool {
    const p = @as(u16, 0x6996) >> @intCast((x ^ (x >> 4)) & 0xF);
    return p & 1 == 0; // 0 = even
}

pub inline fn readRegister(state: *Z80State, r: u8) u8 {
    const value = state.gp_registers[r];
    return value;
}

pub inline fn readImmediate(state: *Z80State, comptime offset: u8) u8 {
    const value = state.memory[state.PC +% offset];
    return value;
}

pub inline fn readExtended(state: *Z80State, comptime offset: u8) u8 {
    const low = state.memory[state.PC + offset];
    const high = state.memory[state.PC + offset + 1];
    const address: u16 = @intCast(low | @as(u16, high) << 8);
    const value = state.memory[address];
    return value;
}

pub inline fn readIndirect(state: *Z80State, rp: u8) u8 {
    const address = state.gp_registers_pairs[rp].getValue();
    const value = state.memory[address];
    return value;
}

pub fn readIndexed(state: *Z80State, comptime offset: u8) u8 {
    const base_address: i32 = @intCast(state.addr_register.getValue());
    const d: i8 = @bitCast(state.memory[state.PC + offset]); // displacement is two's complement
    const address: u16 = @intCast(@mod(base_address + d, 65536));
    const value = state.memory[address];
    return value;
}

pub inline fn storeRegister(state: *Z80State, r: u8, value: u8) void {
    state.gp_registers[r] = value;
}

pub inline fn storeIndirect(state: *Z80State, value: u8, rp: u8) void {
    const address = state.gp_registers_pairs[rp].getValue();
    state.memory[address] = value;
}

pub inline fn storeIndexed(state: *Z80State, value: u8, comptime offset: u8) void {
    const base_address: i32 = @intCast(state.addr_register.getValue());
    const d: i8 = @bitCast(state.memory[state.PC + offset]); // displacement is two's complement
    const address: u16 = @intCast(@mod(base_address + d, 65536));
    state.memory[address] = value;
}

pub inline fn add_a_x(s: *Z80State, rhs: u8) void {
    const t = s.AF.A +% rhs; // +% = wrapping addition
    s.AF.F.N = false;
    s.AF.F.PV = overflow8(t, s.AF.A, ~rhs) != 0; // negated rhs for sum
    s.AF.F.C = @as(u32, rhs) + s.AF.A > 255;
    s.AF.F.Z = t == 0;
    s.AF.F.H = (s.AF.A ^ rhs ^ t) & HF != 0;
    s.AF.A = t;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.X = t & XF != 0; // 3rd bit
    s.AF.F.Y = t & YF != 0; // 5th bit
}

pub inline fn adc_a_x(s: *Z80State, rhs: u8) void {
    const carry = @intFromBool(s.AF.F.C);
    const t = s.AF.A +% rhs +% carry;
    s.AF.F.N = false;
    s.AF.F.PV = overflow8(t, s.AF.A, ~rhs) != 0;
    s.AF.F.C = @as(u32, rhs) + s.AF.A + carry > 255;
    s.AF.F.Z = t == 0;
    s.AF.F.H = (s.AF.A ^ rhs ^ t) & HF != 0;
    s.AF.A = t;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.X = t & XF != 0; // 3rd bit
    s.AF.F.Y = t & YF != 0; // 5th bit
}

pub inline fn sub_a_x(s: *Z80State, rhs: u8) void {
    const t = s.AF.A -% rhs; // +% = wrapping subtraction
    const borrow = s.AF.A < rhs;
    s.AF.F.N = true;
    s.AF.F.PV = overflow8(t, s.AF.A, rhs) != 0;
    s.AF.F.C = borrow;
    s.AF.F.Z = t == 0;
    s.AF.F.H = (s.AF.A ^ rhs ^ t) & HF != 0;
    s.AF.A = t;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.X = t & XF != 0; // 3rd bit
    s.AF.F.Y = t & YF != 0; // 5th bit
}

pub inline fn sbc_a_x(s: *Z80State, rhs: u8) void {
    const carry = @intFromBool(s.AF.F.C);
    const t = s.AF.A -% rhs -% carry;
    s.AF.F.N = true;
    s.AF.F.PV = overflow8(t, s.AF.A, rhs) != 0;
    s.AF.F.C = @as(i32, s.AF.A) - rhs - carry < 0;
    s.AF.F.Z = t == 0;
    s.AF.F.H = (s.AF.A ^ rhs ^ t) & HF != 0;
    s.AF.A = t;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.X = t & XF != 0; // 3rd bit
    s.AF.F.Y = t & YF != 0; // 5th bit
}

pub inline fn btw_a_x(comptime OP: BitwiseOp, s: *Z80State, rhs: u8) void {
    const t = switch (OP) {
        inline .AND => s.AF.A & rhs,
        inline .XOR => s.AF.A ^ rhs,
        inline .OR => s.AF.A | rhs,
    };
    s.AF.A = t;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.Z = t == 0;
    s.AF.F.N = false;
    s.AF.F.H = switch (OP) {
        inline .AND => true,
        inline .XOR => false,
        inline .OR => false,
    };
    s.AF.F.C = false;
    s.AF.F.PV = hasEvenParity8(t);
    s.AF.F.X = t & XF != 0; // 3rd bit
    s.AF.F.Y = t & YF != 0; // 5th bit
}

pub fn and_a_x(s: *Z80State, rhs: u8) void {
    return btw_a_x(BitwiseOp.AND, s, rhs);
}

pub fn or_a_x(s: *Z80State, rhs: u8) void {
    return btw_a_x(BitwiseOp.OR, s, rhs);
}

pub fn xor_a_x(s: *Z80State, rhs: u8) void {
    return btw_a_x(BitwiseOp.XOR, s, rhs);
}

pub fn cp_a_x(s: *Z80State, rhs: u8) void {
    const t = s.AF.A -% rhs;
    const borrow = s.AF.A < rhs;
    s.AF.F.N = true;
    s.AF.F.PV = overflow8(t, s.AF.A, rhs) != 0;
    s.AF.F.C = borrow;
    s.AF.F.Z = t == 0;
    s.AF.F.H = (s.AF.A ^ rhs ^ t) & HF != 0;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.X = rhs & XF != 0; // 3rd bit
    s.AF.F.Y = rhs & YF != 0; // 5th bit
}

pub fn inc_dec_x(s: *Z80State, opcode: *const OpCode, value: u8) u8 {
    const dec: u8 = opcode.z & 1; // (0) inc, (1) dec
    const nf: u8 = dec << 1; // 2 if dec, 1 if inc
    const t = value +% 1 -% nf;
    s.AF.F.N = dec == 1;
    s.AF.F.Z = t == 0;
    s.AF.F.H = (value ^ t) & HF != 0;
    s.AF.F.PV = @intFromBool(value == 127 + dec) != 0;
    s.AF.F.S = t & SF != 0; // sign bit
    s.AF.F.X = t & XF != 0; // 3rd bit
    s.AF.F.Y = t & YF != 0; // 5th bit
    return t;
}

pub inline fn al_a_x(comptime addressing: AddressingMode, state: *Z80State, opcode: *const OpCode) void {
    const rhs = switch (addressing) {
        inline .register => readRegister(state, opcode.z),
        inline .immediate => readImmediate(state, 1),
        inline .indexed => readIndexed(state, 1),
        inline .indirect => readIndirect(state, @intFromEnum(RegisterPairs.HL)),
        else => std.debug.panic("Unsupported addressing {any} for instruction: 0x{x}\n", .{ addressing, opcode }),
    };

    const operand_size = switch (addressing) {
        inline .immediate => 1, // e.g 86 n : +1 for n
        inline .indexed => 1, // e.g DD 86 d : +1 for d
        else => 0,
    };

    state.PC +%= 1 + operand_size;

    switch (opcode.y) {
        0 => add_a_x(state, rhs), // ADD A,
        1 => adc_a_x(state, rhs), // ADC A,
        2 => sub_a_x(state, rhs), // SUB A,
        3 => sbc_a_x(state, rhs), // SBC A,
        4 => and_a_x(state, rhs), // AND A,
        5 => xor_a_x(state, rhs), // XOR A,
        6 => or_a_x(state, rhs), //   OR A,
        7 => cp_a_x(state, rhs), //   CP A,
    }
}

pub inline fn al2_a_x(comptime addressing: AddressingMode, state: *Z80State, opcode: *const OpCode) void {
    const value = switch (addressing) {
        inline .register => readRegister(state, opcode.y),
        inline .indirect => readIndirect(state, @intFromEnum(RegisterPairs.HL)),
        inline .indexed => readIndexed(state, 1),
        else => std.debug.panic("Unsupported addressing {any} for instruction: 0x{x}\n", .{ addressing, opcode }),
    };

    const operand_size = switch (addressing) {
        inline .indexed => 1, // e.g DD 86 d : +1 for d
        else => 0,
    };

    const result = inc_dec_x(state, opcode, value);

    switch (addressing) {
        .register => storeRegister(state, opcode.y, result),
        .indirect => storeIndirect(state, result, @intFromEnum(RegisterPairs.HL)),
        .indexed => storeIndexed(state, result, 1),
        else => std.debug.panic("Unsupported addressing {any} for instruction: 0x{x}\n", .{ addressing, opcode }),
    }

    state.PC +%= 1 + operand_size;
}

pub fn al2_a_r(state: *Z80State, opcode: *const OpCode) void {
    al2_a_x(.register, state, opcode);
}

pub fn al2_a_hl(state: *Z80State, opcode: *const OpCode) void {
    al2_a_x(.indirect, state, opcode);
}

pub fn al2_a_xy(state: *Z80State, opcode: *const OpCode) void {
    al2_a_x(.indexed, state, opcode);
}

pub fn al_a_r(state: *Z80State, opcode: *const OpCode) void {
    al_a_x(.register, state, opcode);
}

pub fn al_a_hl(state: *Z80State, opcode: *const OpCode) void {
    al_a_x(.indirect, state, opcode);
}

pub fn al_a_xy(state: *Z80State, opcode: *const OpCode) void {
    al_a_x(.indexed, state, opcode);
}

pub fn al_a_n(state: *Z80State, opcode: *const OpCode) void {
    al_a_x(.immediate, state, opcode);
}

pub fn ij_prefix(state: *Z80State, _: *const OpCode) void {
    state.R.increment();
    state.PC +%= 1;
    const opcode_int = fetchOpcode(state);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = xy_instructions_table[opcode_int];
    insn_func(state, &opcode);
}

pub fn dd_prefix(state: *Z80State, opcode: *const OpCode) void {
    state.addr_register = &state.IX;
    ij_prefix(state, opcode);
}

pub fn fd_prefix(state: *Z80State, opcode: *const OpCode) void {
    state.addr_register = &state.IY;
    ij_prefix(state, opcode);
}

pub fn ed_prefix(state: *Z80State, _: *const OpCode) void {
    state.PC +%= 1;
    const opcode_int = fetchOpcode(state);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = ed_instructions_table[opcode_int];
    insn_func(state, &opcode);
}

pub fn ld_r_r(state: *Z80State, opcode: *const OpCode) void {
    state.gp_registers[opcode.y] = state.gp_registers[opcode.z];
    state.PC +%= 1;
}

pub fn ld_r_n(state: *Z80State, opcode: *const OpCode) void {
    state.gp_registers[opcode.y] = readImmediate(state, 1);
    state.PC +%= 2;
}

pub fn ld_r_hl(state: *Z80State, opcode: *const OpCode) void {
    state.gp_registers[opcode.y] = readIndirect(state, @intFromEnum(RegisterPairs.HL));
    state.PC +%= 1;
}

pub fn ld_a_bc(state: *Z80State, _: *const OpCode) void {
    const dst = @intFromEnum(processor.EightBitRegisters.A);
    state.gp_registers[dst] = readIndirect(state, @intFromEnum(RegisterPairs.BC));
    state.PC +%= 1;
}

pub fn ld_a_de(state: *Z80State, _: *const OpCode) void {
    const dst = @intFromEnum(processor.EightBitRegisters.A);
    state.gp_registers[dst] = readIndirect(state, @intFromEnum(RegisterPairs.DE));
    state.PC +%= 1;
}

pub fn ld_r_xy(state: *Z80State, opcode: *const OpCode) void {
    state.gp_registers[opcode.y] = readIndexed(state, 1);
    state.PC +%= 2;
}

pub fn ld_a_nn(state: *Z80State, opcode: *const OpCode) void {
    state.gp_registers[opcode.y] = readExtended(state, 1);
    state.PC +%= 3;
}

pub fn ld_a_i(state: *Z80State, _: *const OpCode) void {
    state.PC +%= 1;
    state.AF.A = state.I;
    state.AF.F.S = state.I & SF != 0;
    state.AF.F.Z = state.I == 0;
    state.AF.F.H = false;
    state.AF.F.PV = state.IFF2;
    state.AF.F.N = false;
    state.AF.F.X = state.I & XF != 0; // 3rd bit
    state.AF.F.Y = state.I & YF != 0; // 5th bit
}

pub fn ld_a_r(state: *Z80State, _: *const OpCode) void {
    state.PC +%= 1;
    const r = state.R.getValue();
    state.AF.A = r;
    state.AF.F.S = r & SF != 0;
    state.AF.F.Z = r == 0;
    state.AF.F.H = false;
    state.AF.F.PV = state.IFF2;
    state.AF.F.N = false;
    state.AF.F.X = r & XF != 0; // 3rd bit
    state.AF.F.Y = r & YF != 0; // 5th bit
}

pub fn ld_i_a(state: *Z80State, _: *const OpCode) void {
    state.PC +%= 1;
    state.I = state.AF.A;
}

pub fn ld_r_a(state: *Z80State, _: *const OpCode) void {
    state.PC +%= 1;
    state.R.setValue(state.AF.A);
}

pub fn ld_hl_r(state: *Z80State, opcode: *const OpCode) void {
    state.PC +%= 1;
    state.memory[state.HL.getValue()] = state.gp_registers[opcode.z];
}

pub fn ld_hl_n(state: *Z80State, _: *const OpCode) void {
    state.memory[state.HL.getValue()] = readImmediate(state, 1);
    state.PC +%= 2;
}

pub fn ld_bc_a(state: *Z80State, _: *const OpCode) void {
    state.memory[state.BC.getValue()] = state.AF.A;
    state.PC +%= 1;
}

pub fn ld_de_a(state: *Z80State, _: *const OpCode) void {
    state.memory[state.DE.getValue()] = state.AF.A;
    state.PC +%= 1;
}

pub fn ld_xy_r(state: *Z80State, opcode: *const OpCode) void {
    storeIndexed(state, readRegister(state, opcode.z), 1);
    state.PC +%= 2;
}

pub fn ld_xy_n(state: *Z80State, _: *const OpCode) void {
    const value = readImmediate(state, 2);
    storeIndexed(state, value, 1);
    state.PC +%= 3;
}

const instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    undefined, undefined, ld_bc_a, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, undefined, undefined, ld_a_bc, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, // 0
    undefined, undefined, ld_de_a, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, undefined, undefined, ld_a_de, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, // 1
    undefined, undefined, undefined, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, undefined, undefined, undefined, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, // 2
    undefined, undefined, undefined, undefined, al2_a_hl, al2_a_hl, ld_hl_n, undefined, undefined, undefined, ld_a_nn, undefined, al2_a_r, al2_a_r, ld_r_n, undefined, // 3
    ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 4
    ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 5
    ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 6
    ld_hl_r, ld_hl_r, ld_hl_r, ld_hl_r, ld_hl_r, ld_hl_r, undefined, ld_hl_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 7
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // 8
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // 9
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // A
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // B
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, // C
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, undefined, undefined, dd_prefix, al_a_n, undefined, // D
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, undefined, undefined, ed_prefix, al_a_n, undefined, // E
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, undefined, undefined, fd_prefix, al_a_n, undefined, // f
};

// Indexed addressing XY opcodes
const xy_instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 0
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 1
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 2
    undefined, undefined, undefined, undefined, al2_a_xy, al2_a_xy, ld_xy_n, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 3
    undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, // 4
    undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, // 5
    undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, // 6
    ld_xy_r, ld_xy_r, ld_xy_r, ld_xy_r, ld_xy_r, ld_xy_r, undefined, ld_xy_r, undefined, undefined, undefined, undefined, undefined, undefined, ld_r_xy, undefined, // 7
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, // 8
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, // 9
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, // A
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, // B
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // C
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // D
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // E
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, // F
};

//
const ed_instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 0
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 1
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 2
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 3
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_i_a, undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_r_a, // 4
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_a_i, undefined, undefined, undefined, undefined, undefined, undefined, undefined, ld_a_r, // 5
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 6
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 7
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 8
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 9
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // A
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // B
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // C
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // D
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // E
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // F
};

pub inline fn fetchOpcode(state: *Z80State) u8 {
    const opcode: u8 = state.memory[state.PC];
    state.R.increment();
    return opcode;
}

pub fn exec(state: *Z80State, opcode: u8) void {
    const decoded: *const OpCode = @ptrCast(&opcode);
    const insn_func = instructions_table[opcode];
    insn_func(state, decoded);
}
