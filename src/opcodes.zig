const std = @import("std");
const processor = @import("cpu.zig");
const Z80State = processor.Z80State;
const RegisterPairs = processor.RegisterPairs1;
const SixteenBitRegister = processor.SixteenBitRegister;

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
const CF = 1;

const InstructionFn = *const fn (*Z80State, *const OpCode) u8;

const AddressingMode = enum {
    register, // r
    indirect, // (HL)
    indexed, // (IX+d) or (IY+d)
    immediate, // n
    extended, // (nn)
};

const BitwiseOp = enum { AND, XOR, OR };

/// see: https://web.archive.org/web/20170121033813/http://www.cs.umd.edu:80/class/spring2003/cmsc311/Notes/Comb/overflow.html
inline fn overflow(comptime T: type, result: T, lhs: T, rhs: T) T {
    return (@as(T, (lhs ^ rhs) & (lhs ^ result)) >> (@bitSizeOf(T) - 3)) & PVF;
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

pub inline fn storeExtended(state: *Z80State, value: u8, comptime offset: u8) void {
    const low = state.memory[state.PC + offset];
    const high = state.memory[state.PC + offset + 1];
    const address: u16 = @intCast(low | @as(u16, high) << 8);
    state.memory[address] = value;
}

pub inline fn add_a_x(s: *Z80State, rhs: u8) void {
    const t = s.AF.A +% rhs; // +% = wrapping addition
    s.AF.F.N = false;
    s.AF.F.PV = overflow(u8, t, s.AF.A, ~rhs) != 0; // negated rhs for sum
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
    s.AF.F.PV = overflow(u8, t, s.AF.A, ~rhs) != 0;
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
    s.AF.F.PV = overflow(u8, t, s.AF.A, rhs) != 0;
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
    s.AF.F.PV = overflow(u8, t, s.AF.A, rhs) != 0;
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
    s.AF.F.PV = overflow(u8, t, s.AF.A, rhs) != 0;
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

pub inline fn al_a_x(comptime addressing: AddressingMode, state: *Z80State, opcode: *const OpCode) u8 {
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

    return switch (addressing) {
        inline .immediate => 7,
        inline .indirect => 7,
        inline .indexed => 19,
        else => 4,
    };
}

pub inline fn al2_a_x(comptime addressing: AddressingMode, state: *Z80State, opcode: *const OpCode) u8 {
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

    return switch (addressing) {
        inline .immediate => 7,
        inline .indirect => 11,
        inline .indexed => 23,
        else => 4,
    };
}

pub fn al2_a_r(state: *Z80State, opcode: *const OpCode) u8 {
    return al2_a_x(.register, state, opcode);
}

pub fn al2_a_hl(state: *Z80State, opcode: *const OpCode) u8 {
    return al2_a_x(.indirect, state, opcode);
}

pub fn al2_a_xy(state: *Z80State, opcode: *const OpCode) u8 {
    return al2_a_x(.indexed, state, opcode);
}

pub fn al_a_r(state: *Z80State, opcode: *const OpCode) u8 {
    return al_a_x(.register, state, opcode);
}

pub fn al_a_hl(state: *Z80State, opcode: *const OpCode) u8 {
    return al_a_x(.indirect, state, opcode);
}

pub fn al_a_xy(state: *Z80State, opcode: *const OpCode) u8 {
    return al_a_x(.indexed, state, opcode);
}

pub fn al_a_n(state: *Z80State, opcode: *const OpCode) u8 {
    return al_a_x(.immediate, state, opcode);
}

pub inline fn add_16(dst: *SixteenBitRegister, src: *SixteenBitRegister, state: *Z80State) void {
    const lhs = dst.getValue();
    const rhs = src.getValue();
    const t: u32 = @as(u32, lhs) +% rhs;

    const xy: u8 = @truncate(t >> 8);
    state.AF.F.N = false;
    state.AF.F.C = (t >> 16) & 1 != 0;
    state.AF.F.H = ((lhs ^ rhs ^ t) >> 8) & HF != 0;
    state.AF.F.X = xy & XF != 0;
    state.AF.F.Y = xy & YF != 0;

    dst.setValue(@truncate(t));
}

pub inline fn adc_hl_ss(state: *Z80State, src: *SixteenBitRegister) void {
    const lhs = state.HL.getValue();
    const rhs = src.getValue();
    const carry = @intFromBool(state.AF.F.C);
    const t = @as(u32, lhs) +% rhs +% carry;
    const t16: u16 = @truncate(t);

    const xy: u8 = @truncate(t >> 8);

    state.AF.F.C = (t >> 16) & 1 != 0;
    state.AF.F.H = ((lhs ^ rhs ^ t) >> 8) & HF != 0;
    state.AF.F.PV = overflow(u16, t16, lhs, ~rhs) != 0;
    state.AF.F.S = xy & SF != 0;
    state.AF.F.X = xy & XF != 0;
    state.AF.F.Y = xy & YF != 0;
    state.AF.F.Z = t == 0;
    state.AF.F.N = false;

    state.HL.setValue(t16);
}

pub inline fn sbc_hl_ss(state: *Z80State, src: *SixteenBitRegister) void {
    const lhs = state.HL.getValue();
    const rhs = src.getValue();
    const carry = @intFromBool(state.AF.F.C);
    const t = @as(u32, lhs) -% rhs -% carry;
    const t16: u16 = @truncate(t);

    const xy: u8 = @truncate(t >> 8);

    state.AF.F.C = (t >> 16) & 1 != 0;
    state.AF.F.H = ((lhs ^ rhs ^ t) >> 8) & HF != 0;
    state.AF.F.PV = overflow(u16, t16, lhs, rhs) != 0;
    state.AF.F.S = xy & SF != 0;
    state.AF.F.X = xy & XF != 0;
    state.AF.F.Y = xy & YF != 0;
    state.AF.F.Z = t == 0;
    state.AF.F.N = true;

    state.HL.setValue(t16);
}

// 0 0 s s q 0 1 1
// ss - target register
// q (0) = inc, (1) dec
pub inline fn inc_dec_ss(_: *Z80State, opcode: *const OpCode, ss: *SixteenBitRegister) void {
    const dec: u8 = opcode.y & 1; // (0) inc, (1) dec
    const nf: u8 = dec << 1; // 2 if dec, 1 if inc
    const lhs = ss.getValue();
    const t = lhs +% 1 -% nf;
    ss.setValue(t);
}

pub fn add_hl_bc(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(&state.HL, &state.BC, state);
    return 11;
}

pub fn add_hl_de(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(&state.HL, &state.DE, state);
    return 11;
}

pub fn add_hl_hl(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(&state.HL, &state.HL, state);
    return 11;
}

pub fn add_hl_sp(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(&state.HL, &state.SP, state);
    return 11;
}

pub fn adc_hl_bc(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    adc_hl_ss(state, &state.BC);
    return 15;
}

pub fn adc_hl_de(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    adc_hl_ss(state, &state.DE);
    return 15;
}

pub fn adc_hl_hl(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    adc_hl_ss(state, &state.HL);
    return 15;
}

pub fn adc_hl_sp(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    adc_hl_ss(state, &state.SP);
    return 15;
}

pub fn sbc_hl_bc(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    sbc_hl_ss(state, &state.BC);
    return 15;
}

pub fn sbc_hl_de(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    sbc_hl_ss(state, &state.DE);
    return 15;
}

pub fn sbc_hl_hl(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    sbc_hl_ss(state, &state.HL);
    return 15;
}

pub fn sbc_hl_sp(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    sbc_hl_ss(state, &state.SP);
    return 15;
}

pub fn add_xy_bc(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(state.addr_register, &state.BC, state);
    return 15;
}

pub fn add_xy_de(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(state.addr_register, &state.DE, state);
    return 15;
}

pub fn add_xy_ix(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(state.addr_register, state.addr_register, state);
    return 15;
}

pub fn add_xy_sp(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    add_16(state.addr_register, &state.SP, state);
    return 15;
}

pub fn inc_dec_bc(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    inc_dec_ss(state, opcode, &state.BC);
    return 6;
}

pub fn inc_dec_de(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    inc_dec_ss(state, opcode, &state.DE);
    return 6;
}

pub fn inc_dec_hl(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    inc_dec_ss(state, opcode, &state.HL);
    return 6;
}

pub fn inc_dec_sp(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    inc_dec_ss(state, opcode, &state.SP);
    return 6;
}

pub fn inc_dec_xy(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    inc_dec_ss(state, opcode, state.addr_register);
    return 10;
}

pub fn ij_prefix(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const opcode_int = fetchOpcode(state);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = xy_instructions_table[opcode_int];
    return insn_func(state, &opcode);
}

pub fn dd_prefix(state: *Z80State, opcode: *const OpCode) u8 {
    state.addr_register = &state.IX;
    return ij_prefix(state, opcode);
}

pub fn fd_prefix(state: *Z80State, opcode: *const OpCode) u8 {
    state.addr_register = &state.IY;
    return ij_prefix(state, opcode);
}

pub fn ed_prefix(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const opcode_int = fetchOpcode(state);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = ed_instructions_table[opcode_int];
    return insn_func(state, &opcode);
}

pub fn cb_prefix(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const opcode_int = fetchOpcode(state);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = cb_instructions_table[opcode_int];
    return insn_func(state, &opcode);
}

pub fn ld_r_r(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = state.gp_registers[opcode.z];
    state.PC +%= 1;
    return 4;
}

pub fn ld_r_n(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = readImmediate(state, 1);
    state.PC +%= 2;
    return 7;
}

pub fn ld_r_hl(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = readIndirect(state, @intFromEnum(RegisterPairs.HL));
    state.PC +%= 1;
    return 7;
}

pub fn ld_a_bc(state: *Z80State, _: *const OpCode) u8 {
    const dst = @intFromEnum(processor.EightBitRegisters.A);
    state.gp_registers[dst] = readIndirect(state, @intFromEnum(RegisterPairs.BC));
    state.PC +%= 1;
    return 7;
}

pub fn ld_a_de(state: *Z80State, _: *const OpCode) u8 {
    const dst = @intFromEnum(processor.EightBitRegisters.A);
    state.gp_registers[dst] = readIndirect(state, @intFromEnum(RegisterPairs.DE));
    state.PC +%= 1;
    return 7;
}

pub fn ld_r_xy(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = readIndexed(state, 1);
    state.PC +%= 2;
    return 19;
}

pub fn ld_a_nn(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = readExtended(state, 1);
    state.PC +%= 3;
    return 13;
}

pub fn ld_nn_a(state: *Z80State, _: *const OpCode) u8 {
    storeExtended(state, state.AF.A, 1);
    state.PC +%= 3;
    return 13;
}

pub fn ld_a_i(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    state.AF.A = state.I;
    state.AF.F.S = state.I & SF != 0;
    state.AF.F.Z = state.I == 0;
    state.AF.F.H = false;
    state.AF.F.PV = state.IFF2;
    state.AF.F.N = false;
    state.AF.F.X = state.I & XF != 0; // 3rd bit
    state.AF.F.Y = state.I & YF != 0; // 5th bit
    return 9;
}

pub fn ld_a_r(state: *Z80State, _: *const OpCode) u8 {
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
    return 9;
}

pub fn ld_i_a(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    state.I = state.AF.A;
    return 9;
}

pub fn ld_r_a(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    state.R.setValue(state.AF.A);
    return 9;
}

pub fn ld_hl_r(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    state.memory[state.HL.getValue()] = state.gp_registers[opcode.z];
    return 7;
}

pub fn ld_hl_n(state: *Z80State, _: *const OpCode) u8 {
    state.memory[state.HL.getValue()] = readImmediate(state, 1);
    state.PC +%= 2;
    return 10;
}

pub fn ld_bc_a(state: *Z80State, _: *const OpCode) u8 {
    state.memory[state.BC.getValue()] = state.AF.A;
    state.PC +%= 1;
    return 7;
}

pub fn ld_de_a(state: *Z80State, _: *const OpCode) u8 {
    state.memory[state.DE.getValue()] = state.AF.A;
    state.PC +%= 1;
    return 7;
}

pub fn ld_xy_r(state: *Z80State, opcode: *const OpCode) u8 {
    storeIndexed(state, readRegister(state, opcode.z), 1);
    state.PC +%= 2;
    return 19;
}

pub fn ld_xy_n(state: *Z80State, _: *const OpCode) u8 {
    const value = readImmediate(state, 2);
    storeIndexed(state, value, 1);
    state.PC +%= 3;
    return 19;
}

pub const IncDecOperation = enum {
    increment,
    decrement,
};

// see: http://www.z80.info/zip/z80-documented.pdf chapter 4.2
pub inline fn bt_ld_x(state: *Z80State, _: *const OpCode, comptime op: IncDecOperation) u8 {
    const src = @intFromEnum(RegisterPairs.HL);
    const dst = @intFromEnum(RegisterPairs.DE);

    var t = readIndirect(state, src);

    // (DE) <- (HL)
    storeIndirect(state, t, dst);

    switch (op) {
        inline .increment => {
            state.DE.increment(); // DE += 1
            state.HL.increment(); // HL += 1
        },
        inline .decrement => {
            state.DE.decrement(); // DE -= 1
            state.HL.decrement(); // HL -= 1
        },
    }

    const bc = state.BC.getValue() -% 1;
    state.BC.setValue(bc);

    t +%= state.AF.A;

    state.AF.F.H = false;
    state.AF.F.PV = bc != 0;
    state.AF.F.N = false;
    state.AF.F.X = t & XF != 0; // 3rd bit
    state.AF.F.Y = (t & 2) << 4 != 0; // 1st bit

    return 16;
}

pub inline fn bt_ldr_x(state: *Z80State, opcode: *const OpCode, comptime op: IncDecOperation) u8 {
    const cycles = bt_ld_x(state, opcode, op);

    if (state.BC.getValue() != 0) {
        const pc = state.PC >> 8;
        state.AF.F.X = pc & XF != 0;
        state.AF.F.Y = pc & YF != 0;
        state.PC -%= 1;
        return 21;
    }

    state.PC +%= 1;
    return cycles;
}

pub fn bt_ldi(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    return bt_ld_x(state, opcode, .increment);
}

pub fn bt_ldd(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    return bt_ld_x(state, opcode, .decrement);
}

pub fn bt_ldir(state: *Z80State, opcode: *const OpCode) u8 {
    return bt_ldr_x(state, opcode, .increment);
}

pub fn bt_lddr(state: *Z80State, opcode: *const OpCode) u8 {
    return bt_ldr_x(state, opcode, .decrement);
}

// see: http://www.z80.info/zip/z80-documented.pdf chapter 4.2
pub inline fn bs_cp_x(state: *Z80State, _: *const OpCode, comptime op: IncDecOperation) struct { u8, u16 } {
    const hl = @intFromEnum(RegisterPairs.HL);

    const n = readIndirect(state, hl);
    const t0 = state.AF.A -% n;
    const hf = ((state.AF.A ^ n ^ t0) & HF) >> 4;
    const t1 = t0 -% hf; // n

    switch (op) {
        inline .increment => {
            state.HL.increment(); // HL += 1
        },
        inline .decrement => {
            state.HL.decrement(); // HL -= 1
        },
    }

    const bc = state.BC.getValue() -% 1;
    state.BC.setValue(bc);

    state.AF.F.S = t0 & SF != 0;
    state.AF.F.Z = t0 == 0;
    state.AF.F.H = hf != 0;
    state.AF.F.PV = bc != 0;
    state.AF.F.N = true;
    state.AF.F.X = t1 & XF != 0; // 3rd bit
    state.AF.F.Y = (t1 & 2) << 4 != 0; // 1st bit

    return .{ t0, bc };
}

pub fn bs_cpi(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    _ = bs_cp_x(state, opcode, .increment);
    return 16;
}

pub fn bs_cpd(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    _ = bs_cp_x(state, opcode, .decrement);
    return 16;
}

pub inline fn bs_cpr_x(state: *Z80State, opcode: *const OpCode, comptime op: IncDecOperation) u8 {
    const r = bs_cp_x(state, opcode, op);
    const t0 = r[0];
    const bc = r[1];

    if (bc != 0 and t0 != 0) {
        state.PC -%= 1; // prefix
        const pc = state.PC >> 8;
        state.AF.F.X = pc & XF != 0;
        state.AF.F.Y = pc & YF != 0;
        return 21;
    }

    state.PC +%= 1;
    return 16;
}

pub fn bs_cpir(state: *Z80State, opcode: *const OpCode) u8 {
    return bs_cpr_x(state, opcode, .increment);
}

pub fn bs_cpdr(state: *Z80State, opcode: *const OpCode) u8 {
    return bs_cpr_x(state, opcode, .decrement);
}

pub fn be_ex_de_hl(state: *Z80State, _: *const OpCode) u8 {
    std.mem.swap(processor.SixteenBitRegister, &state.DE, &state.HL);
    state.PC +%= 1;
    return 4;
}

pub fn be_ex_af_af2(state: *Z80State, _: *const OpCode) u8 {
    std.mem.swap(processor.AccumulatorFlagsRegister, &state.AF, &state.AF_);
    state.PC +%= 1;
    return 4;
}

pub fn be_exx(state: *Z80State, _: *const OpCode) u8 {
    std.mem.swap(processor.SixteenBitRegister, &state.BC, &state.BC_);
    std.mem.swap(processor.SixteenBitRegister, &state.DE, &state.DE_);
    std.mem.swap(processor.SixteenBitRegister, &state.HL, &state.HL_);
    state.PC +%= 1;
    return 4;
}

pub fn be_ex_sp_hl(state: *Z80State, _: *const OpCode) u8 {
    const sp = state.SP.getValue();
    std.mem.swap(u8, &state.memory[sp], &state.HL.low);
    std.mem.swap(u8, &state.memory[sp +% 1], &state.HL.high);
    state.PC +%= 1;
    return 19;
}

pub fn be_ex_sp_xy(state: *Z80State, _: *const OpCode) u8 {
    const sp = state.SP.getValue();
    std.mem.swap(u8, &state.memory[sp], &state.addr_register.low);
    std.mem.swap(u8, &state.memory[sp +% 1], &state.addr_register.high);
    state.PC +%= 1;
    return 23;
}

pub fn rot_r(state: *Z80State, opcode: *const OpCode) u8 {
    const value = state.gp_registers[opcode.z];
    var res: u8 = 0;
    var cf: u8 = 0;
    var cycles: u8 = 0;

    switch (opcode.y) {
        0 => { // RLC
            res = std.math.rotl(u8, value, 1);
            cf = res & CF;
            cycles = 8;
        },
        1 => { // RRC
            cf = value & CF;
            res = std.math.rotr(u8, value, 1);
            cycles = 8;
        },
        2 => { // RL
            cf = value & 128;
            const f: u8 = state.AF.getFlags() & CF;
            res = (value << 1) | f;
            cycles = 8;
        },
        3 => { // RR
            cf = value & CF;
            const f: u8 = state.AF.getFlags() & CF;
            res = (value >> 1) | (f << 7);
            cycles = 8;
        },
        4 => { // SLA
            cf = value >> 7;
            res = value << 1;
            cycles = 8;
        },
        5 => { // SRA
            cf = value & CF;
            res = (value & 128) | (value >> 1);
            cycles = 8;
        },
        6 => { // SLL
            cf = value >> 7;
            res = (value << 1) | CF;
            cycles = 8;
        },
        7 => { // SRL
            cf = value & CF;
            res = value >> 1;
            cycles = 8;
        },
    }

    state.gp_registers[opcode.z] = res;

    state.AF.F.N = false;
    state.AF.F.H = false;
    state.AF.F.Z = res == 0;
    state.AF.F.PV = hasEvenParity8(res);
    state.AF.F.C = cf != 0;
    state.AF.F.S = res & SF != 0;
    state.AF.F.X = res & XF != 0;
    state.AF.F.Y = res & YF != 0;

    state.PC +%= 1;
    return cycles;
}

pub fn rlca(state: *Z80State, _: *const OpCode) u8 {
    const value = std.math.rotl(u8, state.AF.A, 1);
    state.AF.A = value;
    state.AF.F.C = value & CF != 0;
    state.AF.F.X = value & XF != 0;
    state.AF.F.Y = value & YF != 0;
    state.AF.F.H = false;
    state.AF.F.N = false;
    state.PC +%= 1;
    return 4;
}

pub fn rrca(state: *Z80State, _: *const OpCode) u8 {
    const value = std.math.rotr(u8, state.AF.A, 1);
    state.AF.A = value;
    state.AF.F.C = value & 128 != 0; // 7th bit of A
    state.AF.F.X = value & XF != 0;
    state.AF.F.Y = value & YF != 0;
    state.AF.F.H = false;
    state.AF.F.N = false;
    state.PC +%= 1;
    return 4;
}

pub fn rla(state: *Z80State, _: *const OpCode) u8 {
    const cf0: u8 = state.AF.getFlags() & CF;
    const cf1: u8 = state.AF.A & 128;
    const value = (state.AF.A << 1) | cf0;
    state.AF.A = value;
    state.AF.F.C = cf1 != 0; // 7th bit of A
    state.AF.F.X = value & XF != 0;
    state.AF.F.Y = value & YF != 0;
    state.AF.F.H = false;
    state.AF.F.N = false;
    state.PC +%= 1;
    return 4;
}

pub fn rra(state: *Z80State, _: *const OpCode) u8 {
    const cf0: u8 = state.AF.getFlags() & CF;
    const cf1: u8 = state.AF.A & CF;
    const value = (state.AF.A >> 1) | (cf0 << 7);
    state.AF.A = value;
    state.AF.F.C = cf1 != 0; // 7th bit of A
    state.AF.F.X = value & XF != 0;
    state.AF.F.Y = value & YF != 0;
    state.AF.F.H = false;
    state.AF.F.N = false;
    state.PC +%= 1;
    return 4;
}

const instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    undefined, undefined, ld_bc_a, inc_dec_bc, al2_a_r, al2_a_r, ld_r_n, rlca, be_ex_af_af2, add_hl_bc, ld_a_bc, inc_dec_bc, al2_a_r, al2_a_r, ld_r_n, rrca, // 0
    undefined, undefined, ld_de_a, inc_dec_de, al2_a_r, al2_a_r, ld_r_n, rla, undefined, add_hl_de, ld_a_de, inc_dec_de, al2_a_r, al2_a_r, ld_r_n, rra, // 1
    undefined, undefined, undefined, inc_dec_hl, al2_a_r, al2_a_r, ld_r_n, undefined, undefined, add_hl_hl, undefined, inc_dec_hl, al2_a_r, al2_a_r, ld_r_n, undefined, // 2
    undefined, undefined, ld_nn_a, inc_dec_sp, al2_a_hl, al2_a_hl, ld_hl_n, undefined, undefined, add_hl_sp, ld_a_nn, inc_dec_sp, al2_a_r, al2_a_r, ld_r_n, undefined, // 3
    ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 4
    ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 5
    ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 6
    ld_hl_r, ld_hl_r, ld_hl_r, ld_hl_r, ld_hl_r, ld_hl_r, undefined, ld_hl_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_r, ld_r_hl, ld_r_r, // 7
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // 8
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // 9
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // A
    al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_r, al_a_hl, al_a_r, // B
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, cb_prefix, undefined, undefined, al_a_n, undefined, // C
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, be_exx, undefined, undefined, undefined, dd_prefix, al_a_n, undefined, // D
    undefined, undefined, undefined, be_ex_sp_hl, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, be_ex_de_hl, undefined, ed_prefix, al_a_n, undefined, // E
    undefined, undefined, undefined, undefined, undefined, undefined, al_a_n, undefined, undefined, undefined, undefined, undefined, undefined, fd_prefix, al_a_n, undefined, // F
};

// Indexed addressing XY opcodes
const xy_instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, add_xy_bc, undefined, undefined, undefined, undefined, undefined, undefined, // 0
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, add_xy_de, undefined, undefined, undefined, undefined, undefined, undefined, // 1
    undefined, undefined, undefined, inc_dec_xy, undefined, undefined, undefined, undefined, undefined, add_xy_ix, undefined, inc_dec_xy, undefined, undefined, undefined, undefined, // 2
    undefined, undefined, undefined, undefined, al2_a_xy, al2_a_xy, ld_xy_n, undefined, undefined, add_xy_sp, undefined, undefined, undefined, undefined, undefined, undefined, // 3
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
    undefined, undefined, undefined, be_ex_sp_xy, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // E
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, al_a_xy, undefined, // F
};

//
const ed_instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 0
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 1
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 2
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 3
    undefined, undefined, sbc_hl_bc, undefined, undefined, undefined, undefined, ld_i_a, undefined, undefined, adc_hl_bc, undefined, undefined, undefined, undefined, ld_r_a, // 4
    undefined, undefined, sbc_hl_de, undefined, undefined, undefined, undefined, ld_a_i, undefined, undefined, adc_hl_de, undefined, undefined, undefined, undefined, ld_a_r, // 5
    undefined, undefined, sbc_hl_hl, undefined, undefined, undefined, undefined, undefined, undefined, undefined, adc_hl_hl, undefined, undefined, undefined, undefined, undefined, // 6
    undefined, undefined, sbc_hl_sp, undefined, undefined, undefined, undefined, undefined, undefined, undefined, adc_hl_sp, undefined, undefined, undefined, undefined, undefined, // 7
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 8
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 9
    bt_ldi, bs_cpi, undefined, undefined, undefined, undefined, undefined, undefined, bt_ldd, bs_cpd, undefined, undefined, undefined, undefined, undefined, undefined, // A
    bt_ldir, bs_cpir, undefined, undefined, undefined, undefined, undefined, undefined, bt_lddr, bs_cpdr, undefined, undefined, undefined, undefined, undefined, undefined, // B
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // C
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // D
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // E
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // F
};

const cb_instructions_table = [256]InstructionFn{
    //      0          1          2          3          4          5          6          7          8          9          A          B          C          D          E          F
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, // 0
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, // 1
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, // 2
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, undefined, rot_r, // 3
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 4
    undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, // 5
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
    const insn_cycles = insn_func(state, decoded);
    state.cycles +%= insn_cycles;
}
