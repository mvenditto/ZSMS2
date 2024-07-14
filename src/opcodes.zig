const std = @import("std");
const processor = @import("cpu.zig");
const byte = @import("byte.zig");
const Z80State = processor.Z80State;
const LineLow = processor.LineLow;
const LineHigh = processor.LineHigh;
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
const ZF = 64;
const YF = 32;
const HF = 16;
const XF = 8;
const PVF = 4;
const NF = 2;
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
/// 0 = even, 1 = odd
inline fn parity(x: u8) u1 {
    const p = @as(u16, 0x6996) >> @intCast((x ^ (x >> 4)) & 0xF);
    return @truncate(p & 1);
}

fn hasEvenParity8(x: u8) bool {
    return parity(x) == 0; // 0 = even
}

fn z80Parity(x: u8) u1 {
    return parity(x) ^ 1; // 1 == even, 0 == odd
}

pub inline fn readRegister(state: *Z80State, r: u8) u8 {
    const value = state.gp_registers[r];
    return value;
}

pub inline fn readImmediate(state: *Z80State, comptime offset: u8) u8 {
    const value = state.memRead(state.PC +% offset);
    return value;
}

pub inline fn readExtended(state: *Z80State, comptime offset: u8) u8 {
    const low = state.memRead(state.PC + offset);
    const high = state.memRead(state.PC + offset + 1);
    const address: u16 = @intCast(low | @as(u16, high) << 8);
    const value = state.memRead(address);
    return value;
}

pub inline fn readImmediateExtended(state: *const Z80State, comptime offset: u8) .{ u8, u8 } {
    const low = state.memRead(state.PC +% offset);
    const high = state.memRead(state.PC +% offset +% 1);
    return .{ low, high };
}

pub inline fn readExtendedUpdateWZ(state: *Z80State, comptime offset: u8) u8 {
    const low = state.memRead(state.PC + offset);
    const high = state.memRead(state.PC + offset + 1);
    const address: u16 = @intCast(low | @as(u16, high) << 8);
    const value = state.memRead(address);
    state.WZ.setValue(address +% 1);
    return value;
}

pub inline fn readIndirect(state: *Z80State, rp: u8) u8 {
    const address = state.gp_registers_pairs[rp].getValue();
    const value = state.memRead(address);
    return value;
}

pub fn readIndexed(state: *Z80State, comptime offset: u8) u8 {
    const base_address: i32 = @intCast(state.addr_register.getValue());
    const d: i8 = @bitCast(state.memRead(state.PC + offset)); // displacement is two's complement
    const address: u16 = @intCast(@mod(base_address + d, 65536));
    const value = state.memRead(address);
    state.WZ.setValue(address);
    return value;
}

pub inline fn storeRegister(state: *Z80State, r: u8, value: u8) void {
    state.gp_registers[r] = value;
}

pub inline fn storeIndirect(state: *Z80State, value: u8, rp: u8) void {
    const address = state.gp_registers_pairs[rp].getValue();
    state.memWrite(address, value);
}

pub inline fn storeIndexed(state: *Z80State, value: u8, comptime offset: u8) void {
    const base_address: i32 = @intCast(state.addr_register.getValue());
    const d: i8 = @bitCast(state.memRead(state.PC + offset)); // displacement is two's complement
    const address: u16 = @intCast(@mod(base_address + d, 65536));
    state.memWrite(address, value);
}

pub inline fn storeIndexedUpdateWZ(state: *Z80State, value: u8, comptime offset: u8) void {
    const base_address: i32 = @intCast(state.addr_register.getValue());
    const d: i8 = @bitCast(state.memRead(state.PC + offset)); // displacement is two's complement
    const address: u16 = @intCast(@mod(base_address + d, 65536));
    state.memWrite(address, value);
    state.WZ.setValue(address);
}

pub inline fn storeExtended(state: *Z80State, value: u8, comptime offset: u8) void {
    const low = state.memRead(state.PC + offset);
    const high = state.memRead(state.PC + offset + 1);
    const address: u16 = @intCast(low | @as(u16, high) << 8);
    state.memWrite(address, value);
}

pub inline fn storeExtendedUpdateWZ(state: *Z80State, value: u8, comptime offset: u8) void {
    const low = state.memRead(state.PC + offset);
    const high = state.memRead(state.PC + offset + 1);
    const address: u16 = @intCast(low | @as(u16, high) << 8);
    state.memWrite(address, value);
    state.WZ.low = @truncate((address +% 1) & 0xFF);
    state.WZ.high = state.AF.A;
}

pub inline fn loadImmediateExtended(state: *const Z80State, dst: *SixteenBitRegister, comptime offset: u8) void {
    dst.low = state.memRead(state.PC +% offset);
    dst.high = state.memRead(state.PC +% offset +% 1);
}

pub fn nop(s: *Z80State, _: *const OpCode) u8 {
    s.PC +%= 1;
    return 4;
}

pub fn nop_nop(s: *Z80State, _: *const OpCode) u8 {
    s.PC +%= 1;
    return 8;
}

// acts as a NOP NOP
pub fn ed_illegal(s: *Z80State, _: *const OpCode) u8 {
    // std.debug.print(
    //     "Illegal opcode ED{x}: nop nop.\n",
    //     .{@as(u8, @bitCast(opcode.*))},
    // );
    s.PC +%= 1;
    return 8;
}

// the prefix (DD or FD) is ignored and the rest of the
// opcode executed as if it was "unprefixed".
pub fn xy_illegal(s: *Z80State, _: *const OpCode) u8 {
    const next_opcode = s.memRead(s.PC);
    const next: OpCode = @bitCast(next_opcode);
    return instructions_table[next_opcode](s, &next) + 4; // 4 t-states for the prefix
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

pub fn al_a_p(state: *Z80State, opcode: *const OpCode) u8 {
    const registers: *[8]*u8 = if (state.memRead(state.PC -% 1) == 0xdd) state.p else state.q;
    const rhs = registers[opcode.z].*;

    state.PC +%= 1;

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

    return 8;
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

pub fn al2_a_p(state: *Z80State, opcode: *const OpCode) u8 {
    const value = state.pq[opcode.y].*;
    const result = inc_dec_x(state, opcode, value);
    state.pq[opcode.y].* = result;
    state.PC +%= 1;
    return 8;
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

    state.WZ.setValue(dst.getValue() +% 1);
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

    state.WZ.setValue(state.HL.getValue() +% 1);
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

    state.WZ.setValue(state.HL.getValue() +% 1);
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

pub fn xy_prefix(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const opcode_int = fetchOpcode(state);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = xy_instructions_table[opcode_int];
    return insn_func(state, &opcode);
}

pub fn dd_prefix(state: *Z80State, opcode: *const OpCode) u8 {
    state.addr_register = &state.IX;
    state.pq = state.p;
    return xy_prefix(state, opcode);
}

pub fn fd_prefix(state: *Z80State, opcode: *const OpCode) u8 {
    state.addr_register = &state.IY;
    state.pq = state.q;
    return xy_prefix(state, opcode);
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

pub fn xy_cb_prefix(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const opcode_int = state.memRead(state.PC +% 1);
    const opcode: OpCode = @bitCast(opcode_int);
    const insn_func = xy_cb_instructions_table[opcode_int];
    return insn_func(state, &opcode);
}

pub fn consumeXYSequence(state: *Z80State) u8 {
    // Consume the sequence of FD and DD prefixes.
    var prefix = state.memRead(state.PC);
    var curr = fetchOpcode(state);

    while (curr & 0xdd == 0xdd) { // 0xdd or 0xfd
        prefix = curr; // update last seen prefix and go on
        state.PC +%= 1;
        state.cycles +%= 4;
        curr = fetchOpcode(state);
    }

    // decrement PC so that the full opcode 0x(dd|fd)hh is executed next
    state.PC -%= 1;

    std.debug.assert(state.memRead(state.PC) & 0xdd == 0xdd);
    std.debug.assert(state.memRead(state.PC +% 1) & 0xdd != 0xdd);

    // The last seen DD or FD prefix is the one that matters.
    return prefix;
}

pub fn xy_xy_seq(state: *Z80State, _: *const OpCode) u8 {
    const final_prefix = @call(.always_inline, consumeXYSequence, .{state});

    const opcode: OpCode = @bitCast(final_prefix);

    if (final_prefix == 0xfd) {
        return fd_prefix(state, &opcode);
    } else {
        return dd_prefix(state, &opcode);
    }
}

pub fn ld_r_r(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = state.gp_registers[opcode.z];
    state.PC +%= 1;
    return 4;
}

pub fn ld_u_r_r(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = state.gp_registers[opcode.z];
    state.PC +%= 1;
    return 8;
}

pub fn ld_p_xyh(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = state.addr_register.high;
    state.PC +%= 1;
    return 8;
}

pub fn ld_xyh_p(state: *Z80State, opcode: *const OpCode) u8 {
    state.addr_register.high = state.gp_registers[opcode.z];
    state.PC +%= 1;
    return 8;
}

pub fn ld_p_xyl(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = state.addr_register.low;
    state.PC +%= 1;
    return 8;
}

pub fn ld_xyl_p(state: *Z80State, opcode: *const OpCode) u8 {
    state.addr_register.low = state.gp_registers[opcode.z];
    state.PC +%= 1;
    return 8;
}

pub fn ld_p_xyh_xyl(state: *Z80State, _: *const OpCode) u8 {
    state.addr_register.high = state.addr_register.low;
    state.PC +%= 1;
    return 8;
}

pub fn ld_p_xyl_xyh(state: *Z80State, _: *const OpCode) u8 {
    state.addr_register.low = state.addr_register.high;
    state.PC +%= 1;
    return 8;
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
    state.WZ.setValue(state.BC.getValue() +% 1);
    state.PC +%= 1;
    return 7;
}

pub fn ld_a_de(state: *Z80State, _: *const OpCode) u8 {
    const dst = @intFromEnum(processor.EightBitRegisters.A);
    state.gp_registers[dst] = readIndirect(state, @intFromEnum(RegisterPairs.DE));
    state.WZ.setValue(state.DE.getValue() +% 1);
    state.PC +%= 1;
    return 7;
}

pub fn ld_r_xy(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = readIndexed(state, 1);
    state.PC +%= 2;
    return 19;
}

pub fn ld_a_nn(state: *Z80State, opcode: *const OpCode) u8 {
    state.gp_registers[opcode.y] = readExtendedUpdateWZ(state, 1);
    state.PC +%= 3;
    return 13;
}

pub fn ld_nn_a(state: *Z80State, _: *const OpCode) u8 {
    storeExtendedUpdateWZ(state, state.AF.A, 1);
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
    state.memWrite(state.HL.getValue(), state.gp_registers[opcode.z]);
    return 7;
}

pub fn ld_hl_n(state: *Z80State, _: *const OpCode) u8 {
    state.memWrite(state.HL.getValue(), readImmediate(state, 1));
    state.PC +%= 2;
    return 10;
}

pub fn ld_bc_a(state: *Z80State, _: *const OpCode) u8 {
    const rp = state.BC.getValue();
    state.memWrite(rp, state.AF.A);
    state.PC +%= 1;
    state.WZ.low = @truncate((rp +% 1) & 0xFF);
    state.WZ.high = state.AF.A;
    return 7;
}

pub fn ld_de_a(state: *Z80State, _: *const OpCode) u8 {
    const rp = state.DE.getValue();
    state.memWrite(rp, state.AF.A);
    state.PC +%= 1;
    state.WZ.low = @truncate((rp +% 1) & 0xFF);
    state.WZ.high = state.AF.A;
    return 7;
}

pub fn ld_xy_r(state: *Z80State, opcode: *const OpCode) u8 {
    storeIndexedUpdateWZ(state, readRegister(state, opcode.z), 1);
    state.PC +%= 2;
    return 19;
}

pub fn ld_xy_n(state: *Z80State, _: *const OpCode) u8 {
    const value = readImmediate(state, 2);
    storeIndexedUpdateWZ(state, value, 1);
    state.PC +%= 3;
    return 19;
}

pub fn ld_dd_nn(state: *Z80State, opcode: *const OpCode) u8 {
    const dd = opcode.y >> 1; // y = 0bdd0
    loadImmediateExtended(state, state.d[dd], 1);
    state.PC +%= 3;
    return 10;
}

pub fn ld_xy_nn(state: *Z80State, _: *const OpCode) u8 {
    loadImmediateExtended(state, state.addr_register, 1);
    state.PC +%= 3;
    return 14;
}

pub fn ld_xy_p_n(state: *Z80State, opcode: *const OpCode) u8 {
    state.pq[opcode.y].* = readImmediate(state, 1);
    state.PC +%= 2;
    return 11;
}

pub fn ld_dd_nn_ext(state: *Z80State, opcode: *const OpCode) u8 {
    const rp = opcode.y >> 1; // y = 0bdd0
    const dd = state.d[rp];
    const addr_low = state.memRead(state.PC +% 1);
    const addr_high = state.memRead(state.PC +% 1 +% 1);
    const address: u16 = @intCast(addr_low | @as(u16, addr_high) << 8);
    dd.low = state.memRead(address);
    const addr_next = address +% 1;
    dd.high = state.memRead(addr_next);
    state.WZ.setValue(addr_next);
    state.PC +%= 3;
    return 20;
}

pub fn ld_hl_nn_ext(state: *Z80State, opcode: *const OpCode) u8 {
    const rp = opcode.y >> 1; // y = 0bdd0
    const dd = state.d[rp];
    const addr_low = state.memRead(state.PC +% 1);
    const addr_high = state.memRead(state.PC +% 1 +% 1);
    const address: u16 = @intCast(addr_low | @as(u16, addr_high) << 8);
    dd.low = state.memRead(address);
    const addr_next = address +% 1;
    dd.high = state.memRead(addr_next);
    state.WZ.setValue(addr_next);
    state.PC +%= 3;
    return 16;
}

pub fn ld_xy_nn_ext(state: *Z80State, _: *const OpCode) u8 {
    const dd = state.addr_register;
    const addr_low = state.memRead(state.PC +% 1);
    const addr_high = state.memRead(state.PC +% 1 +% 1);
    const address: u16 = @intCast(addr_low | @as(u16, addr_high) << 8);
    dd.low = state.memRead(address);
    const addr_next = address +% 1;
    dd.high = state.memRead(addr_next);
    state.WZ.setValue(addr_next);
    state.PC +%= 3;
    return 20;
}

pub fn ld_nn_hl_ext(state: *Z80State, _: *const OpCode) u8 {
    const addr_low = state.memRead(state.PC +% 1);
    const addr_high = state.memRead(state.PC +% 1 +% 1);
    const address: u16 = @intCast(addr_low | @as(u16, addr_high) << 8);
    const address_next = address +% 1;
    state.memWrite(address, state.HL.low);
    state.memWrite(address +% 1, state.HL.high);
    state.WZ.setValue(address_next);
    state.PC +%= 3;
    return 16;
}

pub fn ld_nn_dd_ext(state: *Z80State, opcode: *const OpCode) u8 {
    const rp = opcode.y >> 1; // y = 0bdd0
    const dd = state.d[rp];
    const addr_low = state.memRead(state.PC +% 1);
    const addr_high = state.memRead(state.PC +% 1 +% 1);
    const address: u16 = @intCast(addr_low | @as(u16, addr_high) << 8);
    const address_next = address +% 1;
    state.memWrite(address, dd.low);
    state.memWrite(address +% 1, dd.high);
    state.WZ.setValue(address_next);
    state.PC +%= 3;
    return 20;
}

pub fn ld_nn_xy_ext(state: *Z80State, _: *const OpCode) u8 {
    const dd = state.addr_register;
    const addr_low = state.memRead(state.PC +% 1);
    const addr_high = state.memRead(state.PC +% 1 +% 1);
    const address: u16 = @intCast(addr_low | @as(u16, addr_high) << 8);
    const address_next = address +% 1;
    state.memWrite(address, dd.low);
    state.memWrite(address +% 1, dd.high);
    state.WZ.setValue(address_next);
    state.PC +%= 3;
    return 20;
}

pub fn ld_sp_hl(state: *Z80State, _: *const OpCode) u8 {
    state.SP.high = state.HL.high;
    state.SP.low = state.HL.low;
    state.PC +%= 1;
    return 6;
}

pub fn ld_sp_xy(state: *Z80State, _: *const OpCode) u8 {
    state.SP.high = state.addr_register.high;
    state.SP.low = state.addr_register.low;
    state.PC +%= 1;
    return 10;
}

pub fn push_af(state: *Z80State, _: *const OpCode) u8 {
    state.SP.decrement();
    state.memWrite(state.SP.getValue(), state.AF.A);
    state.SP.decrement();
    state.memWrite(state.SP.getValue(), @bitCast(state.AF.F));
    state.PC +%= 1;
    return 11;
}

pub fn push_qq(state: *Z80State, opcode: *const OpCode) u8 {
    const qq = opcode.y >> 1;
    const rp = &state.gp_registers_pairs[qq];
    var sp = state.SP.getValue();
    sp -%= 1;
    state.memWrite(sp, rp.high);
    sp -%= 1;
    state.memWrite(sp, rp.low);
    state.SP.setValue(sp);
    state.PC +%= 1;
    return 11;
}

pub fn push_xy(state: *Z80State, _: *const OpCode) u8 {
    var sp = state.SP.getValue();
    sp -%= 1;
    state.memWrite(sp, state.addr_register.high);
    sp -%= 1;
    state.memWrite(sp, state.addr_register.low);
    state.SP.setValue(sp);
    state.PC +%= 1;
    return 15;
}

pub fn pop_af(state: *Z80State, _: *const OpCode) u8 {
    var sp = state.SP.getValue();
    state.AF.setFlags(state.memRead(sp));
    sp +%= 1;
    state.AF.A = state.memRead(sp);
    sp +%= 1;
    state.SP.setValue(sp);
    state.PC +%= 1;
    return 10;
}

pub fn pop_qq(state: *Z80State, opcode: *const OpCode) u8 {
    const qq = opcode.y >> 1;
    var rp = &state.gp_registers_pairs[qq];
    var sp = state.SP.getValue();
    rp.low = state.memRead(sp);
    sp +%= 1;
    rp.high = state.memRead(sp);
    sp +%= 1;
    state.SP.setValue(sp);
    state.PC +%= 1;
    return 10;
}

pub fn pop_xy(state: *Z80State, _: *const OpCode) u8 {
    var sp = state.SP.getValue();
    state.addr_register.low = state.memRead(sp);
    sp +%= 1;
    state.addr_register.high = state.memRead(sp);
    sp +%= 1;
    state.SP.setValue(sp);
    state.PC +%= 1;
    return 14;
}

pub fn call_nn(state: *Z80State, _: *const OpCode) u8 {
    var sp = state.SP.getValue();
    const nn_low = state.memRead(state.PC +% 1);
    const nn_high = state.memRead(state.PC +% 1 +% 1);
    const nn: u16 = @intCast(nn_low | @as(u16, nn_high) << 8);
    sp -%= 1;
    state.PC +%= 3;
    state.memWrite(sp, @truncate((state.PC >> 8) & 0xFF));
    sp -%= 1;
    state.memWrite(sp, @truncate(state.PC & 0xFF));
    state.SP.setValue(sp);
    state.PC = nn;
    state.WZ.low = nn_low;
    state.WZ.high = nn_high;
    return 17;
}

pub fn call_cc_nn(state: *Z80State, opcode: *const OpCode) u8 {
    const nn_low = state.memRead(state.PC +% 1);
    const nn_high = state.memRead(state.PC +% 1 +% 1);
    state.PC +%= 3;

    if (evalFlagsCondition(state, opcode.y)) // y=ccc
    {
        var sp = state.SP.getValue();
        const nn: u16 = @intCast(nn_low | @as(u16, nn_high) << 8);
        sp -%= 1;
        state.memWrite(sp, @truncate((state.PC >> 8) & 0xFF));
        sp -%= 1;
        state.memWrite(sp, @truncate(state.PC & 0xFF));
        state.SP.setValue(sp);
        state.PC = nn;
        state.WZ.low = nn_low;
        state.WZ.high = nn_high;
        return 17;
    }

    state.WZ.low = nn_low;
    state.WZ.high = nn_high;

    return 10;
}

pub inline fn ret_(state: *Z80State) u8 {
    var sp = state.SP.getValue();
    const b_low = state.memRead(sp);
    sp +%= 1;
    const b_high = state.memRead(sp);
    sp +%= 1;
    state.SP.setValue(sp);
    state.PC = @intCast(b_low | @as(u16, b_high) << 8);
    state.WZ.low = b_low;
    state.WZ.high = b_high;
    return 10;
}

pub fn ret(state: *Z80State, _: *const OpCode) u8 {
    return ret_(state);
}

pub fn ret_cc(state: *Z80State, opcode: *const OpCode) u8 {
    if (evalFlagsCondition(state, opcode.y)) // y=ccc
    {
        var sp = state.SP.getValue();
        const b_low = state.memRead(sp);
        sp +%= 1;
        const b_high = state.memRead(sp);
        sp +%= 1;
        state.SP.setValue(sp);
        state.PC = @intCast(b_low | @as(u16, b_high) << 8);
        state.WZ.low = b_low;
        state.WZ.high = b_high;
        return 11;
    }

    state.PC +%= 1;

    return 5;
}

pub fn rst_p(state: *Z80State, opcode: *const OpCode) u8 {
    var sp = state.SP.getValue();
    const pc = state.PC +% 1;
    sp -%= 1;
    state.memWrite(sp, @truncate((pc >> 8) & 0xFF));
    sp -%= 1;
    state.memWrite(sp, @truncate(pc & 0xFF));
    state.SP.setValue(sp);
    const p = @as(u8, @intCast(opcode.y)) * 8;
    state.PC = p;
    state.WZ.high = 0;
    state.WZ.low = p;
    return 11;
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

    const bc = state.BC.getValue();

    if (bc != 0) {
        if (bc != 1) {
            state.WZ.setValue(state.PC);
        }
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
    state.WZ.increment();
    return 16;
}

pub fn bs_cpd(state: *Z80State, opcode: *const OpCode) u8 {
    state.PC +%= 1;
    _ = bs_cp_x(state, opcode, .decrement);
    state.WZ.decrement();
    return 16;
}

pub inline fn bs_cpr_x(state: *Z80State, opcode: *const OpCode, comptime op: IncDecOperation) u8 {
    const r = bs_cp_x(state, opcode, op);
    const t0 = r[0];
    const bc = r[1];

    if (bc == 1 or t0 == 0) {
        switch (op) {
            inline .increment => {
                state.WZ.increment();
            },
            inline .decrement => {
                state.WZ.decrement();
            },
        }
    } else {
        state.WZ.setValue(state.PC);
    }

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
    var sp = state.SP.getValue();

    const i = state.memRead(sp);
    state.memWrite(sp, state.HL.low);
    state.HL.low = i;

    sp +%= 1;

    const j = state.memRead(sp);
    state.memWrite(sp, state.HL.high);
    state.HL.high = j;

    state.PC +%= 1;
    state.WZ.setValue(state.HL.getValue());
    return 19;
}

pub fn be_ex_sp_xy(state: *Z80State, _: *const OpCode) u8 {
    var sp = state.SP.getValue();

    const i = state.memRead(sp);
    state.memWrite(sp, state.addr_register.low);
    state.addr_register.low = i;

    sp +%= 1;

    const j = state.memRead(sp);
    state.memWrite(sp, state.addr_register.high);
    state.addr_register.high = j;

    state.WZ.setValue(state.addr_register.getValue());
    state.PC +%= 1;
    return 23;
}

pub inline fn rot_x(state: *Z80State, opcode: *const OpCode, value: u8) u8 {
    var cf: u8 = 0;
    var res: u8 = 0;

    switch (opcode.y) {
        0 => { // RLC
            res = std.math.rotl(u8, value, 1);
            cf = res & CF;
        },
        1 => { // RRC
            cf = value & CF;
            res = std.math.rotr(u8, value, 1);
        },
        2 => { // RL
            cf = value & 128;
            const f: u8 = state.AF.getFlags() & CF;
            res = (value << 1) | f;
        },
        3 => { // RR
            cf = value & CF;
            const f: u8 = state.AF.getFlags() & CF;
            res = (value >> 1) | (f << 7);
        },
        4 => { // SLA
            cf = value >> 7;
            res = value << 1;
        },
        5 => { // SRA
            cf = value & CF;
            res = (value & 128) | (value >> 1);
        },
        6 => { // SLL
            cf = value >> 7;
            res = (value << 1) | CF;
        },
        7 => { // SRL
            cf = value & CF;
            res = value >> 1;
        },
    }

    state.AF.F.N = false;
    state.AF.F.H = false;
    state.AF.F.Z = res == 0;
    state.AF.F.PV = hasEvenParity8(res);
    state.AF.F.C = cf != 0;
    state.AF.F.S = res & SF != 0;
    state.AF.F.X = res & XF != 0;
    state.AF.F.Y = res & YF != 0;

    return res;
}

pub fn rot_r(state: *Z80State, opcode: *const OpCode) u8 {
    const value = state.gp_registers[opcode.z];
    const res = rot_x(state, opcode, value);
    state.gp_registers[opcode.z] = res;
    state.PC +%= 1;
    return 8;
}

pub fn rot_hl(state: *Z80State, opcode: *const OpCode) u8 {
    const hl = @intFromEnum(RegisterPairs.HL);
    const value = readIndirect(state, hl);
    const res = rot_x(state, opcode, value);
    storeIndirect(state, res, hl);
    state.PC +%= 1;
    return 15;
}

// see: "z80-undocumented" 3.5 DDCB Prefix.
pub fn rot_xy(state: *Z80State, opcode: *const OpCode) u8 {
    const value = readIndexed(state, 0);
    const res = rot_x(state, opcode, value);
    storeIndexed(state, res, 0);
    // undocumented (DD|FD)CB instructions that also store the result in
    // an additional register r (B,C,D,E,H,L or A) specified in the last 3 bits of the opcode.
    // opcode.z == 6 is for the documented instructions.
    // For the undocumented ones: [d] [opcode] where opcode.z != 6.
    // opcode.z:
    //  000 B
    //  001 C
    //  010 D
    //  011 E
    //  100 H
    //  101 L
    //  110 (none: documented opcode)
    //  111
    if (opcode.z != 6) {
        state.gp_registers[opcode.z] = res;
    }
    state.PC +%= 2;
    return 23;
}

pub fn rld(state: *Z80State, _: *const OpCode) u8 {
    const addr = state.HL.getValue();
    const value = state.memRead(addr);
    const lo_nib = byte.getLowNibble(value);
    const hi_nib = byte.getHighNibble(value);
    const a_lo_nib = byte.getLowNibble(state.AF.A);
    const new_a = byte.nibblesToByte(hi_nib, byte.getHighNibble(state.AF.A));
    state.AF.A = new_a;
    const new_val = byte.nibblesToByte(a_lo_nib, lo_nib);
    state.memWrite(addr, new_val);
    state.AF.F.S = new_a & SF != 0;
    state.AF.F.Z = new_a == 0;
    state.AF.F.H = false;
    state.AF.F.PV = hasEvenParity8(new_a);
    state.AF.F.N = false;
    state.AF.F.X = new_a & XF != 0;
    state.AF.F.Y = new_a & YF != 0;
    state.PC +%= 1;
    state.WZ.setValue(addr +% 1);
    return 18;
}

pub fn rrd(state: *Z80State, _: *const OpCode) u8 {
    const addr = state.HL.getValue();
    const value = state.memRead(addr);
    const lo_nib = byte.getLowNibble(value);
    const hi_nib = byte.getHighNibble(value);
    const a_lo_nib = byte.getLowNibble(state.AF.A);
    const new_a = byte.nibblesToByte(lo_nib, byte.getHighNibble(state.AF.A));
    state.AF.A = new_a;
    const new_val = byte.nibblesToByte(hi_nib, a_lo_nib);
    state.memWrite(addr, new_val);
    state.AF.F.S = new_a & SF != 0;
    state.AF.F.Z = new_a == 0;
    state.AF.F.H = false;
    state.AF.F.PV = hasEvenParity8(new_a);
    state.AF.F.N = false;
    state.AF.F.X = new_a & XF != 0;
    state.AF.F.Y = new_a & YF != 0;
    state.PC +%= 1;
    state.WZ.setValue(addr +% 1);
    return 18;
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

// In section 4.1 of "The Undocumented Z80 Documented" it is stated
// that for BIT n,r XF and YF are set based on the tested bit.
// Based on test suite SingleStepTests/z80 it seems instead that XF and YF
// are copies of bits 3 and 5 of the tested register (r).
pub fn bit_r(state: *Z80State, opcode: *const OpCode) u8 {
    const n = opcode.y; // what bit to test, res or set

    switch (opcode.x) {
        1 => { // BIT
            const r = state.gp_registers[opcode.z];
            const b = (r >> n) & 1;
            state.AF.F.S = b != 0 and n == 7;
            state.AF.F.Z = b == 0;
            state.AF.F.H = true;
            state.AF.F.N = false;
            state.AF.F.PV = state.AF.F.Z;
            state.AF.F.X = r & XF != 0;
            state.AF.F.Y = r & YF != 0;
        },
        2 => { // RES
            state.gp_registers[opcode.z] &= ~(@as(u8, 1) << n);
        },
        3 => { // SET
            state.gp_registers[opcode.z] |= @as(u8, 1) << n;
        },
        else => {},
    }

    state.PC +%= 1;
    return 8;
}

pub fn bit_xy(state: *Z80State, opcode: *const OpCode) u8 {
    const n = opcode.y; // what bit to test, res or set
    const r = readIndexed(state, 0);
    var cycles: u8 = 0;

    switch (opcode.x) {
        1 => { // BIT
            const b = (r >> n) & 1;
            state.AF.F.S = b != 0 and n == 7;
            state.AF.F.Z = b == 0;
            state.AF.F.H = true;
            state.AF.F.N = false;
            state.AF.F.PV = state.AF.F.Z;
            state.AF.F.X = state.WZ.high & XF != 0;
            state.AF.F.Y = state.WZ.high & YF != 0;
            cycles = 20;
        },
        2 => { // RES
            const res = r & ~(@as(u8, 1) << n);
            storeIndexed(state, res, 0);
            if (opcode.z != 6) {
                state.gp_registers[opcode.z] = res;
            }
            cycles = 23;
        },
        3 => { // SET
            const res = r | @as(u8, 1) << n;
            storeIndexed(state, res, 0);
            if (opcode.z != 6) {
                state.gp_registers[opcode.z] = res;
            }
            cycles = 23;
        },
        else => {},
    }

    state.PC +%= 2;
    return cycles;
}

pub fn bit_hl(state: *Z80State, opcode: *const OpCode) u8 {
    const n = opcode.y; // what bit to test, res or set
    const hl = @intFromEnum(RegisterPairs.HL);
    const r = readIndirect(state, hl);
    var cycles: u8 = 0;
    switch (opcode.x) {
        1 => { // BIT
            const t: u8 = @truncate(r & (@as(u32, 1) << n));
            if (t == 0) {
                state.AF.F.S = false;
                state.AF.F.Z = true;
                state.AF.F.PV = true;
            } else {
                state.AF.F.S = t & SF != 0;
                state.AF.F.Z = false;
                state.AF.F.PV = false;
            }
            state.AF.F.H = true;
            state.AF.F.N = false;
            state.AF.F.X = state.WZ.high & XF != 0;
            state.AF.F.Y = state.WZ.high & YF != 0;
            cycles = 12;
        },
        2 => { // RES
            storeIndirect(state, r & ~(@as(u8, 1) << n), hl);
            cycles = 15;
        },
        3 => { // SET
            storeIndirect(state, r | @as(u8, 1) << n, hl);
            cycles = 15;
        },
        else => {},
    }

    state.PC +%= 1;
    return cycles;
}

// cc    Condition           Flag
//
// 000   Non-Zero (NZ)       Z
// 001   Zero (Z)            Z
// 010   No Carry (NC)       C
// 011   Carry (C)           C
// 100   Parity Odd (PO)     P/V
// 101   Parity Even (PE)    P/V
// 110   Sign Positive (P)   S
// 111   Sign Negative (M)   S
// where cc = opcode.y
pub const FlagConditions = enum(u3) {
    non_zero = 0,
    zero = 1,
    no_carry = 2,
    carry = 3,
    parity_odd = 4,
    parity_even = 5,
    sign_positive = 6,
    sign_negative = 7,
};

pub inline fn evalFlagsCondition(state: *const Z80State, condition: u3) bool {
    const cc: FlagConditions = @enumFromInt(condition);
    const flags = state.AF.F;

    return switch (cc) {
        .non_zero => !flags.Z,
        .zero => flags.Z,
        .no_carry => !flags.C,
        .carry => flags.C,
        .parity_odd => !flags.PV,
        .parity_even => flags.PV,
        .sign_positive => !flags.S,
        .sign_negative => flags.S,
    };
}

pub fn jp_nn(state: *Z80State, _: *const OpCode) u8 {
    const low = state.memRead(state.PC +% 1);
    const high = state.memRead(state.PC +% 2);
    const nn: u16 = @intCast(low | @as(u16, high) << 8);
    state.PC = nn;
    state.WZ.low = low;
    state.WZ.high = high;
    return 10;
}

pub fn jp_cc_nn(state: *Z80State, opcode: *const OpCode) u8 {
    const low = state.memRead(state.PC +% 1);
    const high = state.memRead(state.PC +% 2);
    const nn: u16 = @intCast(low | @as(u16, high) << 8);
    state.WZ.low = low;
    state.WZ.high = high;

    if (evalFlagsCondition(state, opcode.y)) {
        state.PC = nn;
        return 10;
    }

    state.PC +%= 3;
    return 10;
}

pub fn jr_e(state: *Z80State, _: *const OpCode) u8 {
    // NOTE: e actually is e - 2
    const pc: i32 = @intCast(state.PC);
    const e: i8 = @bitCast(state.memRead(state.PC +% 1));
    const address: u16 = @intCast(@mod(pc + e + 2, 65536));
    state.PC = address;
    state.WZ.setValue(address);
    return 12;
}

pub fn jr_ss_e(state: *Z80State, opcode: *const OpCode) u8 {
    // opcode.y = 1ss where ss = cc.
    if (evalFlagsCondition(state, opcode.y & 0b011)) {
        const pc: i32 = @intCast(state.PC);
        const e: i8 = @bitCast(state.memRead(state.PC +% 1));
        const address: u16 = @intCast(@mod(pc + e + 2, 65536));
        state.PC = address;
        state.WZ.setValue(address);
        return 12;
    }

    state.PC +%= 2;
    return 7;
}

pub fn djnz(state: *Z80State, _: *const OpCode) u8 {
    const b_dec = state.BC.high -% 1;
    state.BC.high = b_dec;

    if (b_dec != 0) {
        const pc: i32 = @intCast(state.PC);
        const e: i8 = @bitCast(state.memRead(state.PC +% 1));
        const address: u16 = @intCast(@mod(pc + e + 2, 65536));
        state.PC = address;
        state.WZ.setValue(address);
        return 13;
    }

    state.PC +%= 2;
    return 8;
}

pub fn jp_hl(state: *Z80State, _: *const OpCode) u8 {
    state.PC = state.HL.getValue();
    return 4;
}

pub fn jp_xy(state: *Z80State, _: *const OpCode) u8 {
    state.PC = state.addr_register.getValue();
    return 8;
}

pub fn cpl(state: *Z80State, _: *const OpCode) u8 {
    const t = ~state.AF.A;
    state.AF.F.N = true;
    state.AF.F.H = true;
    state.AF.F.X = t & XF != 0;
    state.AF.F.Y = t & YF != 0;
    state.AF.A = t;
    state.PC +%= 1;
    return 4;
}

pub fn neg(state: *Z80State, _: *const OpCode) u8 {
    const acc = state.AF.A;
    const accs: i32 = @intCast(acc);
    const t: u8 = @intCast(@mod(-accs, 256));
    state.AF.F.S = t & SF != 0;
    state.AF.F.Z = t == 0;
    state.AF.F.H = (acc ^ t) & HF != 0;
    state.AF.F.N = true;
    state.AF.F.PV = acc == 128; //0x80
    state.AF.F.C = acc != 0;
    state.AF.F.X = t & XF != 0;
    state.AF.F.Y = t & YF != 0;
    state.AF.A = t;
    state.PC +%= 1;
    return 8;
}

pub fn scf(state: *Z80State, _: *const OpCode) u8 {
    state.AF.F.C = true;
    state.AF.F.N = false;
    state.AF.F.H = false;
    state.AF.F.X = state.AF.A & XF != 0;
    state.AF.F.Y = state.AF.A & YF != 0;
    state.PC +%= 1;
    return 4;
}

pub fn ccf(state: *Z80State, _: *const OpCode) u8 {
    const cf = state.AF.F.C;
    state.AF.F.C = !cf;
    state.AF.F.H = cf;
    state.AF.F.N = false;
    state.AF.F.X = state.AF.A & XF != 0;
    state.AF.F.Y = state.AF.A & YF != 0;
    state.PC +%= 1;
    return 4;
}

pub fn di(state: *Z80State, _: *const OpCode) u8 {
    state.IFF1 = false;
    state.IFF2 = false;
    state.PC +%= 1;
    return 4;
}

pub fn ei(state: *Z80State, _: *const OpCode) u8 {
    state.IFF1 = true;
    state.IFF2 = true;
    state.PC +%= 1;
    return 4;
}

pub fn im_0(state: *Z80State, _: *const OpCode) u8 {
    state.IM = 0;
    state.PC +%= 1;
    return 8;
}

pub fn im_1(state: *Z80State, _: *const OpCode) u8 {
    state.IM = 1;
    state.PC +%= 1;
    return 8;
}

pub fn im_2(state: *Z80State, _: *const OpCode) u8 {
    state.IM = 2;
    state.PC +%= 1;
    return 8;
}

pub fn daa(state: *Z80State, _: *const OpCode) u8 {
    const ln = byte.getLowNibble(state.AF.A);

    // low-nibble adjustment
    var adjustment: u8 = if (state.AF.F.H or ln > 9) 0x06 else 0x00; // 0x00 or 0x06

    const bcd_carry = state.AF.A > 0x99;
    const prev_carry = state.AF.F.C;

    // high-nibble adjustment needed ?
    if (bcd_carry or prev_carry) {
        adjustment |= 0x60; // it becomes either 0x60 or 0x66
    }

    // add or subtract the diff based on if the prev operation was an add or sub.
    const t: u8 = switch (state.AF.F.N) { // was sub?
        inline true => state.AF.A -% adjustment,
        inline false => state.AF.A +% adjustment,
    };

    state.AF.F.Z = t == 0;
    state.AF.F.S = t & SF != 0;
    state.AF.F.X = t & XF != 0;
    state.AF.F.Y = t & YF != 0;
    state.AF.F.H = (state.AF.A ^ t) & HF != 0;
    state.AF.F.PV = hasEvenParity8(t);
    state.AF.F.C = prev_carry or bcd_carry;

    state.AF.A = t;

    state.PC +%= 1;
    return 4;
}

pub fn io_in_a_n(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const port = readImmediate(state, 0);
    const addr: u16 = @intCast(port | @as(u16, state.AF.A) << 8);
    state.AF.A = state.ioRead(addr);
    state.WZ.setValue(addr +% 1);
    state.PC +%= 1;
    return 11;
}

pub fn io_out_n_a(state: *Z80State, _: *const OpCode) u8 {
    state.PC +%= 1;
    const port = readImmediate(state, 0);
    const addr: u16 = @intCast(port | @as(u16, state.AF.A) << 8);
    state.ioWrite(addr, state.AF.A);
    state.WZ.low = port +% 1;
    state.WZ.high = state.AF.A;
    state.PC +%= 1;
    return 11;
}

pub fn io_in_r_c(state: *Z80State, opcode: *const OpCode) u8 {
    const addr = state.BC.getValue();
    const data = state.ioRead(addr);
    state.gp_registers[opcode.y] = data;
    state.AF.F.S = data & SF != 0;
    state.AF.F.H = false;
    state.AF.F.PV = hasEvenParity8(data);
    state.AF.F.X = data & XF != 0;
    state.AF.F.Y = data & YF != 0;
    state.AF.F.N = false;
    state.AF.F.Z = data == 0;
    state.PC +%= 1;
    state.WZ.setValue(addr +% 1);
    return 12;
}

pub fn io_in_r_U(state: *Z80State, _: *const OpCode) u8 {
    const addr = state.BC.getValue();
    const data = state.ioRead(addr);
    state.AF.F.S = data & SF != 0;
    state.AF.F.H = false;
    state.AF.F.PV = hasEvenParity8(data);
    state.AF.F.X = data & XF != 0;
    state.AF.F.Y = data & YF != 0;
    state.AF.F.N = false;
    state.AF.F.Z = data == 0;
    state.PC +%= 1;
    state.WZ.setValue(addr +% 1);
    return 12;
}

// see: Z80 undocumented - 4.3 I/O Block Instructions
pub inline fn io_ini_x(state: *Z80State, _: *const OpCode) u8 {
    const addr = state.BC.getValue();
    const data = state.ioRead(addr);
    state.memWrite(state.HL.getValue(), data);
    state.HL.increment();
    const t = data +% (@as(u32, state.BC.low +% 1) & 255);
    const B = state.BC.high -% 1;
    state.AF.F.S = B & SF != 0;
    state.AF.F.X = B & XF != 0;
    state.AF.F.Y = B & YF != 0;
    state.AF.F.Z = B == 0;
    state.AF.F.PV = hasEvenParity8(@truncate((t & 7) ^ B));
    state.AF.F.H = t > 255;
    state.AF.F.C = state.AF.F.H;
    state.AF.F.N = (data >> 6) & NF != 0; // 7th bit
    state.WZ.setValue(state.BC.getValue() +% 1);
    state.BC.high = B;
    return data;
}

pub fn io_ini(state: *Z80State, opcode: *const OpCode) u8 {
    _ = io_ini_x(state, opcode);
    state.PC +%= 1;
    return 16;
}

// see: https://github.com/hoglet67/Z80Decoder/wiki/Undocumented-Flags#inxr--otxr-interrupted
pub fn io_inir(state: *Z80State, opcode: *const OpCode) u8 {
    const data = io_ini_x(state, opcode);
    const B = state.BC.high;

    if (B != 0) {
        state.WZ.setValue(state.PC);
        state.PC -%= 1;
        const pc_high = (state.PC >> 8) & 0xFF;
        state.AF.F.X = pc_high & XF != 0;
        state.AF.F.Y = pc_high & YF != 0;
        const p = @intFromBool(state.AF.F.PV);
        if (state.AF.F.C) {
            if (data & 0x80 != 0) {
                state.AF.F.PV = p ^ z80Parity((B -% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x00;
            } else {
                state.AF.F.PV = p ^ z80Parity((B +% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x0F;
            }
        } else {
            state.AF.F.PV = p ^ z80Parity(B & 0x7) ^ 1 != 0;
        }
        return 21;
    }

    state.PC +%= 1;
    return 16;
}

// see: Z80 undocumented - 4.3 I/O Block Instructions
pub inline fn io_ind_x(state: *Z80State, _: *const OpCode) u8 {
    const addr = state.BC.getValue();
    const data = state.ioRead(addr);
    state.memWrite(state.HL.getValue(), data);
    state.HL.decrement();
    const t = data +% (@as(u32, state.BC.low -% 1) & 255);
    const B = state.BC.high -% 1;
    state.AF.F.S = B & SF != 0;
    state.AF.F.X = B & XF != 0;
    state.AF.F.Y = B & YF != 0;
    state.AF.F.Z = B == 0;
    state.AF.F.PV = hasEvenParity8(@truncate((t & 7) ^ B));
    state.AF.F.H = t > 255;
    state.AF.F.C = state.AF.F.H;
    state.AF.F.N = (data >> 6) & NF != 0; // 7th bit
    state.WZ.setValue(state.BC.getValue() -% 1);
    state.BC.high = B;
    return data;
}

pub fn io_ind(state: *Z80State, opcode: *const OpCode) u8 {
    _ = io_ind_x(state, opcode);
    state.PC +%= 1;
    return 16;
}

// see: https://github.com/hoglet67/Z80Decoder/wiki/Undocumented-Flags#inxr--otxr-interrupted
pub fn io_indr(state: *Z80State, opcode: *const OpCode) u8 {
    const data = io_ind_x(state, opcode);
    const B = state.BC.high;

    if (B != 0) {
        state.WZ.setValue(state.PC);
        state.PC -%= 1;
        const pc_high = (state.PC >> 8) & 0xFF;
        state.AF.F.X = pc_high & XF != 0;
        state.AF.F.Y = pc_high & YF != 0;
        const p = @intFromBool(state.AF.F.PV);
        if (state.AF.F.C) {
            if (data & 0x80 != 0) {
                state.AF.F.PV = p ^ z80Parity((B -% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x00;
            } else {
                state.AF.F.PV = p ^ z80Parity((B +% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x0F;
            }
        } else {
            state.AF.F.PV = p ^ z80Parity(B & 0x7) ^ 1 != 0;
        }
        return 21;
    }

    state.PC +%= 1;
    return 16;
}

pub fn io_out_c_r(state: *Z80State, opcode: *const OpCode) u8 {
    const addr = state.BC.getValue();
    const data = state.gp_registers[opcode.y];
    state.ioWrite(addr, data);
    state.WZ.setValue(addr +% 1);
    state.PC +%= 1;
    return 12;
}

pub fn io_out_c_0(state: *Z80State, _: *const OpCode) u8 {
    const addr = state.BC.getValue();
    state.ioWrite(addr, 0);
    state.WZ.setValue(addr +% 1);
    state.PC +%= 1;
    return 12;
}

pub inline fn io_outi_x(state: *Z80State, _: *const OpCode) u8 {
    const data = state.memRead(state.HL.getValue());
    state.BC.high -%= 1;
    const addr = state.BC.getValue();
    const B = state.BC.high;
    state.HL.increment();
    state.ioWrite(addr, data);
    const t = data +% @as(u32, state.HL.low);
    state.AF.F.S = B & SF != 0;
    state.AF.F.X = B & XF != 0;
    state.AF.F.Y = B & YF != 0;
    state.AF.F.Z = B == 0;
    state.AF.F.PV = hasEvenParity8(@truncate((t & 7) ^ B));
    state.AF.F.H = t > 255;
    state.AF.F.C = state.AF.F.H;
    state.AF.F.N = (data >> 6) & NF != 0; // 7th bit
    state.WZ.setValue(state.BC.getValue() +% 1);
    return data;
}

pub fn io_outi(state: *Z80State, opcode: *const OpCode) u8 {
    _ = io_outi_x(state, opcode);
    state.PC +%= 1;
    return 16;
}

pub fn io_otir(state: *Z80State, opcode: *const OpCode) u8 {
    const data = io_outi_x(state, opcode);
    const B = state.BC.high;

    if (B != 0) {
        state.WZ.setValue(state.PC);
        state.PC -%= 1;
        const pc_high = (state.PC >> 8) & 0xFF;
        state.AF.F.X = pc_high & XF != 0;
        state.AF.F.Y = pc_high & YF != 0;
        const p = @intFromBool(state.AF.F.PV);
        if (state.AF.F.C) {
            if (data & 0x80 != 0) {
                state.AF.F.PV = p ^ z80Parity((B -% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x00;
            } else {
                state.AF.F.PV = p ^ z80Parity((B +% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x0F;
            }
        } else {
            state.AF.F.PV = p ^ z80Parity(B & 0x7) ^ 1 != 0;
        }
        return 21;
    }

    state.PC +%= 1;
    return 16;
}

pub inline fn io_outd_x(state: *Z80State, _: *const OpCode) u8 {
    const data = state.memRead(state.HL.getValue());
    state.BC.high -%= 1;
    const addr = state.BC.getValue();
    const B = state.BC.high;
    state.HL.decrement();
    state.ioWrite(addr, data);
    const t = data +% @as(u32, state.HL.low);
    state.AF.F.S = B & SF != 0;
    state.AF.F.X = B & XF != 0;
    state.AF.F.Y = B & YF != 0;
    state.AF.F.Z = B == 0;
    state.AF.F.PV = hasEvenParity8(@truncate((t & 7) ^ B));
    state.AF.F.H = t > 255;
    state.AF.F.C = state.AF.F.H;
    state.AF.F.N = (data >> 6) & NF != 0; // 7th bit
    state.WZ.setValue(state.BC.getValue() -% 1);
    return data;
}

pub fn io_outd(state: *Z80State, opcode: *const OpCode) u8 {
    _ = io_outd_x(state, opcode);
    state.PC +%= 1;
    return 16;
}

pub fn io_otdr(state: *Z80State, opcode: *const OpCode) u8 {
    const data = io_outd_x(state, opcode);
    const B = state.BC.high;

    if (B != 0) {
        state.WZ.setValue(state.PC);
        state.PC -%= 1;
        const pc_high = (state.PC >> 8) & 0xFF;
        state.AF.F.X = pc_high & XF != 0;
        state.AF.F.Y = pc_high & YF != 0;
        const p = @intFromBool(state.AF.F.PV);
        if (state.AF.F.C) {
            if (data & 0x80 != 0) {
                state.AF.F.PV = p ^ z80Parity((B -% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x00;
            } else {
                state.AF.F.PV = p ^ z80Parity((B +% 1) & 0x7) ^ 1 != 0;
                state.AF.F.H = (B & 0x0F) == 0x0F;
            }
        } else {
            state.AF.F.PV = p ^ z80Parity(B & 0x7) ^ 1 != 0;
        }
        return 21;
    }

    state.PC +%= 1;
    return 16;
}

pub fn retn(state: *Z80State, _: *const OpCode) u8 {
    _ = ret_(state);
    state.IFF1 = state.IFF2;
    if (state.IFF1 and state.int_line == LineLow) { // IFF1 and INT line is low
        state.requests.int_signal = true;
    }
    return 14;
}

pub fn reti(state: *Z80State, _: *const OpCode) u8 {
    _ = ret_(state);
    state.IFF1 = state.IFF2;
    if (state.IFF1 and state.int_line == LineLow) { // IFF1 and INT line is low
        state.requests.int_signal = true;
    }
    return 14;
}

pub fn halt(state: *Z80State, _: *const OpCode) u8 {
    state.halt_line = LineHigh;

    state.PC +%= 1;

    while (true) {
        state.R.increment();
        state.cycles +%= 4;
        const requests: u3 = @bitCast(state.requests);
        if (requests > 0) {
            return 0;
        }
    }

    return 0;
}

// zig fmt: off
const instructions_table = [256]InstructionFn{
    //0        1         2             3            4           5         6          7          8             9          A             B            C           D          E        F
    nop,       ld_dd_nn, ld_bc_a,      inc_dec_bc,  al2_a_r,    al2_a_r,  ld_r_n,    rlca,      be_ex_af_af2, add_hl_bc, ld_a_bc,      inc_dec_bc,  al2_a_r,    al2_a_r,   ld_r_n,  rrca,      // 0
    djnz,      ld_dd_nn, ld_de_a,      inc_dec_de,  al2_a_r,    al2_a_r,  ld_r_n,    rla,       jr_e,         add_hl_de, ld_a_de,      inc_dec_de,  al2_a_r,    al2_a_r,   ld_r_n,  rra,       // 1
    jr_ss_e,   ld_dd_nn, ld_nn_hl_ext, inc_dec_hl,  al2_a_r,    al2_a_r,  ld_r_n,    daa,       jr_ss_e,      add_hl_hl, ld_hl_nn_ext, inc_dec_hl,  al2_a_r,    al2_a_r,   ld_r_n,  cpl,       // 2
    jr_ss_e,   ld_dd_nn, ld_nn_a,      inc_dec_sp,  al2_a_hl,   al2_a_hl, ld_hl_n,   scf,       jr_ss_e,      add_hl_sp, ld_a_nn,      inc_dec_sp,  al2_a_r,    al2_a_r,   ld_r_n,  ccf,       // 3
    nop,       ld_r_r,   ld_r_r,       ld_r_r,      ld_r_r,     ld_r_r,   ld_r_hl,   ld_r_r,    ld_r_r,       nop,       ld_r_r,       ld_r_r,      ld_r_r,     ld_r_r,    ld_r_hl, ld_r_r,    // 4
    ld_r_r,    ld_r_r,   nop,          ld_r_r,      ld_r_r,     ld_r_r,   ld_r_hl,   ld_r_r,    ld_r_r,       ld_r_r,    ld_r_r,       nop,         ld_r_r,     ld_r_r,    ld_r_hl, ld_r_r,    // 5
    ld_r_r,    ld_r_r,   ld_r_r,       ld_r_r,      ld_r_r,     ld_r_r,   ld_r_hl,   ld_r_r,    ld_r_r,       ld_r_r,    ld_r_r,       ld_r_r,      ld_r_r,     nop,       ld_r_hl, ld_r_r,    // 6
    ld_hl_r,   ld_hl_r,  ld_hl_r,      ld_hl_r,     ld_hl_r,    ld_hl_r,  halt,      ld_hl_r,   ld_r_r,       ld_r_r,    ld_r_r,       ld_r_r,      ld_r_r,     ld_r_r,    ld_r_hl, nop,       // 7
    al_a_r,    al_a_r,   al_a_r,       al_a_r,      al_a_r,     al_a_r,   al_a_hl,   al_a_r,    al_a_r,       al_a_r,    al_a_r,       al_a_r,      al_a_r,     al_a_r,    al_a_hl, al_a_r,    // 8
    al_a_r,    al_a_r,   al_a_r,       al_a_r,      al_a_r,     al_a_r,   al_a_hl,   al_a_r,    al_a_r,       al_a_r,    al_a_r,       al_a_r,      al_a_r,     al_a_r,    al_a_hl, al_a_r,    // 9
    al_a_r,    al_a_r,   al_a_r,       al_a_r,      al_a_r,     al_a_r,   al_a_hl,   al_a_r,    al_a_r,       al_a_r,    al_a_r,       al_a_r,      al_a_r,     al_a_r,    al_a_hl, al_a_r,    // A
    al_a_r,    al_a_r,   al_a_r,       al_a_r,      al_a_r,     al_a_r,   al_a_hl,   al_a_r,    al_a_r,       al_a_r,    al_a_r,       al_a_r,      al_a_r,     al_a_r,    al_a_hl, al_a_r,    // B
    ret_cc,    pop_qq,   jp_cc_nn,     jp_nn,       call_cc_nn, push_qq,  al_a_n,    rst_p,     ret_cc,       ret,       jp_cc_nn,     cb_prefix,   call_cc_nn, call_nn,   al_a_n,  rst_p,     // C
    ret_cc,    pop_qq,   jp_cc_nn,     io_out_n_a,  call_cc_nn, push_qq,  al_a_n,    rst_p,     ret_cc,       be_exx,    jp_cc_nn,     io_in_a_n,   call_cc_nn, dd_prefix, al_a_n,  rst_p,     // D
    ret_cc,    pop_qq,   jp_cc_nn,     be_ex_sp_hl, call_cc_nn, push_qq,  al_a_n,    rst_p,     ret_cc,       jp_hl,     jp_cc_nn,     be_ex_de_hl, call_cc_nn, ed_prefix, al_a_n,  rst_p,     // E
    ret_cc,    pop_af,   jp_cc_nn,     di,          call_cc_nn, push_af,  al_a_n,    rst_p,     ret_cc,       ld_sp_hl,  jp_cc_nn,     ei,          call_cc_nn, fd_prefix, al_a_n,  rst_p,     // F
};

// Indexed addressing XY opcodes
// zig fmt: off
const xy_instructions_table = [256]InstructionFn{
    //0         1            2             3            4          5             6           7           8           9           A             B             C             D           E           F
    nop_nop,    xy_illegal, xy_illegal,   xy_illegal,  al2_a_p,    al2_a_p,      ld_xy_p_n,  xy_illegal, xy_illegal, add_xy_bc,  xy_illegal,   xy_illegal,   al2_a_p,      al2_a_p,    ld_xy_p_n,  xy_illegal, // 0
    xy_illegal, xy_illegal, xy_illegal,   xy_illegal,  al2_a_p,    al2_a_p,      ld_xy_p_n,  xy_illegal, xy_illegal, add_xy_de,  xy_illegal,   xy_illegal,   al2_a_p,      al2_a_p,    ld_xy_p_n,  xy_illegal, // 1
    xy_illegal, ld_xy_nn,   ld_nn_xy_ext, inc_dec_xy,  al2_a_p,    al2_a_p,      ld_xy_p_n,  xy_illegal, xy_illegal, add_xy_ix,  ld_xy_nn_ext, inc_dec_xy,   al2_a_p,      al2_a_p,    ld_xy_p_n,  xy_illegal, // 2
    xy_illegal, xy_illegal, xy_illegal,   xy_illegal,  al2_a_xy,   al2_a_xy,     ld_xy_n,    xy_illegal, xy_illegal, add_xy_sp,  xy_illegal,   xy_illegal,   al2_a_p,      al2_a_p,    ld_xy_p_n,  xy_illegal, // 3
    nop_nop,    ld_u_r_r,   ld_u_r_r,     ld_u_r_r,    ld_p_xyh,   ld_p_xyl,     ld_r_xy,    ld_u_r_r,   ld_u_r_r,   nop_nop,    ld_u_r_r,     ld_u_r_r,     ld_p_xyh,     ld_p_xyl,   ld_r_xy,    ld_u_r_r,   // 4
    ld_u_r_r,   ld_u_r_r,   nop_nop,      ld_u_r_r,    ld_p_xyh,   ld_p_xyl,     ld_r_xy,    ld_u_r_r,   ld_u_r_r,   ld_u_r_r,   ld_u_r_r,     nop_nop,      ld_p_xyh,     ld_p_xyl,   ld_r_xy,    ld_u_r_r,   // 5
    ld_xyh_p,   ld_xyh_p,   ld_xyh_p,     ld_xyh_p,    nop_nop,    ld_p_xyh_xyl, ld_r_xy,    ld_xyh_p,   ld_xyl_p,   ld_xyl_p,   ld_xyl_p,     ld_xyl_p,     ld_p_xyl_xyh, nop_nop,    ld_r_xy,    ld_xyl_p,   // 6
    ld_xy_r,    ld_xy_r,    ld_xy_r,      ld_xy_r,     ld_xy_r,    ld_xy_r,      xy_illegal, ld_xy_r,    ld_u_r_r,   ld_u_r_r,   ld_u_r_r,     ld_u_r_r,     ld_p_xyh,     ld_p_xyl,   ld_r_xy,    nop_nop,    // 7
    al_a_p,     al_a_p,     al_a_p,       al_a_p,      al_a_p,     al_a_p,       al_a_xy,    al_a_p,     al_a_p,     al_a_p,     al_a_p,       al_a_p,       al_a_p,       al_a_p,     al_a_xy,    al_a_p,     // 8
    al_a_p,     al_a_p,     al_a_p,       al_a_p,      al_a_p,     al_a_p,       al_a_xy,    al_a_p,     al_a_p,     al_a_p,     al_a_p,       al_a_p,       al_a_p,       al_a_p,     al_a_xy,    al_a_p,     // 9
    al_a_p,     al_a_p,     al_a_p,       al_a_p,      al_a_p,     al_a_p,       al_a_xy,    al_a_p,     al_a_p,     al_a_p,     al_a_p,       al_a_p,       al_a_p,       al_a_p,     al_a_xy,    al_a_p,     // A
    al_a_p,     al_a_p,     al_a_p,       al_a_p,      al_a_p,     al_a_p,       al_a_xy,    al_a_p,     al_a_p,     al_a_p,     al_a_p,       al_a_p,       al_a_p,       al_a_p,     al_a_xy,    al_a_p,     // B
    xy_illegal, xy_illegal, xy_illegal,   xy_illegal,  xy_illegal, xy_illegal,   xy_illegal, xy_illegal, xy_illegal, xy_illegal, xy_illegal,   xy_cb_prefix, xy_illegal,   xy_illegal, xy_illegal, xy_illegal, // C
    xy_illegal, xy_illegal, xy_illegal,   xy_illegal,  xy_illegal, xy_illegal,   xy_illegal, xy_illegal, xy_illegal, xy_illegal, xy_illegal,   xy_illegal,   xy_illegal,   xy_xy_seq,  xy_illegal, xy_illegal, // D
    xy_illegal, pop_xy,     xy_illegal,   be_ex_sp_xy, xy_illegal, push_xy,      xy_illegal, xy_illegal, xy_illegal, jp_xy,      xy_illegal,   xy_illegal,   xy_illegal,   xy_illegal, xy_illegal, xy_illegal, // E
    xy_illegal, xy_illegal, xy_illegal,   xy_illegal,  xy_illegal, xy_illegal,   xy_illegal, xy_illegal, xy_illegal, ld_sp_xy,   xy_illegal,   xy_illegal,   xy_illegal,   xy_xy_seq,  xy_illegal, xy_illegal, // F
};

// zig fmt: off
const ed_instructions_table = [256]InstructionFn{
    //0         1           2           3             4           5           6           7           8           9           A           B             C           D           E           F
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // 0
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // 1
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // 2
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // 3
    io_in_r_c,  io_out_c_r, sbc_hl_bc,  ld_nn_dd_ext, neg,        retn,       im_0,       ld_i_a,     io_in_r_c,  io_out_c_r, adc_hl_bc,  ld_dd_nn_ext,  neg,        reti,       im_0,       ld_r_a,     // 4
    io_in_r_c,  io_out_c_r, sbc_hl_de,  ld_nn_dd_ext, neg,        retn,       im_1,       ld_a_i,     io_in_r_c,  io_out_c_r, adc_hl_de,  ld_dd_nn_ext,  neg,        retn,       im_2,       ld_a_r,     // 5
    io_in_r_c,  io_out_c_r, sbc_hl_hl,  ld_nn_dd_ext, neg,        retn,       im_0,       rrd,        io_in_r_c,  io_out_c_r, adc_hl_hl,  ld_dd_nn_ext,  neg,        retn,       im_0,       rld,        // 6
    io_in_r_U,  io_out_c_0, sbc_hl_sp,  ld_nn_dd_ext, neg,        retn,       im_1,       ed_illegal, io_in_r_c,  io_out_c_r, adc_hl_sp,  ld_dd_nn_ext,  neg,        retn,       im_2,       ed_illegal, // 7
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // 8
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // 9
    bt_ldi,     bs_cpi,     io_ini,     io_outi,      ed_illegal, ed_illegal, ed_illegal, ed_illegal, bt_ldd,     bs_cpd,     io_ind,     io_outd,       ed_illegal, ed_illegal, ed_illegal, ed_illegal, // A
    bt_ldir,    bs_cpir,    io_inir,    io_otir,      ed_illegal, ed_illegal, ed_illegal, ed_illegal, bt_lddr,    bs_cpdr,    io_indr,    io_otdr,       ed_illegal, ed_illegal, ed_illegal, ed_illegal, // B
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // C
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // D
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // E
    ed_illegal, ed_illegal, ed_illegal, ed_illegal,   ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal, ed_illegal,    ed_illegal, ed_illegal, ed_illegal, ed_illegal, // F
};

// zig fmt: off
const cb_instructions_table = [256]InstructionFn{
    // 0   1      2      3      4      5      6       7      8      9      A      B      C      D      E       F
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, // 0
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, // 1
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, // 2
    rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_r, rot_hl, rot_r, // 3
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // 4
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // 5
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // 6
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // 7
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // 8
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // 9
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // A
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // B
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // C
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // D
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // E
    bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_r, bit_hl, bit_r, // F
};

// DDCBx or FDCBx
// zig fmt: off
const xy_cb_instructions_table = [256]InstructionFn{
    //0     1       2       3       4       5       6       7       8       9       A       B       C       D       E       F
    rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, // 0
    rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, // 1
    rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, // 2
    rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, rot_xy, // 3
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // 4
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // 5
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // 6
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // 7
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // 8
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // 9
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // A
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // B
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // C
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // D
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // E
    bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, bit_xy, // F
};

pub inline fn fetchOpcode(state: *Z80State) u8 {
    const opcode: u8 = state.memRead(state.PC);
    state.R.increment();
    return opcode;
}

pub fn exec(state: *Z80State, opcode: u8) void {
    const decoded: *const OpCode = @ptrCast(&opcode);
    const insn_func = instructions_table[opcode];
    const insn_cycles = insn_func(state, decoded);
    state.cycles +%= insn_cycles;
}

pub fn execLoop(state: *Z80State) void {
    while(true)
    {
        const requests: u3 = @bitCast(state.requests);
        
        if (requests > 0) {
            if (state.requests.nmi_signal){ // non-maskable interrupts (NMI)
                state.iff1 = false;
                if (state.halt_line == LineLow) state.halt_line = LineHigh;
                state.R.increment();
                // push PC onto the stack at SP
                var sp = state.SP.getValue();
                sp -%= 1;
                state.memWrite(sp, (state.PC >> 8) & 0xFF);
                sp -%= 1;
                state.memWrite(sp, state.PC & 0xFF);
                state.SP.setValue(sp);
                // set PC (and MEMPTR) to the interrupt handler at 0x66
                state.PC = 0x66;
                state.WZ.low = 0x66;
                state.WZ.high = 0;
                state.cycles +%= 11;
                continue;
            }
            else if (state.requests.int_signal) { // maskable interrupts (INT)
                if (state.halt_line == LineLow) state.halt_line = LineHigh;
                var sp = state.SP.getValue();
                sp -%= 1;
                state.memWrite(sp, (state.PC >> 8) & 0xFF);
                sp -%= 1;
                state.memWrite(sp, state.PC & 0xFF);
                state.SP.setValue(sp);
                // set PC (and MEMPTR) to the interrupt handler at 0x38
                state.PC = 0x38;
                state.WZ.low = 0x38;
                state.WZ.high = 0;
                state.cycles +%= 13;
                continue;
            }
        }

        const opcode = fetchOpcode(state);
        const decoded: *const OpCode = @ptrCast(&opcode);
        const insn_func = instructions_table[opcode];
        const insn_cycles = insn_func(state, decoded);
        state.cycles +%= insn_cycles;
    }
}

pub const OpExecError = error {OpNotImplemented,OpIllegal};

pub fn testExec(state: *Z80State, opcode: u8) OpExecError!void {
    const decoded: *const OpCode = @ptrCast(&opcode);
    
    var table = &instructions_table;
    var next = state.memRead(state.PC +% 1);

    switch (opcode) {
        0xdd => {
            if (next == 0xcb){
                table = &xy_cb_instructions_table;
            }
            else {
                table = &xy_instructions_table;
            }
        },
        0xfd => {
            if (next == 0xcb){
                table = &xy_cb_instructions_table;
            }
            else {
                table = &xy_instructions_table;
            }
        },
        0xcb => table = &cb_instructions_table,
        0xed => table = &ed_instructions_table,
        else => {
            next = opcode;
        }
    }

    const func: ?InstructionFn = table[next];

    if (func)
    {
        const insn_func = instructions_table[opcode];
        const insn_cycles = insn_func(state, decoded);
        state.cycles +%= insn_cycles;
    }
    else {
        return OpExecError.OpNotImplemented;
    }
}