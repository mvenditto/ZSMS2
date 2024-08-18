//! This file contains the structures to represent a Z80 CPU state.

const std = @import("std");
const assert = std.debug.assert;
const host_endianess = @import("builtin").target.cpu.arch.endian();

const build_opts = @import("build_options");

/// If true simulates Q, otherwise do not. Affects how SCF/CCF flags are calculated.
pub const z80_sim_q = build_opts.z80_sim_q;

/// Represents a 16-bit register.
pub const SixteenBitRegister = packed struct {
    high: u8 = 0, // 0-7
    low: u8 = 0, // 8-15

    pub inline fn getValue(self: *const SixteenBitRegister) u16 {
        return @intCast(self.low | @as(u16, self.high) << 8);
    }

    pub inline fn setValue(self: *SixteenBitRegister, value: u16) void {
        self.low = @truncate(value & 0xFF);
        self.high = @truncate((value >> 8) & 0xFF);
    }

    pub inline fn increment(self: *SixteenBitRegister) void {
        const r = @addWithOverflow(self.low, 1);
        const carry = r[1];
        self.low = r[0];
        self.high +%= carry;
    }

    pub inline fn decrement(self: *SixteenBitRegister) void {
        const r = @subWithOverflow(self.low, 1);
        const borrow = r[1];
        self.low = r[0];
        self.high -%= borrow;
    }

    /// Sets the low byte of the register.
    pub inline fn setLow(self: *SixteenBitRegister, value: u8) void {
        self.low = value;
    }

    /// Sets the high byte of the register.
    pub inline fn setHigh(self: *SixteenBitRegister, value: u8) void {
        self.high = value;
    }

    comptime {
        std.debug.assert(@sizeOf(@This()) == @sizeOf(u16));
        std.debug.assert(@bitSizeOf(@This()) == @bitSizeOf(u16));
        std.debug.assert(@bitOffsetOf(SixteenBitRegister, "high") == 0);
        std.debug.assert(@bitOffsetOf(SixteenBitRegister, "low") == 8);
    }
};

/// The Flag registers (F) that supply information about the status of the Z80 CPU.
pub const FlagsRegister = packed struct {
    /// Carry flag
    C: bool = false, // 0
    /// Add/Subtract flag
    N: bool = false, // 1
    /// Parity/Overflow flag
    PV: bool = false, // 2
    /// NotUsed
    X: bool = false, // 3
    /// Half-Carry flag
    H: bool = false, // 4
    /// NotUsed
    Y: bool = false, // 5
    /// Zero flag
    Z: bool = false, // 6
    /// Sign flag
    S: bool = false, // 7

    comptime {
        std.debug.assert(@sizeOf(@This()) == @sizeOf(u8));
        std.debug.assert(@bitSizeOf(@This()) == @bitSizeOf(u8));
        std.debug.assert(@bitOffsetOf(FlagsRegister, "C") == 0);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "N") == 1);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "PV") == 2);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "X") == 3);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "H") == 4);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "Y") == 5);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "Z") == 6);
        std.debug.assert(@bitOffsetOf(FlagsRegister, "S") == 7);
    }
};

/// The 16-bit register that stores the accumulator (A) and flags (F).
pub const AccumulatorFlagsRegister = packed struct {
    F: FlagsRegister = .{},
    A: u8 = 0,

    pub inline fn getFlags(self: *const AccumulatorFlagsRegister) u8 {
        return @bitCast(self.F);
    }

    pub inline fn setFlags(self: *AccumulatorFlagsRegister, flags: u8) void {
        self.F = @bitCast(flags);
    }

    pub inline fn getAccumulator(self: *const AccumulatorFlagsRegister) u8 {
        return self.A;
    }

    pub inline fn setAccumulator(self: *AccumulatorFlagsRegister, val: u8) u8 {
        self.A = val;
    }

    pub inline fn getValue(self: *const AccumulatorFlagsRegister) u16 {
        const low: u8 = @bitCast(self.F);
        const high: u8 = self.A;
        return @intCast(low | @as(u16, high) << 8);
    }

    pub inline fn setValue(self: *AccumulatorFlagsRegister, value: u16) void {
        const low: u8 = @truncate(value & 0xFF);
        const high: u8 = @truncate((value >> 8) & 0xFF);
        self.A = high;
        self.F = @bitCast(low);
    }

    comptime {
        std.debug.assert(@sizeOf(@This()) == @sizeOf(u16));
        std.debug.assert(@bitSizeOf(@This()) == @bitSizeOf(u16));
    }
};

pub const EightBitRegisters = enum(u8) {
    B = 0, // 000
    C = 1, // 001
    D = 2, // 010
    E = 3, // 011
    H = 4, // 100
    L = 5, // 101
    F = 6, // 110
    A = 7, // 111
};

pub const RegisterPairs1 = enum(u8) {
    BC = 0,
    DE = 1,
    HL = 2,
    SP = 3,
};

// featuring AF instead of SP.
pub const RegisterPairs2 = enum(u8) {
    BC = 0,
    DE = 1,
    HL = 2,
    AF = 3,
};

/// The 8-bit memory refresh register (R).
/// see: http://www.z80.info/z80_faq.htm#Q-1
pub const MemoryRefreshRegister = packed struct(u8) {
    /// 7-bits refresh counter, incremented on every fetch.
    refresh_counter: u7 = 0,
    /// 8th bit, remains as programmed (e.g from an LD R, A instruction).
    R7: u1 = 0,

    const Self = @This();

    /// Returns the full 8-bit value of the whole R register.
    pub inline fn getValue(self: *const Self) u8 {
        return @bitCast(self.*);
    }

    pub inline fn setValue(self: *Self, value: u8) void {
        self.* = @bitCast(value);
    }

    /// Increments the 7-bits memory refresh counter.
    /// Does not affect R7.
    pub inline fn increment(self: *Self) void {
        self.refresh_counter +%= 1;
    }
};

// I/O functions

/// Called by the emulator when a memory **read** is required.
/// Must return the byte read from `address`.
pub const ReadFnPtr = *const fn (
    /// The context, as passed when initializing the `Z80State`.
    ctx: *anyopaque,
    /// The 16-bit address to read from.
    address: u16,
) u8;

/// Called by the emulator when a **write** read is required.
pub const WriteFnPtr = *const fn (
    /// The context, as passed when initializing the `Z80State`.
    ctx: *anyopaque,
    /// The 16-bit address to write `value`to.
    address: u16,
    /// The value to write at the specified memory location.
    value: u8,
) void;

/// Called by the emulator when an I/O **read** is required.
/// Must return the byte read from the specified port.
pub const IOReadFnPtr = *const fn (
    /// The context, as passed when initializing the `Z80State`.
    ctx: *anyopaque,
    /// The address from the address-bus. The low byte represents the`port`.
    address: u16,
) u8;

/// Called by the emulator when an I/O **write** is required.
pub const IOWriteFnPtr = *const fn (
    /// The context, as passed when initializing the `Z80State`.
    ctx: *anyopaque,
    /// The address from the address-bus. The low byte represents the `port`.
    address: u16,
    /// The value to write at the specified I/O port.
    value: u8,
) void;

pub const InterruptAck = *const fn (ctx: *anyopaque) u8;

pub const Z80Requests = packed struct(u3) {
    /// Maskable interrupt request.
    int_signal: bool,
    /// Non-maskable interrupt request.
    nmi_signal: bool,
    /// Reset request.
    rst_signal: bool,
};

pub const LineLow = 0;
pub const LineHigh = 1;

/// The state of the emulated Z80 CPU.
pub const Z80State = packed struct {
    // General-purpose registers
    BC: SixteenBitRegister = .{},
    DE: SixteenBitRegister = .{},
    HL: SixteenBitRegister = .{},
    AF: AccumulatorFlagsRegister = .{},

    // Alternative (shadow) general-purpose registers
    BC_: SixteenBitRegister = .{},
    DE_: SixteenBitRegister = .{},
    HL_: SixteenBitRegister = .{},
    AF_: AccumulatorFlagsRegister = .{},

    // Special-purpose registers
    SP: SixteenBitRegister = .{}, // stack pointer
    IX: SixteenBitRegister = .{}, // index register x
    IY: SixteenBitRegister = .{}, // index register y

    // The MEMPTR, see: https://github.com/floooh/emu-info/blob/master/z80/memptr_eng.txt
    WZ: SixteenBitRegister = .{}, // aka MEMPTR

    // Index: B,C,D,E,IXh,IXl,A
    _p0: *u8 = undefined, // B
    _p1: *u8 = undefined, // C
    _p2: *u8 = undefined, // D
    _p3: *u8 = undefined, // E
    _p4: *u8 = undefined, // IXh
    _p5: *u8 = undefined, // IXl
    _p6: *u8 = undefined, //
    _p7: *u8 = undefined, // A

    // Index: B,C,D,E,IYh,IYl,A
    _q0: *u8 = undefined, // B
    _q1: *u8 = undefined, // C
    _q2: *u8 = undefined, // D
    _q3: *u8 = undefined, // E
    _q4: *u8 = undefined, // IYh
    _q5: *u8 = undefined, // IYl
    _q6: *u8 = undefined, //
    _q7: *u8 = undefined, // A

    // Index: BC,DE,HL,SP
    _d0: *SixteenBitRegister = undefined, // B
    _d1: *SixteenBitRegister = undefined, // C
    _d2: *SixteenBitRegister = undefined, // D
    _d3: *SixteenBitRegister = undefined, // E

    /// The Program Counter.
    PC: u16 = 0,

    /// The memory Refresh counter.
    R: MemoryRefreshRegister = .{},

    /// The Interrupt page address (high-order byte).
    I: u8 = 0,

    /// The interrupt mode.
    IM: u8 = 0,

    /// Updated by every instruction based on wherever or not Flags were affected:
    /// * Flags affected: Q <- F.
    /// * Flags NOT affected: Q <- 0.
    ///
    /// Q affects SCF/CCF flags calculation.
    Q: u8 = 0,

    /// Either IX or IY, based on what FD/DD prefix was fetched last.
    addr_register: *SixteenBitRegister = undefined,

    /// Index {B, C, D, E, H, L, A, F, B', C', ..., F'}.
    /// General purpose register + shadow registers.
    gp_registers: *[16]u8 = undefined,

    /// Index {BC, DE, HL, AF}.
    gp_registers_pairs: *[4]SixteenBitRegister = undefined,

    /// Index: {B, C, D, E, IXh, IXl, A}.
    p: *[8]*u8 = undefined,

    /// Index {B, C, D, E, IYh, IYl, A}.
    q: *[8]*u8 = undefined,

    /// Index {BC, DE, HL, SP}. Featuring SP instead of AF.
    d: *[4]*SixteenBitRegister = undefined, // BC,DE,HL,SP

    /// Either `p` or `q`, based on what FD/DD prefix was fetched last.
    pq: *[8]*u8 = undefined,

    /// The count of emulated T-cycles.
    cycles: u64 = 0,

    /// The max number of cycles to execute.
    max_cycles: u64 = std.math.maxInt(u64),

    // inline vtable
    ctx: *anyopaque,
    memReadFn: ReadFnPtr,
    memWriteFn: WriteFnPtr,
    ioReadFn: IOReadFnPtr,
    ioWriteFn: IOWriteFnPtr,
    intAck: InterruptAck,

    // !!!
    // NOTE: bool fields, MUST stay after the function pointers, see: https://github.com/ziglang/zig/issues/20539
    // !!!

    // Interrupt Enable Flip-Flops IFF1 and IFF2
    /// Disables interrupts from being accepted
    IFF1: bool = false,
    /// Temporary storage location for IFF1
    IFF2: bool = false,

    requests: Z80Requests = .{},

    // lines
    int_line: u1 = LineHigh,
    halt_line: u1 = LineHigh,

    const Self = @This();

    pub fn init(
        allocator: std.mem.Allocator,
        ctx: anytype,
        read_fn: ReadFnPtr,
        write_fn: WriteFnPtr,
        io_read_fn: IOReadFnPtr,
        io_write_fn: IOWriteFnPtr,
        // int_ack: InterruptAck,
    ) !*Self {
        var state = try allocator.create(Self);

        const ptr_info = @typeInfo(@TypeOf(ctx));
        assert(ptr_info == .Pointer); // must be a pointer
        assert(ptr_info.Pointer.size == .One); // must be a single-item pointer
        assert(@typeInfo(ptr_info.Pointer.child) == .Struct); // must point to a struct

        state.PC = 0;
        state.I = 0;
        state.IFF1 = false;
        state.IFF2 = false;

        if (build_opts.z80_sim_q) {
            state.Q = 0;
        }

        state.R.setValue(0);

        state.BC.setValue(0xFFFF);
        state.DE.setValue(0xFFFF);
        state.HL.setValue(0xFFFF);
        state.AF.setValue(0xFFFF);

        state.IX.setValue(0xFFFF);
        state.IY.setValue(0xFFFF);
        state.SP.setValue(0xFFFF);
        state.WZ.setValue(0xFFFF);

        state.BC_.setValue(0xFFFF);
        state.DE_.setValue(0xFFFF);
        state.HL_.setValue(0xFFFF);
        state.AF_.setValue(0xFFFF);

        state.ctx = @constCast(@ptrCast(@alignCast(ctx)));
        state.memReadFn = read_fn;
        state.memWriteFn = write_fn;
        state.ioReadFn = io_read_fn;
        state.ioWriteFn = io_write_fn;
        state.intAck = undefined;

        state.requests = Z80Requests{
            .int_signal = false,
            .nmi_signal = false,
            .rst_signal = false,
        };

        // registers
        state.gp_registers = @ptrCast(state);
        state.gp_registers_pairs = @ptrCast(state);

        state.cycles = 0;

        // register sets (indexers)
        state._p0 = &state.BC.high;
        state._p1 = &state.BC.low;
        state._p2 = &state.DE.high;
        state._p3 = &state.DE.low;
        state._p4 = &state.IX.high;
        state._p5 = &state.IX.low;
        state._p6 = undefined;
        state._p7 = &state.AF.A;

        state._q0 = &state.BC.high;
        state._q1 = &state.BC.low;
        state._q2 = &state.DE.high;
        state._q3 = &state.DE.low;
        state._q4 = &state.IY.high;
        state._q5 = &state.IY.low;
        state._q6 = undefined;
        state._q7 = &state.AF.A;

        state._d0 = &state.BC;
        state._d1 = &state.DE;
        state._d2 = &state.HL;
        state._d3 = &state.SP;

        const indexed: [*]*u8 = @ptrCast(state);

        const p_start = @offsetOf(Self, "_p0") / @sizeOf(*u8);
        state.p = indexed[p_start .. p_start + 8];

        const q_start = @offsetOf(Self, "_q0") / @sizeOf(*u8);
        state.q = indexed[q_start .. q_start + 8];

        state.pq = state.p;

        const indexed2: [*]*SixteenBitRegister = @ptrCast(state);

        const d_start = @offsetOf(Self, "_d0") / @sizeOf(*u8);
        state.d = indexed2[d_start .. d_start + 4];

        return state;
    }

    pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
        allocator.destroy(self);
    }

    pub fn memRead(self: *const Self, address: u16) u8 {
        return self.memReadFn(self.ctx, address);
    }

    pub fn memWrite(self: *const Self, address: u16, value: u8) void {
        self.memWriteFn(self.ctx, address, value);
    }

    pub fn ioRead(self: *const Self, port: u16) u8 {
        return self.ioReadFn(self.ctx, port);
    }

    pub fn ioWrite(self: *const Self, port: u16, value: u8) void {
        self.ioWriteFn(self.ctx, port, value);
    }

    pub fn interruptAck(self: *const Self) u8 {
        return self.intAck(self.ctx);
    }

    pub fn resetSignals(self: *Self) void {
        self.requests = @bitCast(@as(u3, 0));
    }

    pub inline fn updateQ(self: *Self) void {
        if (build_opts.z80_sim_q) {
            self.Q = @bitCast(self.AF.F);
        }
    }

    pub inline fn resetQ(self: *Self) void {
        if (build_opts.z80_sim_q) {
            self.Q = 0;
        }
    }
};
