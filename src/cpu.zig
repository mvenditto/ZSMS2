const std = @import("std");
const host_endianess = @import("builtin").target.cpu.arch.endian();

pub const SixteenBitRegister = packed struct {
    high: u8 = 0, // 0-7
    low: u8 = 0, // 8-15

    pub inline fn getValue(self: *const SixteenBitRegister) u16 {
        // return switch (host_endianess) {
        //     .little => @bitCast(self.*),
        //     .big => {
        //         const integer: u16 = @bitCast(self.*);
        //         return @byteSwap(integer);
        //     },
        // };
        return @intCast(self.low | @as(u16, self.high) << 8);
    }

    pub fn setValue(self: *SixteenBitRegister, value: u16) void {
        // self.* = switch (host_endianess) {
        //     .little => @bitCast(value),
        //     .big => @bitCast(@byteSwap(value)),
        // };
        self.low = @truncate(value & 0xFF);
        self.high = @truncate((value >> 8) & 0xFF);
    }

    pub fn setLow(self: *SixteenBitRegister, value: u8) void {
        self.low = value;
    }

    pub fn setHigh(self: *SixteenBitRegister, value: u8) void {
        self.high = value;
    }

    comptime {
        std.debug.assert(@sizeOf(@This()) == @sizeOf(u16));
        std.debug.assert(@bitSizeOf(@This()) == @bitSizeOf(u16));
        std.debug.assert(@bitOffsetOf(SixteenBitRegister, "high") == 0);
        std.debug.assert(@bitOffsetOf(SixteenBitRegister, "low") == 8);
    }
};

/// The Flag registers supply information about the status of the Z80 CPU.
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

pub const AccumulatorFlagsRegister = packed struct {
    F: FlagsRegister = .{},
    A: u8 = 0,

    pub fn getFlags(self: *const AccumulatorFlagsRegister) u8 {
        return @bitCast(self.F);
    }

    pub fn setFlags(self: *AccumulatorFlagsRegister, flags: u8) void {
        self.F = @bitCast(flags);
    }

    pub inline fn getAccumulator(self: *const AccumulatorFlagsRegister) u8 {
        return self.A;
    }

    pub inline fn setAccumulator(self: *AccumulatorFlagsRegister, val: u8) u8 {
        self.A = val;
    }

    pub fn getValue(self: *AccumulatorFlagsRegister) u16 {
        return switch (host_endianess) {
            .little => @bitCast(self.*),
            .big => {
                const integer: u16 = @bitCast(self.*);
                return @byteSwap(integer);
            },
        };
    }

    pub fn setValue(self: *AccumulatorFlagsRegister, value: u16) void {
        self.* = switch (host_endianess) {
            .little => @bitCast(value),
            .big => @bitCast(@byteSwap(value)),
        };
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

// see: http://www.z80.info/z80_faq.htm#Q-1
pub const MemoryRefreshRegister = packed struct(u8) {
    refresh_counter: u7 = 0, // 7-bits refresh counter, incremented on every fetch.
    R7: u1 = 0, // 8th bit, remains as programmed from an LD R, A instruction.

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
    SP: u16 = 0, // stack pointer
    PC: u16 = 0, // program counter
    IX: SixteenBitRegister = .{}, // index register x
    IY: SixteenBitRegister = .{}, // index register y
    R: MemoryRefreshRegister = .{}, // memory refresh
    I: u8 = 0, // interrupt page address (high-order byte)

    // Interrupt Enable Flip-Flops
    IFF1: bool = false, // Disables interrupts from being accepted
    IFF2: bool = false, // Temporary storage location for IFF1

    addr_register: *SixteenBitRegister = undefined,

    // indexing over the 16 8-bit registers.
    gp_registers: *[16]u8 = undefined,

    // indexing over the 8 16-bit registers
    gp_registers_pairs: *[4]SixteenBitRegister = undefined,

    // memory
    memory: [*]u8 = undefined,
    memory_len: usize,

    const Self = @This();

    pub fn create(allocator: std.mem.Allocator) !*Self {
        var state = try allocator.create(Self);
        state.gp_registers = @ptrCast(state);
        state.gp_registers_pairs = @ptrCast(state);
        const buff = try allocator.alloc(u8, 64 * 1024); // 64KB
        state.memory_len = buff.len;
        state.memory = buff.ptr;
        return state;
    }

    pub fn free(self: *Self, allocator: std.mem.Allocator) void {
        const buff: []u8 = self.memory[0..self.memory_len];
        allocator.free(buff);
        allocator.destroy(self);
    }
};
