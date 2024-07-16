const std = @import("std");
const cpu = @import("cpu.zig");
const expect = std.testing.expect;
const expectEquals = std.testing.expectEqual;

pub const TestIO = struct {
    memory: [64 * 1024]u8,
    ports: [256]u8,

    const Self = @This();

    pub fn init() !*Self {
        const s = try std.testing.allocator.create(Self);
        s.memory = std.mem.zeroes([64 * 1024]u8);
        s.ports = std.mem.zeroes([256]u8);
        return s;
    }

    pub fn deinit(self: *Self) void {
        std.testing.allocator.destroy(self);
    }

    pub fn read(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        return self.memory[address];
    }

    pub fn write(ctx: *anyopaque, address: u16, value: u8) void {
        var self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        self.memory[address] = value;
    }

    pub fn ioRead(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);
        return self.ports[port];
    }

    pub fn ioWrite(ctx: *const anyopaque, address: u16, value: u8) void {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);
        self.ports[port] = value;
    }
};

var test_io = TestIO{
    .memory = std.mem.zeroes([64 * 1024]u8),
    .ports = std.mem.zeroes([256]u8),
};

test "test IO" {
    var io = try TestIO.init();
    defer io.deinit();

    var s = try cpu.Z80State.init(
        std.heap.page_allocator,
        io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );

    io.memory[1234] = 124;

    const m = io.memory[1234];

    try expectEquals(124, m);

    const p = s.memRead(1234);

    try expectEquals(124, p);
}

test "xz" {
    var s = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    s.BC.high = 10;
    s.BC.low = 11;
    s.DE.high = 12;
    s.DE.low = 13;
    s.IX.high = 14;
    s.IX.low = 15;
    s.AF.A = 16;
    s.addr_register = &s.IX;

    std.debug.print("l={d} h={d}\n", .{ s.IX.low, s.IX.high });

    for (s.p, 0..) |p, i| {
        if (i == 6) continue;
        std.debug.print("[{d}] : {d}\n", .{ i, p.* });
    }
}

test "16 bit registers set/get" {
    var r = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );

    var i: u16 = 0;
    while (i < std.math.maxInt(u16)) : (i += 1) {
        // BC
        r.BC.setValue(i);
        try expect(r.BC.getValue() == i);

        // DE
        r.DE.setValue(i);
        try expect(r.DE.getValue() == i);

        // HL
        r.HL.setValue(i);
        try expect(r.HL.getValue() == i);

        // AF
        r.AF.setValue(i);
        try expect(r.AF.getValue() == i);
    }
}

test "Flags register set/get" {
    var r = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    if (false) {
        r.AF.setFlags(0b00000000);
        try expect(r.AF.F.zero == false);
        try expect(r.AF.F.sub == false);
        try expect(r.AF.F.hcarry == false);
        try expect(r.AF.F.getCarry == false);

        r.AF.setFlags(0b01110000);
        try expect(r.AF.F.zero == false);
        try expect(r.AF.F.sub == true);
        try expect(r.AF.F.hcarry == true);
        try expect(r.AF.F.getCarry == true);

        r.AF.setFlags(0b00110000);
        try expect(r.AF.F.zero == false);
        try expect(r.AF.F.sub == false);
        try expect(r.AF.F.hcarry == true);
        try expect(r.AF.F.getCarry == true);

        r.AF.setFlags(0b00010000);
        try expect(r.AF.F.zero == false);
        try expect(r.AF.F.sub == false);
        try expect(r.AF.F.hcarry == false);
        try expect(r.AF.F.getCarry == true);

        r.AF.setFlags(0b11110000);
        try expect(r.AF.F.zero == true);
        try expect(r.AF.F.sub == true);
        try expect(r.AF.F.hcarry == true);
        try expect(r.AF.F.getCarry == true);
    }
}

test "16 bit register inc/dec" {
    for (0..65536) |i| {
        const value: u16 = @truncate(i);
        var r = cpu.SixteenBitRegister{};
        r.setValue(value);
        try expectEquals(value, r.getValue());
        r.increment();
        try expectEquals(value +% 1, r.getValue());
        r.setValue(value);
        r.decrement();
        try expectEquals(value -% 1, r.getValue());
    }
}

test "16 bit register" {
    var r = cpu.SixteenBitRegister{
        .high = 0,
        .low = 0,
    };

    try expect(r.high == 0);
    try expect(r.low == 0);

    var i: u8 = 0;

    while (i < 255) : (i += 1) {
        r.setHigh(i);
        try expect(r.high == i);
        try expect(r.low == 0);
    }

    r.high = 0;

    i = 0;
    while (i < 255) : (i += 1) {
        r.setLow(i);
        try expect(r.low == i);
        try expect(r.high == 0);
    }

    const expected: u16 = std.math.maxInt(u16);
    r.setValue(expected);
    const value = r.getValue();
    try expect(value == expected);

    const expected2: u16 = 32768;
    r.setValue(expected2);
    const value2 = r.getValue();
    try expect(value2 == expected2);
}

test "BCD product code decode" {
    const parseBCDProdcutCode = @import("cartridge.zig").parseBCDProductCode;

    const expected = 107026;
    const bcd_input = [3]u8{ 0x26, 0x70, 0xa0 };
    const value = parseBCDProdcutCode(bcd_input[0..3]);
    try expect(value == expected);
}

test "load rom" {
    if (false) {
        const cartridge = @import("cartridge.zig");

        const rom = try cartridge.loadRomAbsolute(
            "rom.sms",
        );

        defer rom.close();

        if (rom.sega_header) |h| {
            const tmr_sega: []const u8 = h.raw_header[0..8];
            try expect(std.mem.eql(u8, tmr_sega, "TMR SEGA"));

            const rom_size_kb = cartridge.getRomSizeKB(h.rom_size);
            const product_name = cartridge.tryGetCartridgeName(h.product_code);

            std.debug.print("\nCartridge:\n", .{});
            std.debug.print("    Size: {d} bytes - {d}KB (0x{x})\n", .{ rom.size, rom_size_kb, h.rom_size });
            std.debug.print("    SegaHeader: {s}\n", .{tmr_sega});
            std.debug.print("    Checksum: rom=0x{x} calc=0x{x}\n", .{ h.checksum, h.checksum_calc });
            std.debug.print("    Product Code: {d} ({s})\n", .{ h.product_code, product_name });
            std.debug.print("    System/Region: {any}\n", .{h.system_region});
            std.debug.print("    Version: {d}\n", .{h.version});
        }

        if (rom.sdsc_header) |sdsc| {
            std.debug.print("    SDSC Version: {d}.{d}\n", .{ sdsc.version.major, sdsc.version.minor });
            std.debug.print("    Author: '{s}'\n", .{sdsc.author_name});
            std.debug.print("    Title: '{s}'\n", .{sdsc.program_name});
            std.debug.print("    Description: '{s}'\n", .{sdsc.description});
            std.debug.print("    Release Date: {d}/{d}/{d}\n", .{
                sdsc.release_date.day,
                sdsc.release_date.month,
                sdsc.release_date.year,
            });
        }
    }
}

test "register array access" {
    var state = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    const expected = [8]u8{ 10, 11, 20, 21, 30, 31, 255, 41 };

    state.BC.high = expected[@intFromEnum(cpu.EightBitRegisters.B)];
    state.BC.low = expected[@intFromEnum(cpu.EightBitRegisters.C)];
    state.DE.high = expected[@intFromEnum(cpu.EightBitRegisters.D)];
    state.DE.low = expected[@intFromEnum(cpu.EightBitRegisters.E)];
    state.HL.high = expected[@intFromEnum(cpu.EightBitRegisters.H)];
    state.HL.low = expected[@intFromEnum(cpu.EightBitRegisters.L)];
    state.AF.A = expected[@intFromEnum(cpu.EightBitRegisters.A)];
    state.AF.F = @bitCast(expected[@intFromEnum(cpu.EightBitRegisters.F)]);

    std.debug.print("\n", .{});

    inline for (state.gp_registers[0..8], 0..) |b, i| {
        const r: cpu.EightBitRegisters = @enumFromInt(i);
        std.debug.print("{d} {any}: {d}\n", .{ i, r, b });
        try expectEquals(expected[i], b);
    }
}

test "register pairs array access" {
    var state = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    const expected = [4]u16{ 8192, 16384, 32768, 65535 };

    state.BC.setValue(expected[@intFromEnum(cpu.RegisterPairs2.BC)]);
    state.DE.setValue(expected[@intFromEnum(cpu.RegisterPairs2.DE)]);
    state.HL.setValue(expected[@intFromEnum(cpu.RegisterPairs2.HL)]);
    state.AF.setValue(expected[@intFromEnum(cpu.RegisterPairs2.AF)]);

    std.debug.print("\n", .{});

    inline for (state.gp_registers_pairs[0..4], 0..) |rp, i| {
        const r: cpu.RegisterPairs1 = @enumFromInt(i);
        std.debug.print("{d}: {any}\n", .{ i, r });
        const expected_value: u16 = expected[i];
        try expectEquals(expected_value, rp.getValue());
    }
}

test "ADD A,r" {
    const op = @import("opcodes.zig");
    var state = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    state.AF.A = 0x44;
    _ = op.add_a_x(state, 0x11);
    try expect(state.AF.A == 0x55);
    try expect(state.AF.getFlags() == 0b00000000);
}

test "opcode xyz decode" {
    const op = @import("opcodes.zig");

    // ADD A,A
    const opcode_87: u8 = 0x87;
    const ins_87: op.OpCode = @bitCast(opcode_87);
    try expectEquals(2, ins_87.x); // Arithmetic/logic operations table (alu)
    try expectEquals(0, ins_87.y); // ADD A,A
    const r87: cpu.EightBitRegisters = @enumFromInt(ins_87.z);
    try expectEquals(cpu.EightBitRegisters.A, r87); // r == A

    // ADD A,B
    const opcode_80: u8 = 0x80;
    const ins_80: op.OpCode = @bitCast(opcode_80);
    try expectEquals(2, ins_80.x); // Arithmetic/logic operations table (alu)
    try expectEquals(0, ins_80.y); // ADD A,B
    const r80: cpu.EightBitRegisters = @enumFromInt(ins_80.z);
    try expectEquals(cpu.EightBitRegisters.B, r80); // r == B

    // ADD ADC A,r
    const opcode_89: u8 = 0x89;
    const ins_89: op.OpCode = @bitCast(opcode_89);
    try expectEquals(2, ins_89.x); // Arithmetic/logic operations table (alu)
    try expectEquals(1, ins_89.y); // ADC A,C
    const r89: cpu.EightBitRegisters = @enumFromInt(ins_89.z);
    try expectEquals(cpu.EightBitRegisters.C, r89); // r == B
}

test "FD-DD sequence" {
    const op = @import("opcodes.zig");

    var s = try cpu.Z80State.init(
        std.heap.page_allocator,
        &test_io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    defer s.deinit(std.heap.page_allocator);

    s.PC = 0;

    const seq: [6]u8 = [_]u8{ 0xdd, 0xdd, 0xfd, 0xdd, 0xfd, 0x42 };

    for (seq, 0..) |opcode, i| {
        test_io.memory[i] = opcode;
    }

    const final = op.consumeXYSequence(s);

    try expectEquals(0xfd, final);
    try expectEquals(0x42, test_io.memory[s.PC +% 1]);
}
