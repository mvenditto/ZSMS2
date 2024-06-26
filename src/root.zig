const std = @import("std");
const cpu = @import("cpu.zig");
const expect = std.testing.expect;
const expectEquals = std.testing.expectEqual;

const OpcodeTest = struct {
    name: []const u8,
    initial: struct { pc: u16, sp: u16, a: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, f: u8, i: u8, r: u8, ix: u16, iy: u16, ram: []const [2]u16 },
    final: struct { pc: u16, sp: u16, a: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, f: u8, i: u8, r: u8, ix: u16, iy: u16, ram: []const [2]u16 },

    fn toZ80State(self: *const OpcodeTest) !*cpu.Z80State {
        var s = try cpu.Z80State.create(std.heap.page_allocator);
        const init = self.initial;
        s.AF.A = init.a;
        s.AF.setFlags(init.f);
        s.BC.high = init.b;
        s.BC.low = init.c;
        s.DE.high = init.d;
        s.DE.low = init.e;
        s.HL.high = init.h;
        s.HL.low = init.l;
        s.SP = init.sp;
        s.PC = init.pc;
        s.I = init.i;
        s.IX.setValue(init.ix);
        s.IY.setValue(init.iy);
        s.R = init.r;

        for (init.ram) |loc| {
            s.memory[loc[0]] = @truncate(loc[1]);
        }

        return s;
    }

    fn expectState(self: *const OpcodeTest, s: *const cpu.Z80State) !void {
        const final = self.final;

        expectEquals(final.f, s.AF.getFlags()) catch |err| {
            const exp_flags: cpu.FlagsRegister = @bitCast(final.f);
            std.debug.print("Flags mismatch:\n  expected 0b{b:0>8} {any},\n  found    0b{b:0>8} {any}\n", .{
                final.f,         exp_flags,
                s.AF.getFlags(), s.AF.F,
            });
            std.debug.print("Context:\n  initial {any},\n  final {any}\n", .{ self.initial, self.final });
            return err;
        };

        try expectEquals(final.a, s.AF.A);
        try expectEquals(final.b, s.BC.high);
        try expectEquals(final.c, s.BC.low);
        try expectEquals(final.d, s.DE.high);
        try expectEquals(final.e, s.DE.low);
        try expectEquals(final.h, s.HL.high);
        try expectEquals(final.l, s.HL.low);
        try expectEquals(final.sp, s.SP);
        try expectEquals(final.pc, s.PC);
        try expectEquals(final.i, s.I);
        try expectEquals(final.ix, s.IX.getValue());
        try expectEquals(final.iy, s.IY.getValue());
        // try expectEquals(final.r, s.R);

        for (final.ram) |loc| {
            const address: u16 = @truncate(loc[0]);
            const value: u8 = @truncate(loc[1]);
            try expectEquals(value, s.memory[address]);
        }
    }
};

pub fn printError(
    trace: ?*std.builtin.StackTrace,
    comptime format: []const u8,
    args: anytype,
) void {
    @setCold(true);

    const size = 0x1000;
    const trunc_msg = "(msg truncated)";
    var buf: [size + trunc_msg.len]u8 = undefined;

    const msg = std.fmt.bufPrint(buf[0..size], format, args) catch |err| switch (err) {
        error.NoSpaceLeft => blk: {
            @memcpy(buf[size..], trunc_msg);
            break :blk &buf;
        },
    };
    std.debug.print("{s}:\nTRACE:\n{any}\n", .{ msg, trace });
}

test "dasdsa" {
    var x = cpu.SixteenBitRegister{};
    x.setValue(8192);
    std.debug.print("\n\nhigh=0x{x}, low=0x{x}, value=0x{x} ({d})\n\n", .{ x.high, x.low, x.getValue(), x.getValue() });
}

test "16 bit registers set/get" {
    var r = try cpu.Z80State.create(std.heap.page_allocator);

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
    var r = try cpu.Z80State.create(std.heap.page_allocator);
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
    var state = try cpu.Z80State.create(std.heap.page_allocator);
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
    var state = try cpu.Z80State.create(std.heap.page_allocator);
    const expected = [4]u16{ 8192, 16384, 32768, 65535 };

    state.BC.setValue(expected[@intFromEnum(cpu.RegisterPairs2.BC)]);
    state.DE.setValue(expected[@intFromEnum(cpu.RegisterPairs2.DE)]);
    state.HL.setValue(expected[@intFromEnum(cpu.RegisterPairs2.HL)]);
    state.AF.setValue(expected[@intFromEnum(cpu.RegisterPairs2.AF)]);

    std.debug.print("\n", .{});

    const host_endianess = @import("builtin").target.cpu.arch.endian();

    inline for (state.gp_registers_pairs[0..4], 0..) |b, i| {
        const r: cpu.EightBitRegisters = @enumFromInt(i);
        std.debug.print("{d} {any}: {d}\n", .{ i, r, b });
        const expected_value: u16 = switch (host_endianess) {
            .little => @byteSwap(expected[i]),
            .big => expected[i],
        };
        try expectEquals(expected_value, b);
    }
}

test "ADD A,r" {
    const op = @import("opcodes.zig");
    var state = try cpu.Z80State.create(std.heap.page_allocator);
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

fn parseZ80TestFile(file_path: []const u8) ![]OpcodeTest {
    const file = try std.fs.openFileAbsolute(
        file_path,
        .{
            .mode = std.fs.File.OpenMode.read_only,
        },
    );

    const json = try file.readToEndAlloc(std.heap.page_allocator, 2_000_000);

    const parsed = try std.json.parseFromSlice(
        []OpcodeTest,
        std.heap.page_allocator,
        json,
        .{ .ignore_unknown_fields = true },
    );

    const tests = parsed.value;

    return tests;
}

fn getFileName(file_path: []const u8) []const u8 {
    var split_iter = std.mem.splitBackwardsSequence(
        u8,
        file_path,
        std.fs.path.sep_str,
    );

    const filename = split_iter.next().?;

    var split_iter2 = std.mem.splitSequence(u8, filename, ".");

    const file_name_no_ext = split_iter2.next().?;

    return file_name_no_ext;
}

test "SingleStepTests/z80" {
    const op = @import("opcodes.zig");

    const opcodes_to_test = [_][]const u8{
        "87", "80", "81", "82", "83", "84", "85", "86", "dd 86", "fd 86", "c6", // ADD A,
        "8F", "88", "89", "8A", "8B", "8C", "8D", "8E", "dd 8E", "fd 8E", "ce", // ADC A,
        "97", "90", "91", "92", "93", "94", "95", "96", "dd 8E", "fd 8E", "d6", // SUB A,
        "9F", "98", "99", "9A", "9B", "9C", "9D", "9E", "dd 9E", "fd 9E", "de", // SBC A,
        "A7", "A0", "A1", "A2", "A3", "A4", "A5", "A6", "dd A6", "fd A6", "e6", // AND A,
        "AF", "A8", "A9", "AA", "AB", "AC", "AD", "AE", "dd AE", "fd AE", "ee", // XOR A,
        "B7", "B0", "B1", "B2", "B3", "B4", "B5", "B6", "dd B6", "fd B6", "f6", // OR A,
        "BF", "B8", "B9", "BA", "BB", "BC", "BD", "BE", "dd BE", "fd BE", "fe", // CP A,
        "3C", "04", "0C", "14", "1C", "24", "2C", "34", "dd 34", "fd 34", // INC A,
        "3D", "05", "0D", "15", "1D", "25", "2D", "35", "dd 35", "fd 35", // DEC A,
    };

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    const tests_base_path = try std.fs.cwd().realpathAlloc(
        allocator,
        ".\\tests\\SingleStepTests\\z80\\v1",
    );

    var failed: u32 = 0;

    std.debug.print("\nSingleStepTests:\n", .{});

    for (opcodes_to_test) |test_file_name| {
        const file_path = try std.mem.concat(
            allocator,
            u8,
            &[_][]const u8{ tests_base_path, "\\", test_file_name, ".json" },
        );

        const tests = try parseZ80TestFile(file_path);

        for (tests[1..], 0..) |t, i| {
            const s = try t.toZ80State();

            const opcode = s.fetchAtPC();
            op.exec(s, opcode);

            t.expectState(s) catch |err| {
                printError(
                    @errorReturnTrace(),
                    "[{d}] {s}: FAIL",
                    .{ i, t.name },
                );
                failed += 1;
                return err;
            };
            // std.debug.print("[{d}] {s}: OK\n", .{ i, t.name });
        }

        std.debug.print("   [{s}] - {d}, failed {d} ({d}/1000)\n", .{ test_file_name, 1000 - failed, failed, 1000 - failed });

        if (failed > 0) {
            return error.TestUnexpectedValue;
        }
    }
}
