const std = @import("std");
const cpu = @import("cpu.zig");
const expect = std.testing.expect;
const expectEquals = std.testing.expectEqual;

const CycleSample = struct {
    addr_pins: ?u16 = null,
    data_pins: ?u8 = null,
    reqs_pins: [4]u8 = [4]u8{ "-", "-", "-", "-" },
};

const OpcodeTest = struct {
    name: []const u8,
    initial: struct { pc: u16, sp: u16, a: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, f: u8, wz: u16, af_: u16, bc_: u16, de_: u16, hl_: u16, i: u8, r: u8, ix: u16, iy: u16, iff1: u1, iff2: u1, ram: []const [2]u16 },
    final: struct { pc: u16, sp: u16, a: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, f: u8, wz: u16, af_: u16, bc_: u16, de_: u16, hl_: u16, i: u8, r: u8, ix: u16, iy: u16, iff1: u1, iff2: u1, ram: []const [2]u16 },
    cycles: []const [3]std.json.Value,
    fn toZ80State(self: *const OpcodeTest, allocator: std.mem.Allocator) !*cpu.Z80State {
        var s = try cpu.Z80State.init(allocator);
        const init = self.initial;
        s.AF.A = init.a;
        s.AF.setFlags(init.f);
        s.BC.high = init.b;
        s.BC.low = init.c;
        s.DE.high = init.d;
        s.DE.low = init.e;
        s.HL.high = init.h;
        s.HL.low = init.l;
        s.SP.setValue(init.sp);
        s.PC = init.pc;
        s.I = init.i;
        s.IX.setValue(init.ix);
        s.IY.setValue(init.iy);
        s.R.setValue(init.r);
        s.IFF1 = init.iff1 == 1;
        s.IFF2 = init.iff2 == 1;
        s.WZ.setValue(init.wz);
        s.AF_.setValue(init.af_);
        s.BC_.setValue(init.bc_);
        s.DE_.setValue(init.de_);
        s.HL_.setValue(init.hl_);

        for (init.ram) |loc| {
            s.memory[loc[0]] = @truncate(loc[1]);
        }

        return s;
    }

    fn expectState(self: *const OpcodeTest, s: *const cpu.Z80State) !void {
        const final = self.final;

        try expectEquals(final.a, s.AF.A);
        try expectEquals(final.b, s.BC.high);
        try expectEquals(final.c, s.BC.low);
        try expectEquals(final.d, s.DE.high);
        try expectEquals(final.e, s.DE.low);
        try expectEquals(final.h, s.HL.high);
        try expectEquals(final.l, s.HL.low);
        try expectEquals(final.sp, s.SP.getValue());
        try expectEquals(final.pc, s.PC);

        expectEquals(final.f, s.AF.getFlags()) catch |err| {
            const exp_flags: cpu.FlagsRegister = @bitCast(final.f);
            std.debug.print("Flags mismatch:\n  expected 0b{b:0>8} {any},\n  found    0b{b:0>8} {any}\n", .{
                final.f,         exp_flags,
                s.AF.getFlags(), s.AF.F,
            });
            std.debug.print("Context:\n  initial {any},\n  final {any}\n", .{ self.initial, self.final });
            return err;
        };

        try expectEquals(final.ix, s.IX.getValue());
        try expectEquals(final.iy, s.IY.getValue());
        try expectEquals(final.i, s.I);
        try expectEquals(final.iff1 == 1, s.IFF1);
        try expectEquals(final.iff2 == 1, s.IFF2);
        try expectEquals(final.r, s.R.getValue());
        try expectEquals(self.cycles.len, s.cycles);

        try expectEquals(final.af_, s.AF_.getValue());
        try expectEquals(final.bc_, s.BC_.getValue());
        try expectEquals(final.de_, s.DE_.getValue());
        try expectEquals(final.hl_, s.HL_.getValue());
        try expectEquals(final.wz, s.WZ.getValue());

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

test "mem swap" {
    var s = try cpu.Z80State.init(std.heap.page_allocator);
    defer s.deinit(std.heap.page_allocator);
    s.memory[0] = 42;
    s.memory[1000] = 55;
    try expectEquals(42, s.memory[0]);
    try expectEquals(55, s.memory[1000]);
    std.mem.swap(u8, &s.memory[0], &s.memory[1000]);
    try expectEquals(55, s.memory[0]);
    try expectEquals(42, s.memory[1000]);
}

test "16 bit registers set/get" {
    var r = try cpu.Z80State.init(std.heap.page_allocator);

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
    var r = try cpu.Z80State.init(std.heap.page_allocator);
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
    var state = try cpu.Z80State.init(std.heap.page_allocator);
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
    var state = try cpu.Z80State.init(std.heap.page_allocator);
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
    var state = try cpu.Z80State.init(std.heap.page_allocator);
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

    const json = try file.readToEndAlloc(std.heap.page_allocator, 10_000_000);

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
        // 8-bit Arithment and Logical group
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
        // Load group
        "ed 57", "ed 5F", "7F", "78", "79", "7A", "7B", "dd 7e", "fd 7e", "7C", "7D", "7E", "0A", "1A", "3A", "3E", // LD A,r + LD A,(HL|BC|DE)
        "47", "40", "41", "42", "43", "44", "45", "46", "dd 46", "fd 46", "06", // LD B,r
        "4F", "48", "49", "4A", "4B", "4C", "4D", "4E", "dd 4e", "fd 4e", "0e", // LD C,r
        "57", "50", "51", "52", "53", "54", "55", "56", "dd 56", "fd 56", "16", // LD D,r
        "5F", "58", "59", "5A", "5B", "5C", "5D", "5E", "dd 5e", "fd 5e", "1e", // LD E,r
        "67", "60", "61", "62", "63", "64", "65", "66", "dd 66", "fd 66", "26", // LD H,r
        "6F", "68", "69", "6A", "6B", "6C", "6D", "6E", "dd 6e", "fd 6e", "2e", // LD L,r
        "ed 47", "ed 4f", "32", // LD I,A - LD R,A
        "77", "70", "71", "72", "73", "74", "75", "36", // LD (HL),r
        "02", "12", // LD (BC|DE),A
        "dd 77", "dd 70", "dd 71", "dd 72", "dd 73", "dd 74", "dd 75", "dd 36", "fd 36", // LD(IX|IY+d),r
        // Block Transfer Group
        "ED A0", "ED B0", "ED A8", "ED B8", // LDI,LDIR,LDIR,LDDR
        // Block Search Group
        "ED A1", "ED B1", "ED A9", "ED B9", // CPI,CPIR,CPIR,CPDR
        // Block Exchange Group
        "08", "D9", "EB", "E3", "dd E3", "fd E3", // EX and EXX
        // 16-bit Arithment and Logical group
        "09", "19", "29", "39", // ADD HL,
        "dd 09", "dd 19", "dd 29", "dd 39", // ADD IX,
        "fd 09", "fd 19", "fd 29", "fd 39", // ADD IY,
        "ed 4a", "ed 5a", "ed 6a", "ed 7a", // ADC HL,
        "ed 42", "ed 52", "ed 62", "ed 72", // SBC HL,
        "03", "13", "23", "33", "dd 23", "fd 23", // INC
        "0b", "1b", "2b", "3b", "dd 2b", "fd 2b", // DEC
        // Rotate and Shift group
        "cb 07", "cb 00", "cb 01", "cb 02", "cb 03", "cb 04", "cb 05", // RLC
        "cb 0F", "cb 08", "cb 08", "cb 0A", "cb 0B", "cb 0C", "cb 0D", // RRC
        "cb 17", "cb 10", "cb 11", "cb 12", "cb 13", "cb 14", "cb 15", // RL
        "cb 1F", "cb 18", "cb 18", "cb 1A", "cb 1B", "cb 1C", "cb 1D", // RR
        "cb 20", "cb 21", "cb 22", "cb 23", "cb 24", "cb 25", "cb 27", // SLA
        "cb 28", "cb 29", "cb 2A", "cb 2B", "cb 2C", "cb 2D", "cb 2F", // SRA
        "cb 3f", "cb 38", "cb 39", "cb 3A", "cb 3B", "cb 3C", "cb 3D", // SRL
        "cb 37", "cb 30", "cb 31", "cb 31", "cb 33", "cb 34", "cb 35", // SLL
        "07", "0f", "17", "1f", // RLCA, RRCA, RLA, RRA
        // Bit manipulation group BIT r,b SET r,b RES r,b
        "cb 40", "cb 41", "cb 42", "cb 43", "cb 44", "cb 45", "cb 47", "cb 48", "cb 49", "cb 4a", "cb 4b", "cb 4c", "cb 4d", "cb 4e", "cb 4f", //
        "cb 50", "cb 51", "cb 52", "cb 53", "cb 54", "cb 55", "cb 57", "cb 58", "cb 59", "cb 5a", "cb 5b", "cb 5c", "cb 5d", "cb 5e", "cb 5f",
        "cb 60", "cb 61", "cb 62", "cb 63", "cb 64", "cb 65", "cb 67", "cb 68", "cb 69", "cb 6a", "cb 6b", "cb 6c", "cb 6d", "cb 6e", "cb 6f",
        "cb 70", "cb 71", "cb 72", "cb 73", "cb 74", "cb 75", "cb 77", "cb 78", "cb 79", "cb 7a", "cb 7b", "cb 7c", "cb 7d", "cb 7e", "cb 7f",
        "cb 80", "cb 81", "cb 82", "cb 83", "cb 84", "cb 85", "cb 87", "cb 88", "cb 89", "cb 8a", "cb 8b", "cb 8c", "cb 8d", "cb 8e", "cb 8f",
        "cb 90", "cb 91", "cb 92", "cb 93", "cb 94", "cb 95", "cb 97", "cb 98", "cb 99", "cb 9a", "cb 9b", "cb 9c", "cb 9d", "cb 9e", "cb 9f",
        "cb a0", "cb a1", "cb a2", "cb a3", "cb a4", "cb a5", "cb a7", "cb a8", "cb a9", "cb aa", "cb ab", "cb ac", "cb ad", "cb ae", "cb af",
        "cb b0", "cb b1", "cb b2", "cb b3", "cb b4", "cb b5", "cb b7", "cb b8", "cb b9", "cb ba", "cb bb", "cb bc", "cb bd", "cb be", "cb bf",
        "cb c0", "cb c1", "cb c2", "cb c3", "cb c4", "cb c5", "cb c7", "cb c8", "cb c9", "cb ca", "cb cb", "cb cc", "cb cd", "cb ce", "cb cf",
        "cb d0", "cb d1", "cb d2", "cb d3", "cb d4", "cb d5", "cb d7", "cb d8", "cb d9", "cb da", "cb db", "cb dc", "cb dd", "cb de", "cb df",
        "cb e0", "cb e1", "cb e2", "cb e3", "cb e4", "cb e5", "cb e7", "cb e8", "cb e9", "cb ea", "cb eb", "cb ec", "cb ed", "cb ee", "cb ef",
        "cb f0", "cb f1", "cb f2", "cb f3", "cb f4", "cb f5", "cb f7", "cb f8", "cb f9", "cb fa", "cb fb", "cb fc", "cb fd", "cb fe", "cb ff",
        "cb 06",       "cb 16",       "cb 26",       "cb 36",       "cb 0e",       "cb 1e",       "cb 2e",       "cb 3e", // (HL)
        // DDCB - rotate/shift
        "dd cb __ 00", "dd cb __ 01", "dd cb __ 02", "dd cb __ 03", "dd cb __ 04", "dd cb __ 05", "dd cb __ 06", "dd cb __ 07",
        "dd cb __ 08", "dd cb __ 09", "dd cb __ 0a", "dd cb __ 0b", "dd cb __ 0c", "dd cb __ 0d", "dd cb __ 0e", "dd cb __ 0f",
        "dd cb __ 10", "dd cb __ 11", "dd cb __ 12", "dd cb __ 13", "dd cb __ 14", "dd cb __ 15", "dd cb __ 16", "dd cb __ 17",
        "dd cb __ 18", "dd cb __ 19", "dd cb __ 1a", "dd cb __ 1b", "dd cb __ 1c", "dd cb __ 1d", "dd cb __ 1e", "dd cb __ 1f",
        "dd cb __ 20", "dd cb __ 21", "dd cb __ 22", "dd cb __ 23", "dd cb __ 24", "dd cb __ 25", "dd cb __ 26", "dd cb __ 27",
        "dd cb __ 28", "dd cb __ 29", "dd cb __ 2a", "dd cb __ 2b", "dd cb __ 2c", "dd cb __ 2d", "dd cb __ 2e", "dd cb __ 2f",
        "dd cb __ 30", "dd cb __ 31", "dd cb __ 32", "dd cb __ 33", "dd cb __ 34", "dd cb __ 35", "dd cb __ 36", "dd cb __ 37",
        "dd cb __ 38", "dd cb __ 39", "dd cb __ 3a", "dd cb __ 3b", "dd cb __ 3c", "dd cb __ 3d", "dd cb __ 3e", "dd cb __ 3f",
        // FDCB rotate/shift
        "fd cb __ 00", "fd cb __ 01", "fd cb __ 02", "fd cb __ 03", "fd cb __ 04", "fd cb __ 05", "fd cb __ 06", "fd cb __ 07",
        "fd cb __ 08", "fd cb __ 09", "fd cb __ 0a", "fd cb __ 0b", "fd cb __ 0c", "fd cb __ 0d", "fd cb __ 0e", "fd cb __ 0f",
        "fd cb __ 10", "fd cb __ 11", "fd cb __ 12", "fd cb __ 13", "fd cb __ 14", "fd cb __ 15", "fd cb __ 16", "fd cb __ 17",
        "fd cb __ 18", "fd cb __ 19", "fd cb __ 1a", "fd cb __ 1b", "fd cb __ 1c", "fd cb __ 1d", "fd cb __ 1e", "fd cb __ 1f",
        "fd cb __ 20", "fd cb __ 21", "fd cb __ 22", "fd cb __ 23", "fd cb __ 24", "fd cb __ 25", "fd cb __ 26", "fd cb __ 27",
        "fd cb __ 28", "fd cb __ 29", "fd cb __ 2a", "fd cb __ 2b", "fd cb __ 2c", "fd cb __ 2d", "fd cb __ 2e", "fd cb __ 2f",
        "fd cb __ 30", "fd cb __ 31", "fd cb __ 32", "fd cb __ 33", "fd cb __ 34", "fd cb __ 35", "fd cb __ 36", "fd cb __ 37",
        "fd cb __ 38", "fd cb __ 39", "fd cb __ 3a", "fd cb __ 3b", "fd cb __ 3c", "fd cb __ 3d", "fd cb __ 3e", "fd cb __ 3f",
        // DDCB bit manipulation
        "dd cb __ 40", "dd cb __ 41", "dd cb __ 42", "dd cb __ 43", "dd cb __ 44", "dd cb __ 45", "dd cb __ 46", "dd cb __ 47",
        "dd cb __ 48", "dd cb __ 49", "dd cb __ 4a", "dd cb __ 4b", "dd cb __ 4c", "dd cb __ 4d", "dd cb __ 4e", "dd cb __ 4f",
        "dd cb __ 50", "dd cb __ 51", "dd cb __ 52", "dd cb __ 53", "dd cb __ 54", "dd cb __ 55", "dd cb __ 56", "dd cb __ 57",
        "dd cb __ 58", "dd cb __ 59", "dd cb __ 5a", "dd cb __ 5b", "dd cb __ 5c", "dd cb __ 5d", "dd cb __ 5e", "dd cb __ 5f",
        "dd cb __ 60", "dd cb __ 61", "dd cb __ 62", "dd cb __ 63", "dd cb __ 64", "dd cb __ 65", "dd cb __ 66", "dd cb __ 67",
        "dd cb __ 68", "dd cb __ 69", "dd cb __ 6a", "dd cb __ 6b", "dd cb __ 6c", "dd cb __ 6d", "dd cb __ 6e", "dd cb __ 6f",
        "dd cb __ 70", "dd cb __ 71", "dd cb __ 72", "dd cb __ 73", "dd cb __ 74", "dd cb __ 75", "dd cb __ 76", "dd cb __ 77",
        "dd cb __ 78", "dd cb __ 79", "dd cb __ 7a", "dd cb __ 7b", "dd cb __ 7c", "dd cb __ 7d", "dd cb __ 7e", "dd cb __ 7f",
        "dd cb __ 80", "dd cb __ 81", "dd cb __ 82", "dd cb __ 83", "dd cb __ 84", "dd cb __ 85", "dd cb __ 86", "dd cb __ 87",
        "dd cb __ 88", "dd cb __ 89", "dd cb __ 8a", "dd cb __ 8b", "dd cb __ 8c", "dd cb __ 8d", "dd cb __ 8e", "dd cb __ 8f",
        "dd cb __ 90", "dd cb __ 91", "dd cb __ 92", "dd cb __ 93", "dd cb __ 94", "dd cb __ 95", "dd cb __ 96", "dd cb __ 97",
        "dd cb __ 98", "dd cb __ 99", "dd cb __ 9a", "dd cb __ 9b", "dd cb __ 9c", "dd cb __ 9d", "dd cb __ 9e", "dd cb __ 9f",
        "dd cb __ a0", "dd cb __ a1", "dd cb __ a2", "dd cb __ a3", "dd cb __ a4", "dd cb __ a5", "dd cb __ a6", "dd cb __ a7",
        "dd cb __ a8", "dd cb __ a9", "dd cb __ aa", "dd cb __ ab", "dd cb __ ac", "dd cb __ ad", "dd cb __ ae", "dd cb __ af",
        "dd cb __ b0", "dd cb __ b1", "dd cb __ b2", "dd cb __ b3", "dd cb __ b4", "dd cb __ b5", "dd cb __ b6", "dd cb __ b7",
        "dd cb __ b8", "dd cb __ b9", "dd cb __ ba", "dd cb __ bb", "dd cb __ bc", "dd cb __ bd", "dd cb __ be", "dd cb __ bf",
        "dd cb __ c0", "dd cb __ c1", "dd cb __ c2", "dd cb __ c3", "dd cb __ c4", "dd cb __ c5", "dd cb __ c6", "dd cb __ c7",
        "dd cb __ c8", "dd cb __ c9", "dd cb __ ca", "dd cb __ cb", "dd cb __ cc", "dd cb __ cd", "dd cb __ ce", "dd cb __ cf",
        "dd cb __ d0", "dd cb __ d1", "dd cb __ d2", "dd cb __ d3", "dd cb __ d4", "dd cb __ d5", "dd cb __ d6", "dd cb __ d7",
        "dd cb __ d8", "dd cb __ d9", "dd cb __ da", "dd cb __ db", "dd cb __ dc", "dd cb __ dd", "dd cb __ de", "dd cb __ df",
        "dd cb __ e0", "dd cb __ e1", "dd cb __ e2", "dd cb __ e3", "dd cb __ e4", "dd cb __ e5", "dd cb __ e6", "dd cb __ e7",
        "dd cb __ e8", "dd cb __ e9", "dd cb __ ea", "dd cb __ eb", "dd cb __ ec", "dd cb __ ed", "dd cb __ ee", "dd cb __ ef",
        "dd cb __ f0", "dd cb __ f1", "dd cb __ f2", "dd cb __ f3", "dd cb __ f4", "dd cb __ f5", "dd cb __ f6", "dd cb __ f7",
        "dd cb __ f8", "dd cb __ f9", "dd cb __ fa", "dd cb __ fb", "dd cb __ fc", "dd cb __ fd", "dd cb __ fe", "dd cb __ ff",
        // FDCB bit manipulation
        "fd cb __ 40", "fd cb __ 41", "fd cb __ 42", "fd cb __ 43", "fd cb __ 44", "fd cb __ 45", "fd cb __ 46", "fd cb __ 47",
        "fd cb __ 48", "fd cb __ 49", "fd cb __ 4a", "fd cb __ 4b", "fd cb __ 4c", "fd cb __ 4d", "fd cb __ 4e", "fd cb __ 4f",
        "fd cb __ 50", "fd cb __ 51", "fd cb __ 52", "fd cb __ 53", "fd cb __ 54", "fd cb __ 55", "fd cb __ 56", "fd cb __ 57",
        "fd cb __ 58", "fd cb __ 59", "fd cb __ 5a", "fd cb __ 5b", "fd cb __ 5c", "fd cb __ 5d", "fd cb __ 5e", "fd cb __ 5f",
        "fd cb __ 60", "fd cb __ 61", "fd cb __ 62", "fd cb __ 63", "fd cb __ 64", "fd cb __ 65", "fd cb __ 66", "fd cb __ 67",
        "fd cb __ 68", "fd cb __ 69", "fd cb __ 6a", "fd cb __ 6b", "fd cb __ 6c", "fd cb __ 6d", "fd cb __ 6e", "fd cb __ 6f",
        "fd cb __ 70", "fd cb __ 71", "fd cb __ 72", "fd cb __ 73", "fd cb __ 74", "fd cb __ 75", "fd cb __ 76", "fd cb __ 77",
        "fd cb __ 78", "fd cb __ 79", "fd cb __ 7a", "fd cb __ 7b", "fd cb __ 7c", "fd cb __ 7d", "fd cb __ 7e", "fd cb __ 7f",
        "fd cb __ 80", "fd cb __ 81", "fd cb __ 82", "fd cb __ 83", "fd cb __ 84", "fd cb __ 85", "fd cb __ 86", "fd cb __ 87",
        "fd cb __ 88", "fd cb __ 89", "fd cb __ 8a", "fd cb __ 8b", "fd cb __ 8c", "fd cb __ 8d", "fd cb __ 8e", "fd cb __ 8f",
        "fd cb __ 90", "fd cb __ 91", "fd cb __ 92", "fd cb __ 93", "fd cb __ 94", "fd cb __ 95", "fd cb __ 96", "fd cb __ 97",
        "fd cb __ 98", "fd cb __ 99", "fd cb __ 9a", "fd cb __ 9b", "fd cb __ 9c", "fd cb __ 9d", "fd cb __ 9e", "fd cb __ 9f",
        "fd cb __ a0", "fd cb __ a1", "fd cb __ a2", "fd cb __ a3", "fd cb __ a4", "fd cb __ a5", "fd cb __ a6", "fd cb __ a7",
        "fd cb __ a8", "fd cb __ a9", "fd cb __ aa", "fd cb __ ab", "fd cb __ ac", "fd cb __ ad", "fd cb __ ae", "fd cb __ af",
        "fd cb __ b0", "fd cb __ b1", "fd cb __ b2", "fd cb __ b3", "fd cb __ b4", "fd cb __ b5", "fd cb __ b6", "fd cb __ b7",
        "fd cb __ b8", "fd cb __ b9", "fd cb __ ba", "fd cb __ bb", "fd cb __ bc", "fd cb __ bd", "fd cb __ be", "fd cb __ bf",
        "fd cb __ c0", "fd cb __ c1", "fd cb __ c2", "fd cb __ c3", "fd cb __ c4", "fd cb __ c5", "fd cb __ c6", "fd cb __ c7",
        "fd cb __ c8", "fd cb __ c9", "fd cb __ ca", "fd cb __ cb", "fd cb __ cc", "fd cb __ cd", "fd cb __ ce", "fd cb __ cf",
        "fd cb __ d0", "fd cb __ d1", "fd cb __ d2", "fd cb __ d3", "fd cb __ d4", "fd cb __ d5", "fd cb __ d6", "fd cb __ d7",
        "fd cb __ d8", "fd cb __ d9", "fd cb __ da", "fd cb __ db", "fd cb __ dc", "fd cb __ fd", "fd cb __ de", "fd cb __ df",
        "fd cb __ e0", "fd cb __ e1", "fd cb __ e2", "fd cb __ e3", "fd cb __ e4", "fd cb __ e5", "fd cb __ e6", "fd cb __ e7",
        "fd cb __ e8", "fd cb __ e9", "fd cb __ ea", "fd cb __ eb", "fd cb __ ec", "fd cb __ ed", "fd cb __ ee", "fd cb __ ef",
        "fd cb __ f0", "fd cb __ f1", "fd cb __ f2", "fd cb __ f3", "fd cb __ f4", "fd cb __ f5", "fd cb __ f6", "fd cb __ f7",
        "fd cb __ f8", "fd cb __ f9", "fd cb __ fa", "fd cb __ fb", "fd cb __ fc", "fd cb __ fd", "fd cb __ fe", "fd cb __ ff",
        "c3", // JP nn
        "18", "20", "28", "38", "30", // JP e
        "c2", "3a", "72", "7a", "e2", "ea", "f2", "fa", // JP cc, nn
        "e9", // JP (HL)
        "dd e9", "fd e9", // JP (IX|IY)
        "10", // DJNZ
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
            const s = try t.toZ80State(allocator);
            defer s.deinit(allocator);

            const opcode = op.fetchOpcode(s);
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
