const std = @import("std");
const cpu = @import("cpu.zig");
const opcodes = @import("opcodes.zig");
const tape = @import("tap.zig");

const TestPlatform = enum {
    /// ZX Spectrum
    spectrum,
    /// CP/M
    cpm,
};

const TestFormat = enum {
    /// DOS COM file
    com,
    /// Spectrum tape format
    tap,
    /// Tap but needs the ZX spectrum rom
    tap_woody,
};

pub const TestIO = struct {
    memory: [64 * 1024]u8,
    ports: [256]u8,
    platform: TestPlatform,

    const Self = @This();

    pub fn init(platform: TestPlatform) !*Self {
        const s = try std.testing.allocator.create(Self);
        s.memory = std.mem.zeroes([64 * 1024]u8);
        s.ports = std.mem.zeroes([256]u8);
        s.platform = platform;
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

        // do not write outside ROM
        if (self.platform == .spectrum and address <= 0x3FFF) {
            return;
        }

        self.memory[address] = value;
    }

    pub fn ioRead(ctx: *const anyopaque, address: u16) u8 {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));

        const port: u8 = @truncate(address & 0xFF);

        if (self.platform == .spectrum) {
            if (port % 2 == 0) { // event ports
                return 0xbf; // 191
            } else { // odd ports
                return 0xff; // 255
            }
        }

        return self.ports[port];
    }

    pub fn ioWrite(ctx: *const anyopaque, address: u16, value: u8) void {
        const self: *Self = @ptrCast(@constCast(@alignCast(ctx)));
        const port: u8 = @truncate(address & 0xFF);
        self.ports[port] = value;
    }
};

const TestRom = struct {
    /// Test title
    title: []const u8,
    /// The author(s) of the test suite
    author: []const u8,
    /// The path to the test ROM
    file_path: []const u8,
    /// The test system (for I/O, e.g print and stuff)
    platform: TestPlatform,
    /// The test format
    format: TestFormat,
};

const test_suite = [_]TestRom{
    TestRom{ .title = "Preliminary tests (yaze-1.14)", .author = "Frank D. Cringle (2004)", .file_path = "prelim.com", .platform = .cpm, .format = .com },
    TestRom{ .title = "Z80 instruction exerciser (doc) (yaze-1.14)", .author = "Frank D. Cringle (2004)", .file_path = "zexdoc.com", .platform = .cpm, .format = .com },
    TestRom{ .title = "Z80 instruction exerciser (all) (yaze-1.14)", .author = "Frank D. Cringle (2004)", .file_path = "zexall.com", .platform = .cpm, .format = .com },
    TestRom{ .title = "Diagnostics II V1.2 CPU Test (Supersoft)", .author = "Supersoft Associates (1981)", .file_path = "CPUTEST.COM", .platform = .cpm, .format = .com }, // https://gitlab.com/retroabandon/cpm-re/-/tree/main/ss-cpudiag
    TestRom{
        .title = "Z80 Test suite",
        .author = "Mark Woodmass (2008)",
        .file_path = "Z80_Test_Suite_2008_woodmass.tap",
        .platform = .spectrum,
        .format = .tap_woody,
    },
    TestRom{ .title = "Zilog Z80 CPU Test Suite (doc) (1.2a)", .author = "Patrik Rak", .file_path = "z80doc.tap", .platform = .spectrum, .format = .tap },
    TestRom{ .title = "Zilog Z80 CPU Test Suite (all) (1.2a)", .author = "Patrik Rak", .file_path = "z80full.tap", .platform = .spectrum, .format = .tap },
    TestRom{ .title = "Zilog Z80 CPU Test Suite (memptr) (1.2a)", .author = "Patrik Rak", .file_path = "z80memptr.tap", .platform = .spectrum, .format = .tap },
    TestRom{ .title = "Zilog Z80 CPU Test Suite (ccf) (1.2a)", .author = "Patrik Rak", .file_path = "z80ccf.tap", .platform = .spectrum, .format = .tap },
    TestRom{ .title = "Zilog Z80 CPU Test Suite (ccfscr) (1.2a)", .author = "Patrik Rak", .file_path = "z80ccfscr.tap", .platform = .spectrum, .format = .tap },
};

// see: http://www.gaby.de/cpm/manuals/archive/cpm22htm/ch5.htm
fn cpmBdos(r: *cpu.Z80State, s: *Screen) !void {
    const writer = std.io.getStdOut().writer();

    switch (r.BC.low) {
        2 => { // Console Output
            const char = r.DE.low;
            if (char == 0xa) {
                s.out[s.idx] = 0x0;
                try writer.writeByte(0xa);
                s.idx = 0;
            } else if (s.idx < 255) {
                s.out[s.idx] = char; // E
                try writer.writeByte(char);
                s.idx = s.idx + 1;
            }
        },
        9 => { // Print string
            var addr = r.DE.getValue();
            while (true) : (addr +%= 1) {
                const char = r.memRead(addr);

                if (char == 0xa) {
                    s.out[s.idx] = 0x0;
                    try writer.writeByte(0xa);
                    s.idx = 0;
                } else if (s.idx < 255 and char != '$') {
                    s.out[s.idx] = char; // (DE)
                    try writer.writeByte(char);
                    s.idx = s.idx + 1;
                } else {
                    break;
                }
            }
        },
        else => |syscall| {
            std.debug.print(
                "Unhandled CP/M system call: {d}\n",
                .{syscall},
            );
            return;
        },
    }

    // fake return from the syscall
    const fakeOpcode: opcodes.OpCode = @bitCast(@as(u8, 0));
    _ = opcodes.ret(r, &fakeOpcode);
}

fn zxSpectrum(r: *cpu.Z80State, s: *Screen) !void {
    const char = r.AF.A;
    const writer = std.io.getStdOut().writer();
    if (s.spectrum_tab == 0 and s.spectrum_at == 0) {
        switch (char) {
            0x0D => { // CR
                s.out[s.idx] = 0x0;
                try writer.writeByte(0xa);
                s.spectrum_x = 0;
                s.idx = 0;
            },
            0x17, 0x9 => { // TAB
                s.spectrum_tab = 2;
                s.tab_expansion = 0;
            },
            0x7f => { // ©
                s.out[s.idx] = '©';
                s.idx = (s.idx + 1) % s.out.len;
            },
            0x16 => { // AT
                // not implemented
            },
            else => {
                if (char >= 32 and char < 127) {
                    s.out[s.idx] = char;
                    try writer.writeByte(char);
                    s.idx = (s.idx + 1) % s.out.len;

                    if (s.spectrum_x +% 1 > 32) {
                        try writer.writeByte(0xa);
                        s.idx = 0;
                    }
                    s.spectrum_x = (s.spectrum_x +% 1) % 32;
                } else {
                    // std.debug.print(" [[0x{x} | {c}]] ", .{ char, char });
                }
            },
        }
    } else { // TAB expansion, expect 2 bytes after <TAB>
        switch (s.spectrum_tab) {
            2 => {
                s.tab_expansion = char;
                s.spectrum_tab -= 1;
            },
            1 => {
                s.tab_expansion |= @as(u16, char) << 8;
                s.tab_expansion %= 32;
                s.tab_expansion = s.tab_expansion - s.spectrum_x;
                s.spectrum_tab -= 1;
                while (s.tab_expansion > 0) : (s.tab_expansion -= 1) {
                    try writer.writeByte(' ');
                }
            },
            else => s.spectrum_tab = 0,
        }
    }
    _ = opcodes.ret(r, &@bitCast(@as(u8, 0)));
}

const Screen = struct {
    out: [255]u8,
    idx: usize,
    spectrum_tab: u8,
    spectrum_at: u8,
    spectrum_x: u8,
    tab_expansion: u16,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator) !*Self {
        const s = try allocator.create(Self);
        s.out = std.mem.zeroes([255]u8);
        s.idx = 0;
        s.spectrum_tab = 0;
        s.tab_expansion = 0;
        s.spectrum_x = 0;
        s.spectrum_at = 0;
        return s;
    }
};

fn tryReadByte(stdin: *const std.fs.File) u8 {
    return stdin.reader().readByte() catch {
        return 0;
    };
}

fn spectrumLastKeyUpdater(io: *TestIO, s: *cpu.Z80State) void {
    const stdin = std.io.getStdIn();
    defer stdin.close();

    while (true) {
        std.time.sleep(20 * std.time.ms_per_s);
        const k = tryReadByte(&stdin);

        if (k != 0) {
            std.debug.print("PC={d}\n", .{s.PC});
            io.memory[23560] = k; // 23560: LAST-K - Last key pressed;
        }
    }
}

fn runTest(test_rom: TestRom) !void {
    const allocator = std.heap.page_allocator;

    const base_path = try std.fs.cwd().realpathAlloc(allocator, "./tests/roms");
    defer allocator.free(base_path);

    const full_path = try std.fs.path.join(allocator, &[_][]const u8{ base_path, test_rom.file_path });
    defer allocator.free(full_path);

    std.debug.print("Loading test rom: {s}\n", .{full_path});

    const file = try std.fs.openFileAbsolute(
        full_path,
        .{
            .mode = std.fs.File.OpenMode.read_only,
        },
    );
    defer file.close();

    var io = TestIO{
        .memory = std.mem.zeroes([64 * 1024]u8),
        .ports = std.mem.zeroes([256]u8),
        .platform = test_rom.platform,
    };

    const screen = try Screen.init(allocator);

    var s = try cpu.Z80State.init(
        std.heap.page_allocator,
        &io,
        TestIO.read,
        TestIO.write,
        TestIO.ioRead,
        TestIO.ioWrite,
    );
    defer s.deinit(std.heap.page_allocator);

    const rom = try file.reader().readAllAlloc(allocator, 1024 * 1024); // 1024KB
    defer allocator.free(rom);

    var start_addr: u16 = 0x0000;
    var code_addr: u16 = 0x0000;
    var code_size: u16 = 0;

    switch (test_rom.format) {
        .com => {
            start_addr = 0x0100; // The whole program is loaded at 100h
            code_addr = 0x0000;
            code_size = @truncate(rom.len); // a .COM file it's a flat binary
        },
        .tap, .tap_woody => {
            const tap = try tape.parseTapFile(allocator, rom);
            var offset: u16 = 0;
            std.debug.print("Loading TAP file...\n", .{});
            // Find the code block
            for (0..tap.blocks.len) |i| {
                const b = tap.blocks[i];
                std.debug.print("{d:0>4}  block({d}): len={d} \n", .{ offset, i, b.len });

                switch (b.data) {
                    .header => |h| {
                        // filename is white-space padded
                        const trim = std.mem.lastIndexOfNone(u8, &h.file_name, " ") orelse h.file_name.len;
                        std.debug.print("{d:0>4}    head> type={d} filename='{s}' datalen={d} p1=0x{X} p2={d}\n", .{
                            offset + 2,
                            @intFromEnum(h.typ),
                            h.file_name[0 .. trim + 1],
                            h.data_len,
                            h.param_1,
                            h.param_2,
                        });
                        if (h.typ == .code) {
                            code_size = h.data_len;
                            start_addr = h.param_1;
                            code_addr = offset + b.len //
                            + 2 // flag + checksum of the header
                            + 2 // file + checksum of the block itself
                            + 1;
                        }
                    },
                    .data_block => |d| {
                        std.debug.print("{d:0>4}    data> blocksize={d}\n", .{ offset + 2, d.len });
                    },
                }

                offset += b.len + 2;
            }

            std.debug.assert(code_size > 0);

            io.memory[0x7000] = 0xcd; // call nn
            io.memory[0x7001] = @truncate(start_addr & 0xFF); // low
            io.memory[0x7002] = @truncate((start_addr >> 8) & 0xFF); // high
            io.memory[0x7003] = 0x76; // halt
            s.PC = 0x7000;
            io.ports[0xFE] = 0xBF; // needed by some tests

            if (test_rom.format == .tap_woody) {
                const full_path2 = try std.fs.path.join(allocator, &[_][]const u8{ base_path, "zx_firmware.rom" });
                defer allocator.free(full_path2);

                std.debug.print("Loading ZX Firmware rom: {s}\n", .{full_path2});

                const firmware_file = try std.fs.openFileAbsolute(
                    full_path2,
                    .{
                        .mode = std.fs.File.OpenMode.read_only,
                    },
                );
                defer firmware_file.close();
                const zx_firmware = try firmware_file.reader().readAllAlloc(allocator, 16 * 1024);
                std.debug.assert(zx_firmware.len == 16384);
                std.mem.copyForwards(u8, &io.memory, zx_firmware);

                // start_addr = 0x8057;
                s.SP.setValue(0x7FE8);
                s.AF.setValue(0x3222);
            }
        },
    }
    std.debug.print("start_address=0x{x},code_size={d},code_addr={d}\n", .{
        start_addr,
        code_size,
        code_addr,
    });

    const program = rom[code_addr..code_size];

    std.debug.print("Program len: {d}\n", .{program.len});

    s.PC = start_addr;

    for (program, start_addr..) |opcode, i| {
        io.memory[i] = opcode;
    }

    switch (test_rom.platform) {
        .spectrum => {
            s.I = 0x3f;
            s.IM = 1;
            const thread = try std.Thread.spawn(.{}, spectrumLastKeyUpdater, .{ &io, s });
            _ = thread;
        },
        else => {},
    }

    while (true) {
        const opcode = opcodes.fetchOpcode(s);

        opcodes.exec(s, opcode);

        switch (test_rom.platform) {
            .cpm => switch (s.PC) {
                0x0000 => break, // HALT
                5 => try cpmBdos(s, screen), // BDOS syscall
                else => {},
            },
            .spectrum => switch (s.PC) {
                0x0D6B => _ = opcodes.ret(s, &@bitCast(@as(u8, 0))), // CLS
                0x1601 => _ = opcodes.ret(s, &@bitCast(@as(u8, 0))), // CHAN_OPEN
                0x0010 => _ = try zxSpectrum(s, screen), // PRINT A CHARACTER' RESTART
                // 4761
                else => {},
            },
        }
    }
}

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const allocator = arena.allocator();
    defer arena.deinit();

    var args = try std.process.argsWithAllocator(allocator);
    _ = args.skip(); // zig

    const arg = args.next();

    if (arg) |cmd| {
        if (std.mem.eql(u8, "list", cmd)) {
            std.debug.print("[ID]  Description\n", .{});
            std.debug.print("------------------\n", .{});
            for (test_suite, 0..) |test_, i| {
                std.debug.print("[{d}] - {s}\n", .{ i, test_.title });
            }
            std.debug.print("\nTo run:  zig build run-z80-test-suite -- run [ID]\n\n", .{});
        } else if (std.mem.eql(u8, "run", cmd)) {
            if (args.next()) |test_idx_str| {
                const test_idx = try std.fmt.parseInt(usize, test_idx_str, 10);
                if (test_idx >= test_suite.len) {
                    std.debug.print("Unknown test {d}, exit\n", .{test_idx});
                    return;
                }
                try runTest(test_suite[test_idx]);
            }
        } else {
            std.debug.print("Unknown cmd: {s}\n", .{cmd});
        }
    }
}
