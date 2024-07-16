const std = @import("std");
const cpu = @import("cpu.zig");
const expect = std.testing.expect;
const expectEquals = std.testing.expectEqual;

const OpcodeTest = struct {
    name: []const u8,
    initial: struct { pc: u16, sp: u16, a: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, f: u8, q: u8, wz: u16, af_: u16, bc_: u16, de_: u16, hl_: u16, i: u8, r: u8, ix: u16, iy: u16, iff1: u1, iff2: u1, ram: [][2]u16 },
    final: struct { pc: u16, sp: u16, a: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, f: u8, q: u8, wz: u16, af_: u16, bc_: u16, de_: u16, hl_: u16, i: u8, r: u8, ix: u16, iy: u16, iff1: u1, iff2: u1, ram: [][2]u16 },
    cycles: []const [3]std.json.Value,
    ports: ?[]const [3]std.json.Value = null,

    fn toZ80State(self: *const OpcodeTest, allocator: std.mem.Allocator) !*cpu.Z80State {
        const impl = struct {
            pub fn read(ctx: *anyopaque, address: u16) u8 {
                const self2: *OpcodeTest = @ptrCast(@alignCast(ctx));
                for (self2.initial.ram) |location| {
                    const addr = location[0];
                    const value = location[1];
                    if (addr == address) {
                        return @truncate(value);
                    }
                }
                std.debug.panic("read: out of bounds, addr={d}\n", .{address});
            }

            pub fn write(ctx: *anyopaque, address: u16, value: u8) void {
                const self2: *OpcodeTest = @ptrCast(@alignCast(ctx));
                for (0..self2.initial.ram.len) |i| {
                    if (self2.initial.ram[i][0] == address) {
                        self2.initial.ram[i][1] = @intCast(value);
                        return;
                    }
                }
                std.debug.panic("write: out of bounds, addr={d}\n", .{address});
            }

            pub fn ioRead(ctx: *anyopaque, address: u16) u8 {
                const self2: *OpcodeTest = @ptrCast(@alignCast(ctx));
                for (self2.ports.?) |io_port| {
                    if (io_port[0].integer != address) continue;
                    std.debug.assert(io_port[2].string[0] == 'r');
                    return @intCast(io_port[1].integer);
                }
                std.debug.panic("io-read: FAIL, port={d} not found!\n", .{address});
            }

            pub fn ioWrite(ctx: *anyopaque, address: u16, value: u8) void {
                const self2: *OpcodeTest = @ptrCast(@alignCast(ctx));
                for (self2.ports.?) |io_port| {
                    if (io_port[0].integer != address) continue;
                    std.debug.assert(io_port[2].string[0] == 'w');
                    std.debug.assert(io_port[1].integer == value);
                    return;
                }

                std.debug.panic("io-write: FAIL, port={d} not found!\n", .{address});
            }
        };

        var s = try cpu.Z80State.init(
            allocator,
            self,
            impl.read,
            impl.write,
            impl.ioRead,
            impl.ioWrite,
        );

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

        if (cpu.z80_sim_q) {
            s.Q = init.q;
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

        try expectEquals(final.ix, s.IX.getValue());
        try expectEquals(final.iy, s.IY.getValue());
        try expectEquals(final.i, s.I);
        try expectEquals(final.iff1 == 1, s.IFF1);
        try expectEquals(final.iff2 == 1, s.IFF2);
        try expectEquals(final.r, s.R.getValue());

        if (cpu.z80_sim_q) {
            try expectEquals(final.q, s.Q);
        }

        try expectEquals(self.cycles.len, s.cycles);

        try expectEquals(final.af_, s.AF_.getValue());
        try expectEquals(final.bc_, s.BC_.getValue());
        try expectEquals(final.de_, s.DE_.getValue());
        try expectEquals(final.hl_, s.HL_.getValue());
        try expectEquals(final.wz, s.WZ.getValue());

        for (final.ram) |loc| {
            const address: u16 = @truncate(loc[0]);
            const value: u8 = @truncate(loc[1]);
            try expectEquals(value, s.memRead(address));
        }

        expectEquals(final.f, s.AF.getFlags()) catch |err| {
            const exp_flags: cpu.FlagsRegister = @bitCast(final.f);
            std.debug.print("Flags mismatch:\n  expected 0b{b:0>8} {any},\n  found    0b{b:0>8} {any}\n", .{
                final.f,         exp_flags,
                s.AF.getFlags(), s.AF.F,
            });
            std.debug.print("Context:\n  initial {any},\n  final {any}\n", .{ self.initial, self.final });
            return err;
        };
    }
};

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

pub fn printError(trace: ?*std.builtin.StackTrace, comptime format: []const u8, args: anytype) !void {
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

var test_io = TestIO{
    .memory = std.mem.zeroes([64 * 1024]u8),
    .ports = std.mem.zeroes([256]u8),
};

fn parseZ80TestFile(file_path: []const u8, allocator: std.mem.Allocator) !std.json.Parsed([]OpcodeTest) {
    const file = try std.fs.openFileAbsolute(
        file_path,
        .{
            .mode = std.fs.File.OpenMode.read_only,
        },
    );

    const json = try file.readToEndAlloc(allocator, 10_000_000);

    const parsed = try std.json.parseFromSlice(
        []OpcodeTest,
        allocator,
        json,
        .{ .ignore_unknown_fields = true },
    );

    return parsed;
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

fn execSingleStepTests(start_from: ?[]const u8) !void {
    const op = @import("opcodes.zig");

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    const tests_base_path = try std.fs.cwd().realpathAlloc(
        allocator,
        "./tests/SingleStepTests/z80/v1",
    );

    std.debug.print("\nSingleStepTests:\n", .{});

    const dir = try std.fs.openDirAbsolute(tests_base_path, .{
        .access_sub_paths = false,
        .iterate = true,
    });

    var dirwalker = try dir.walk(allocator);
    defer dirwalker.deinit();

    var start = if (start_from != null) false else true;

    while (try dirwalker.next()) |entry| {
        const test_file_name = entry.path;

        if (!start) {
            if (start_from) |start_from_file_name| {
                if (!std.mem.eql(u8, start_from_file_name, test_file_name)) {
                    std.debug.print("{s}: SKIPPED\n", .{test_file_name});
                    continue;
                } else {
                    start = true;
                }
            }
        }

        if (std.mem.eql(u8, "76", test_file_name[0..2]) or
            std.mem.eql(u8, "dd 76", test_file_name[0..5]) or
            std.mem.eql(u8, "fd 76", test_file_name[0..5]))
        {
            std.debug.print("{s}: SKIPPED\n", .{test_file_name});
            continue;
        }

        var arena2 = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        defer arena2.deinit();
        const allocator2 = arena2.allocator();

        const file_path = try std.mem.concat(
            allocator2,
            u8,
            &[_][]const u8{ tests_base_path, "/", test_file_name },
        );
        const parsed = try parseZ80TestFile(file_path, allocator2);
        const tests = parsed.value;

        for (tests, 0..) |t, i| {
            const s = try t.toZ80State(allocator2);
            defer s.deinit(allocator2);
            const opcode = op.fetchOpcode(s);
            op.exec(s, opcode);
            t.expectState(s) catch |err| {
                try printError(
                    @errorReturnTrace(),
                    "[{d}] {s}: FAIL",
                    .{ i, t.name },
                );
                return err;
            };
        }

        std.debug.print("{s}: OK\n", .{test_file_name});
    }
}

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const allocator = arena.allocator();
    defer arena.deinit();

    var args = try std.process.argsWithAllocator(allocator);
    _ = args.skip(); // zig

    const arg = args.next();

    var start_from: ?[]const u8 = null;

    if (arg) |cmd| {
        if (std.mem.eql(u8, "--start-from", cmd)) {
            if (args.next()) |file_name| {
                start_from = file_name;
            }
        } else {
            std.debug.print("Unknown cmd: {s}\n", .{cmd});
        }
    }

    try execSingleStepTests(start_from);
}
