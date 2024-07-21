const std = @import("std");

pub const TapBlock = struct {
    len: u16,
    flag: u8,
    data: union(enum) {
        header: struct {
            typ: enum(u8) {
                program = 0,
                number_array = 1,
                character_array = 2,
                code = 3,
            },
            file_name: [10]u8,
            data_len: u16,
            param_1: u16,
            param_2: u16,
        },
        data_block: []u8,
    },
    checksum: u8,
};

pub const TapFile = struct {
    raw: []u8,
    blocks: []TapBlock,

    const Self = @This();

    pub fn deinit(self: Self, allocator: std.mem.Allocator) void {
        allocator.free(self.raw);
    }
};

pub fn parseTapFile(allocator: std.mem.Allocator, tap: []u8) !TapFile {
    var blocks = std.ArrayList(TapBlock).init(allocator);
    var buff = tap[0..];
    var i: usize = 0;

    while (true) {
        if (buff.len == 0) break;

        // Size of this block
        const len = buff[0] | @as(u16, buff[1]) << 8;
        const data = buff[2 .. len + 2];

        // Checksum
        var cs: u8 = 0;
        for (data[0 .. data.len - 1]) |b| {
            cs ^= b;
        }

        std.debug.assert(cs == data[data.len - 1]);

        var block: TapBlock = undefined;

        if (len == 19) { // header
            block = TapBlock{
                .len = len,
                .flag = data[0],
                .checksum = data[data.len - 1],
                .data = .{
                    .header = .{
                        .typ = @enumFromInt(data[1]),
                        .file_name = data[2..12].*,
                        .data_len = data[12] | @as(u16, data[13]) << 8,
                        .param_1 = data[14] | @as(u16, data[15]) << 8,
                        .param_2 = data[16] | @as(u16, data[17]) << 8,
                    },
                },
            };
        } else {
            block = TapBlock{
                .len = len,
                .flag = data[0],
                .data = .{ .data_block = data[1 .. len - 1] },
                .checksum = data[data.len - 1],
            };
            std.debug.assert(block.len - 2 == block.data.data_block.len); // -2 = flags+checksum
        }

        (try blocks.addOne()).* = block;
        i += 1;
        buff = buff[len + 2 ..];
    }

    return .{
        .raw = tap,
        .blocks = blocks.items,
    };
}
