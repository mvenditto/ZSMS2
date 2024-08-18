const std = @import("std");
const byte = @import("byte.zig");
const File = std.fs.File;
const arch = @import("builtin").target.cpu.arch;
const host_endianess = arch.endian();

pub const SystemRegion = enum(u4) {
    sms_japan = 0x3,
    sms_export = 0x4,
    gg_japan = 0x5,
    gg_export = 0x6,
    gg_international = 0x7,
};

/// The SEGA header.
pub const SEGAHeader = struct {
    /// The raw 16-byte header
    raw_header: *const [16]u8,
    /// The 16bit checksum from the rom
    checksum: u16,
    /// The calculated checksum
    checksum_calc: u16,
    /// The 2.5 byte product code
    product_code: u20,
    /// The rom size code
    rom_size: u4,
    /// The rom system/region
    system_region: SystemRegion,
    /// The rom version
    version: u4,
};

/// The SDSC header for homebrew ROMs.
pub const SDSCHeader = struct {
    raw_header: *const [16]u8,
    version: struct {
        minor: u8,
        major: u8,
    },
    release_date: struct {
        day: u8,
        month: u8,
        year: u16,
    },
    author_name: [*:0]const u8,
    program_name: [*:0]const u8,
    description: [*:0]const u8,
};

pub const ROM = struct {
    /// The total size of the rom in bytes
    size: usize,
    /// A buffer containing the whole rom memory
    mem: []const u8,
    /// The SEGA header, if present
    sega_header: ?SEGAHeader,
    /// The SDSC header, if present
    sdsc_header: ?SDSCHeader,

    pub fn close(self: *const ROM) void {
        std.heap.page_allocator.free(self.mem);
    }
};

pub const ROMLoadError = File.OpenError ||
    File.StatError ||
    File.ReadError ||
    std.mem.Allocator.Error ||
    error{StreamTooLong};

/// Coverts the 2.5 bytes of the product code to its decimal representation.
/// eg. [26 70 A] becomes 107026.
pub fn parseBCDProductCode(d: *const [3]u8) u20 {
    var res: u20 = 0;
    // first two bytes are the last 4 digits of the product code in BCD format
    res += byte.getLowNibble(d[0]) * std.math.pow(u20, 10, 0);
    res += byte.getHighNibble(d[0]) * std.math.pow(u20, 10, 1);
    res += byte.getLowNibble(d[1]) * std.math.pow(u20, 10, 2);
    res += byte.getHighNibble(d[1]) * std.math.pow(u20, 10, 3);
    // high-nibble of the 5th byte is the remaining digits of the product code
    res += byte.getHighNibble(d[2]) * std.math.pow(u20, 10, 4);
    return res;
}

/// Tries to map the product code to a human-readable name.
pub fn tryGetCartridgeName(product_code: u20) []const u8 {
    return switch (product_code) {
        500...599 => "Japanese C-5xx",
        1300...1399 => "Japanese G-13xx",
        3901...3901 => "Parker Borthers",
        4001...4499 => "The Sega Card (32KB)",
        4501...4507 => "The Sega Cartridge (32KB)",
        4580...4584 => "The Sega Cartridge (32KB)",
        5051...5199 => "The Mega Cartridge (128KB)",
        5500...5599 => "The Mega Plus Cartridge (128KB with battery-backed RAM)",
        5044...5044 => "The Combo Cartridge",
        6001...6081 => "The Combo Cartridge",
        7001...7499 => "The Two-Mega Cartridge (256KB)",
        7500...7599 => "The Two-Mega Plus Cartridge (256KB with battery-backed RAM)",
        8001...8499 => "The 3-Dimensional Mega Cartridge",
        9001...9499 => "The Four-Mega Cartridge (512KB)",
        9500...9599 => "The Four-Mega Plus Cartridge (512KB with battery-backed RAM)",
        else => "Unknown cartridge name",
    };
}

/// Calculates the checksum of a lower-region and an optional upper-region of the ROM memory.
///
/// The checksum is the 16 bit (little-endian) sum of the byte in the specified ranges.
pub fn calcChecksum(
    data: []const u8,
    start_lower: usize,
    end_lower: usize,
    opt_start_upper: ?usize,
    opt_end_upper: ?usize,
) u16 {
    var cs: u32 = 0;

    for (data[start_lower .. end_lower + 1]) |b| {
        cs += b;
    }

    // depending on the rom size, there may be an upper range
    if (opt_start_upper) |start_upper| {
        if (opt_end_upper) |end_upper| {
            for (data[start_upper .. end_upper + 1]) |b| {
                cs += b;
            }
        }
    }

    cs = switch (host_endianess) {
        .big => @byteSwap(cs),
        .little => cs,
    };

    return @truncate(cs);
}

pub fn getRomSizeKB(rom_size: u4) u16 {
    return switch (rom_size) {
        0xA => 8,
        0xB => 16,
        0xC => 32,
        0xD => 48,
        0xE => 64,
        0xF => 128,
        0x0 => 256,
        0x1 => 512,
        0x2 => 1024,
        else => 0,
    };
}

/// Calculates the checksum of a whole rom, based on the rom size specified in the header.
pub fn calcTotalChecksum(data: []const u8, rom_size: u4) u16 {
    return switch (rom_size) {
        0xA => calcChecksum(data, 0x0000, 0x1fef, null, null), // 8KB
        0xB => calcChecksum(data, 0x0000, 0x3fef, null, null), // 16KB
        0xC => calcChecksum(data, 0x0000, 0x7fef, null, null), // 32KB
        0xD => calcChecksum(data, 0x0000, 0xbfef, null, null), // 48KB
        0xE => calcChecksum(data, 0x0000, 0x7fef, 0x8000, 0x0FFFF), // 64KB
        0xF => calcChecksum(data, 0x0000, 0x7fef, 0x8000, 0x1FFFF), // 128KB
        0x0 => calcChecksum(data, 0x0000, 0x7fef, 0x8000, 0x3FFFF), // 256KB
        0x1 => calcChecksum(data, 0x0000, 0x7fef, 0x8000, 0x7FFFF), // 512KB
        0x2 => calcChecksum(data, 0x0000, 0x7fef, 0x8000, 0xFFFFF), // 1024KB
        else => 0,
    };
}

/// Tries to extract the 16-byte SEGA header searching in well known locations.
fn tryFindSegaHeader(data: []const u8) ?struct { offset: usize, data: *const [16]u8 } {
    const locations = [3]usize{ 0x7ff0, 0x3ff0, 0x1ff0 };

    inline for (locations) |offset| {
        const buff = data[offset .. offset + 16];
        if (std.mem.eql(u8, buff[0..8], "TMR SEGA")) {
            return .{ .offset = offset, .data = buff };
        }
    }

    return null;
}

/// Tries to extract the SDSC header based on the offset of the SEGA header.
fn tryExtractSDSCHeader(data: []const u8, header_offset: usize) ?SDSCHeader {
    const sdsc_offset = header_offset - 16;
    const sdsc_header = data[sdsc_offset .. sdsc_offset + 16];

    if (!std.mem.eql(u8, sdsc_header[0..4], "SDSC")) {
        return null;
    }

    const h = sdsc_header[4..16];
    const author_addr: u16 = @bitCast(h[6..8].*);
    const name_addr: u16 = @bitCast(h[8..10].*);
    const desc_addr: u16 = @bitCast(h[10..12].*);
    const rom_begin_addr = @intFromPtr(data.ptr);

    const author: [*:0]const u8 = switch (author_addr) {
        0xffff => "",
        0x0000 => "",
        else => @ptrFromInt(rom_begin_addr + author_addr),
    };

    const name: [*:0]const u8 = switch (name_addr) {
        0xffff => "",
        0x0000 => "",
        else => @ptrFromInt(rom_begin_addr + name_addr),
    };

    const desc: [*:0]const u8 = switch (desc_addr) {
        0xffff => "",
        0x0000 => "",
        else => @ptrFromInt(desc_addr + name_addr),
    };

    return SDSCHeader{
        .raw_header = @ptrCast(sdsc_header),
        .version = .{
            .major = @truncate(byte.bcdToDecimal(h[0..1])),
            .minor = @truncate(byte.bcdToDecimal(h[1..2])),
        },
        .release_date = .{
            .day = @truncate(byte.bcdToDecimal(h[2..3])),
            .month = @truncate(byte.bcdToDecimal(h[3..4])),
            .year = @truncate(byte.bcdToDecimal(h[4..6])),
        },
        .author_name = author,
        .program_name = name,
        .description = desc,
    };
}

/// Opens and parses a ROM from the given absolute path to a file.
pub fn loadRomAbsolute(absolute_path: []const u8) ROMLoadError!ROM {
    const file = try std.fs.openFileAbsolute(absolute_path, .{
        .mode = std.fs.File.OpenMode.read_only,
    });

    defer file.close();

    const allocator = std.heap.page_allocator;
    const buff = try file.reader().readAllAlloc(allocator, 1024 * 1024); // 1024KB

    // Find the SEGA header, if any
    const opt_sega_header_location = tryFindSegaHeader(buff);

    var sdsc_header: ?SDSCHeader = null;
    var sega_header: ?SEGAHeader = null;

    if (opt_sega_header_location) |sega_header_location| {
        const header = sega_header_location.data;
        const rom_size = byte.getLowNibble(header[15]);
        const checksum: u16 = @bitCast(buff[0x7ffa .. 0x7ffa + 2].*);
        const checksum_calc = calcTotalChecksum(buff, rom_size);
        const product_code = parseBCDProductCode(buff[0x7ffc .. 0x7ffc + 3]);
        const region_code = byte.getHighNibble(header[15]);
        const version = byte.getHighNibble(header[14]);

        // if a SEGA header was detected, try to find an SDSC header
        sdsc_header = tryExtractSDSCHeader(buff, sega_header_location.offset);

        sega_header = SEGAHeader{
            .checksum = checksum,
            .checksum_calc = checksum_calc,
            .raw_header = header,
            .rom_size = rom_size,
            .product_code = product_code,
            .system_region = @enumFromInt(region_code),
            .version = version,
        };
    }

    return ROM{
        .size = buff.len,
        .mem = buff,
        .sega_header = sega_header,
        .sdsc_header = sdsc_header,
    };
}

pub fn printRomInfo(rom: *const ROM) void {
    if (rom.sega_header) |h| {
        const tmr_sega: []const u8 = h.raw_header[0..8];

        const rom_size_kb = getRomSizeKB(h.rom_size);
        const product_name = tryGetCartridgeName(h.product_code);

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
