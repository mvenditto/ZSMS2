const std = @import("std");

/// Extracts the low nibble of the input byte.
pub inline fn getLowNibble(b: u8) u4 {
    return @intCast(b & 0x0F);
}

/// Extracts the high nibble of the input byte.
pub inline fn getHighNibble(b: u8) u4 {
    return @intCast((b >> 4) & 0x0F);
}

/// Combines two 4-bit unsigned integers to form a byte.
pub inline fn nibblesToByte(low: u4, high: u4) u8 {
    return low | (@as(u8, high) << 4);
}

/// Converts the bytes of a Binary Coded Decimal to its decimal representation.
pub fn bcdToDecimal(d: []const u8) u32 {
    var res: u32 = 0;
    var i: u32 = 0;
    var p: u32 = 0;

    while (i < d.len) : (i += 1) {
        res += getLowNibble(d[i]) * std.math.pow(u32, 10, p);
        res += getHighNibble(d[i]) * std.math.pow(u32, 10, p + 1);
        p += 2;
    }

    return res;
}
