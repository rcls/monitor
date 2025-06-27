
pub fn usqrt(x : u32) -> u32 {
    let mut top = 16;
    for i in 0..16 {
        if x < 1 << i * 2 {
            top = i;
            break;
        }
    }
    let mut result = 0;
    for bit in (0 .. top).rev() {
        let trial = result + (1 << bit);
        if trial * trial <= x {
            result = trial;
        }
    }
    result
}

#[cfg(test)]
pub fn nsqrt(x: u32) -> u32 {
    if x == 0 {
        return 0;
    }
    let mut result = 2 << (x.ilog2() >> 1);
    // or result = x >> (x.ilog2() + 1 >> 1);
    loop {
        let next = (result + x / result) >> 1;
        if next >= result {
            return result;
        }
        result = next;
    }
}

#[cfg(test)]
fn check_usqrt(x: u32) {
    let s = usqrt(x);
    let n = nsqrt(x);
    assert_eq!(s, n, "x={x}");
    assert!(s * s <= x);
    assert!(x <= s * s + 2 * s);
}

#[test]
fn test_usqrt() {
    for x in 0 ..= 1<<20 {
        check_usqrt(x);
    }
}
