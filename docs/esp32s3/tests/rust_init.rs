#[path = "../../../esp-wifi-hal/src/s3_mac.rs"]
mod s3_mac;

mod ffi {
    pub unsafe fn slowclk_cal_get() -> u32 {
        0x12345678
    }
}

unsafe extern "C" {
    fn s3_test_reset(seed: u32);
    fn s3_test_reference_init();
    fn s3_test_trace_length() -> usize;
    fn s3_test_trace_entry(index: usize, address: *mut usize, value: *mut u32, kind: *mut u8);
    fn s3_test_callbacks();
}

fn trace() -> Vec<(usize, u32, u8)> {
    (0..unsafe { s3_test_trace_length() })
        .map(|index| {
            let (mut address, mut value, mut kind) = (0, 0, 0);
            unsafe {
                s3_test_trace_entry(index, &mut address, &mut value, &mut kind);
            }
            (address, value, kind)
        })
        .collect()
}

#[test]
fn rust_init_matches_reviewed_c_transactions_and_callbacks() {
    for seed in [0, u32::MAX, 0xaaaaaaaa, 0x55555555] {
        unsafe {
            s3_test_reset(seed);
            s3_test_reference_init();
            s3_test_callbacks();
        }
        let reference = trace();
        unsafe {
            s3_test_reset(seed);
            s3_mac::init();
            s3_test_callbacks();
        }
        let rust = trace();
        assert_eq!(
            rust, reference,
            "MMIO order/value mismatch with initial register pattern {seed:#x}"
        );
        assert!(reference.len() > 100);
    }
}
