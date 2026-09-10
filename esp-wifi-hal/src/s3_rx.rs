//! ESP32-S3 RX length checks, independent of DMA and MMIO.

pub(crate) const CONTROL_HEADER_LENGTH: usize = 48;

/// The S3 descriptor registers expose a DRAM offset and upper status bits.
pub(crate) const fn descriptor_address(raw: u32) -> Option<usize> {
    let offset = raw & 0xfffff;
    if offset == 0 {
        None
    } else {
        Some((0x3fc00000 | offset) as usize)
    }
}

pub(crate) fn valid_length(buffer: &[u8]) -> bool {
    let Some(word) = buffer.get(44..CONTROL_HEADER_LENGTH) else {
        return false;
    };
    let payload_length = buffer.len() - CONTROL_HEADER_LENGTH;
    let sig_length = (u32::from_le_bytes(word.try_into().unwrap()) & 0xfff) as usize;
    // Trailer rounding in BorrowedBuffer can subtract up to three padding bytes.
    // A header-only descriptor must never reach that arithmetic.
    payload_length >= 4 && sig_length >= payload_length
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_offsets_exclude_status_bits_and_zero_is_empty() {
        assert_eq!(descriptor_address(0x0009b3d4), Some(0x3fc9b3d4));
        assert_eq!(descriptor_address(0x0109b3e0), Some(0x3fc9b3e0));
        assert_eq!(descriptor_address(0), None);
        assert_eq!(descriptor_address(0x01000000), None);
    }

    #[test]
    fn truncated_and_header_only_descriptors_are_rejected() {
        let mut bytes = [0u8; 51];
        for sig in 0u32..=4095 {
            bytes[44..48].copy_from_slice(&sig.to_le_bytes());
            for length in 0..=bytes.len() {
                assert!(!valid_length(&bytes[..length]));
            }
        }
    }

    #[test]
    fn accepted_lengths_keep_trailer_rounding_in_bounds() {
        let mut bytes = [0u8; 128];
        for payload in 4..=80 {
            for sig in 0u32..=4095 {
                bytes[44..48].copy_from_slice(&sig.to_le_bytes());
                let valid = valid_length(&bytes[..CONTROL_HEADER_LENGTH + payload]);
                assert_eq!(valid, sig as usize >= payload);
                if valid {
                    let trailer = ((sig as usize - payload) + 3) & !3;
                    assert!(trailer <= sig as usize);
                    assert!(sig as usize - trailer <= payload);
                }
            }
        }
    }

    #[test]
    fn s2_length_word_does_not_reject_s3_ofdm_frames() {
        let mut bytes = [0u8; 80];
        bytes[24..28].fill(0xff);
        bytes[44..48].copy_from_slice(&36u32.to_le_bytes());
        assert!(valid_length(&bytes));
        bytes[44..48].copy_from_slice(&31u32.to_le_bytes());
        assert!(!valid_length(&bytes));
    }
}
