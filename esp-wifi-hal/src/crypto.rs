/// Length in bytes of a WEP-40 key.
pub const WEP40_KEY_LENGTH: usize = 5;
/// Length in bytes of a WEP-104 key.
pub const WEP104_KEY_LENGTH: usize = 13;
/// Length in bytes of a TKIP key.
pub const TKIP_KEY_LENGTH: usize = 16;
/// Length in bytes of an AES-128 key.
pub const AES_128_KEY_LENGTH: usize = 16;
/// Length in bytes of an AES-256 key.
pub const AES_256_KEY_LENGTH: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// A key supporting to different lengths.
pub enum MultiLengthKey<'a, const SHORT: usize, const LONG: usize> {
    /// Short key variant.
    Short(&'a [u8; SHORT]),
    /// Long key variant.
    Long(&'a [u8; LONG]),
}
impl<const SHORT: usize, const LONG: usize> MultiLengthKey<'_, SHORT, LONG> {
    /// Get the key length.
    pub const fn key_length(&self) -> usize {
        if let Self::Short(_) = self {
            SHORT
        } else {
            LONG
        }
    }
    /// Get a slice of the key.
    pub const fn key(&self) -> &[u8] {
        match self {
            Self::Short(key) => key.as_slice(),
            Self::Long(key) => key.as_slice(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// The type of the key.
pub enum KeyType {
    /// A key for use between two stations.
    Pairwise,
    /// A key used for group addressed traffic.
    Group,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Parameters for AES based ciphers.
pub struct AesCipherParameters<'a> {
    /// The key for the cipher.
    ///
    /// WARNING: Currently only the 128 bit keys work correctly.
    pub key: MultiLengthKey<'a, AES_128_KEY_LENGTH, AES_256_KEY_LENGTH>,
    /// The type of the key.
    pub key_type: KeyType,
    /// Enable management frame protection.
    pub mfp_enabled: bool,
    /// Enable signalling and payload protection for A-MSDUs.
    pub spp_enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Parameters for all supported ciphers.
///
/// Only CCMP has been tested so far. GCMP is commented out, since it doesn't work on the ESP32 and
/// ESP32-S2 and I'm not sure, whether it's just not supported, or I'm doing something wrong.
pub enum CipherParameters<'a> {
    /// Wired Equivalent Privacy (WEP)
    ///
    /// WARNING: I think by now it goes without saying, just how fucking broken WEP is. It should
    /// absolutely not be used for anything! The only purpose it currently serves, is as a good
    /// read on Wikipedia.
    Wep(MultiLengthKey<'a, WEP40_KEY_LENGTH, WEP104_KEY_LENGTH>),
    /// Temporal Key Integrity Protocol (TKIP)
    ///
    /// WARNING: It's WEP but slightly less shit. Still very broken though.
    Tkip(&'a [u8; TKIP_KEY_LENGTH], KeyType),
    /// AES CTR/CBC-MAC Mode Protocol (CCMP)
    Ccmp(AesCipherParameters<'a>),
    /*
    /// AES Galois/CTR Mode Protocol (GCMP)
    ///
    /// WARNING: This doesn't work on the ESP32 and ESP32-S2 currently.
    Gcmp(AesCipherParameters<'a>),
    */
}
impl CipherParameters<'_> {
    /// Get the [AesCipherParameters] if any.
    pub const fn aes_cipher_parameters(&self) -> Option<&AesCipherParameters> {
        match self {
            CipherParameters::Ccmp(parameters) => Some(parameters),
            // CipherParameters::Gcmp(parameters) => Some(parameters),
            _ => None,
        }
    }
    /// Check if the key is 256 bits long.
    pub fn is_256_bit_key(&self) -> bool {
        self.aes_cipher_parameters()
            .is_none_or(|aes_cipher_parameters| {
                aes_cipher_parameters.key.key_length() == AES_256_KEY_LENGTH
            })
    }
    /// Check if the algorithm is WEP104.
    pub const fn is_wep_104(&self) -> bool {
        if let CipherParameters::Wep(key) = self {
            key.key_length() == WEP104_KEY_LENGTH
        } else {
            false
        }
    }
    /// Check if signalling and payload protection for A-MSDUs is enabled.
    pub fn is_spp_enabled(&self) -> bool {
        self.aes_cipher_parameters()
            .is_none_or(|aes_cipher_parameters| aes_cipher_parameters.spp_enabled)
    }
    /// Check if management frame protection (MFP) is enabled.
    pub fn is_mfp_enabled(&self) -> bool {
        self.aes_cipher_parameters()
            .is_none_or(|aes_cipher_parameters| aes_cipher_parameters.mfp_enabled)
    }
    /// Get the key type if any.
    pub fn key_type(&self) -> Option<KeyType> {
        self.aes_cipher_parameters()
            .map(|aes_cipher_parameters| aes_cipher_parameters.key_type)
            .or(if let CipherParameters::Tkip(_, key_type) = self {
                Some(*key_type)
            } else {
                None
            })
    }
    /// Get the key slice.
    pub const fn key(&self) -> &[u8] {
        match self {
            Self::Wep(key) => key.key(),
            Self::Tkip(key, _) => key.as_slice(),
            _ => self.aes_cipher_parameters().unwrap().key.key(),
        }
    }
    /// Check if the cipher is an authenticated encryption with associated data (AEAD) algorithm.
    pub const fn is_aead(&self) -> bool {
        matches!(self, Self::Wep(_))
    }
    /// Check if the key is for pairwise addressed frames.
    pub fn is_pairwise(&self) -> bool {
        if let Some(key_type) = self.key_type() {
            key_type == KeyType::Pairwise
        } else {
            // This means the cipher is WEP, for which the key is always pairwise and group.
            true
        }
    }
    /// Check if the key is for group addressed frames.
    pub fn is_group(&self) -> bool {
        if let Some(key_type) = self.key_type() {
            key_type == KeyType::Group
        } else {
            // This means the cipher is WEP, for which the key is always pairwise and group.
            true
        }
    }
    /// The ID of the algorithm.
    pub const fn algorithm(&self) -> u8 {
        match self {
            Self::Wep(_) => 1,
            Self::Tkip(_, _) => 2,
            Self::Ccmp(_) => 3,
            // Self::Gcmp(_) => 5,
        }
    }
    /// Get the length of the MIC.
    pub fn mic_length(&self) -> usize {
        self.aes_cipher_parameters()
            .map_or(0, |aes_cipher_parameters| {
                aes_cipher_parameters.key.key_length()
            })
    }
}
