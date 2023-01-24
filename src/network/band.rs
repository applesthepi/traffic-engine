use super::{Network};

#[derive(Default, Debug, Clone, Copy)]
pub struct BandIdentity {
	pub band: u32,
	pub clip: u32,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct Band {
	pub src_clip: u32,
	pub src_min: u8,
	pub src_max: u8,

	pub dst_clip: u32,
	pub dst_min: u8,
	pub dst_max: u8,

	pub empty: bool,
}

impl Band {
	pub fn new(
		network: &mut Network,
		src_clip: u32, dst_clip: u32
	) -> u32 {
		let id = network.fetch_band_id();
		let mut wa_bands = network.bands.write().unwrap();
		wa_bands[id as usize] = Self {
			src_clip,
			src_min: u8::MAX,
			src_max: u8::MAX,
			dst_clip,
			dst_min:
			u8::MAX,
			dst_max:
			u8::MAX,
			empty: true
		};
		id
	}
}