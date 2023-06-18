use super::{Network, CLIP_MAX_LENGTH};

#[derive(Default, Debug, Clone, Copy)]
pub struct BandIdentity {
	pub band: u32,
	pub clip: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct Band {
	pub id: u32,
	pub src_clip: u32,
	pub src_min: u8,
	pub src_max: u8,
	/// Each fixed slot in a clip can have multiple
	/// lanes splitting; This selects
	/// which one of those is part of this band.
	pub src_fixed_idx: [u8; CLIP_MAX_LENGTH],

	pub dst_clip: u32,
	pub dst_min: u8,
	pub dst_max: u8,
	/// Each fixed slot in a clip can have multiple
	/// lanes merging; This selects
	/// which one of those is part of this band.
	pub dst_fixed_idx: [u8; CLIP_MAX_LENGTH],

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
			id,
			src_clip,
			src_min: u8::MAX,
			src_max: u8::MAX,
			src_fixed_idx: [0; CLIP_MAX_LENGTH],
			dst_clip,
			dst_min: u8::MAX,
			dst_max: u8::MAX,
			dst_fixed_idx: [0; CLIP_MAX_LENGTH],
			empty: true
		};
		drop(wa_bands);
		network.clips.write().unwrap()[src_clip as usize].add_fw_band(id);
		id
	}

	pub fn default(
	) -> Self {
		Self {
			id: 0,
			src_clip: 0,
			src_min: u8::MAX,
			src_max: u8::MAX,
			src_fixed_idx: [0; CLIP_MAX_LENGTH],
			dst_clip: 0,
			dst_min: u8::MAX,
			dst_max: u8::MAX,
			dst_fixed_idx: [0; CLIP_MAX_LENGTH],
			empty: true
		}
	}
}