use crate::network::CLIP_MAX_LENGTH;

use super::control_point::BandControlPoint;

#[derive(Debug, Clone, Copy)]
/// Controls MUST be ordered first to last!
/// Each band can have multiple beziars that apply
/// to the lanes later on. Within the band, multiple
/// controls are needed to determin which lanes are
/// grouped with the same root beziar.
pub struct Control {
	pub empty: bool,

	pub src_min: u8,
	pub src_max: u8,

	pub dst_min: u8,
	pub dst_max: u8,

	pub lane_src_lnum: [u8; CLIP_MAX_LENGTH],
	pub lane_src_fixed_idx: [u8; CLIP_MAX_LENGTH],

	pub c1: BandControlPoint,
	pub c2: BandControlPoint,
}

impl Default for Control {
	fn default() -> Self {
		Self {
			empty: true,
			src_min: u8::MAX,
			src_max: u8::MAX,
			dst_min: u8::MAX,
			dst_max: u8::MAX,
			lane_src_lnum: [u8::MAX; CLIP_MAX_LENGTH],
			lane_src_fixed_idx: [u8::MAX; CLIP_MAX_LENGTH],
			c1: BandControlPoint::default(),
			c2: BandControlPoint::default(),
		}
	}
}