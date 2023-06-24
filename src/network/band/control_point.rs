use crate::network::CP_MIN_DISTANCE;

#[derive(Debug, Clone, Copy)]
pub struct BandControlPoint {
	pub forward: f32,
	pub vangle: f32,
}

impl BandControlPoint {
	pub fn new(
		forward: f32,
		vangle: f32,
	) -> Self {
		Self {
			forward,
			vangle,
		}
	}
}

impl Default for BandControlPoint {
	fn default() -> Self {
		Self {
			forward: CP_MIN_DISTANCE,
			vangle: 0.0,
		}
	}
}