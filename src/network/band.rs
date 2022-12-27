use std::sync::{atomic::Ordering, Arc, RwLock};

use crate::network_allocation;

use super::{Network};

#[derive(Default, Debug, Clone)]
pub struct BandIdentity {
	pub band: u32,
	pub clip: u32,
}

#[derive(Debug)]
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
		network: &Arc<Network>,
		src_clip: u32, dst_clip: u32
	) -> u32 {

		let network_c = network.clone();
		let allocation = network_allocation!(network_c);

		// ID

		let id = allocation.band_count.fetch_add(
			1,
			Ordering::Relaxed
		) + 1;
		// println!("aquired band id {}", id);

		// ALLOCATION

		allocation.bands.write().unwrap().insert(id, Arc::new(
			RwLock::new(Self {
				src_clip,
				src_min: u8::MAX,
				src_max: u8::MAX,
				dst_clip,
				dst_min:
				u8::MAX,
				dst_max:
				u8::MAX,
				empty: true
			})
		));

		id
	}
}