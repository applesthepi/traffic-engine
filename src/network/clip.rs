use std::sync::{atomic::Ordering, Arc};

use parking_lot::RwLock;

use super::Network;

#[derive(Debug, Default, Clone)]
pub struct Fixed {
	pub fw: u32,
	pub bw: u32,
}

#[derive(Debug, Default)]
pub struct Clip {
	// Fixed size of how long the clip is. Forward then back.
	pub lanes_fixed: Vec<Fixed>,
	pub fw_bands: Vec<u32>,
}

impl Clip {
	pub fn new(
		network: &mut Network
	) -> u32 {

		let allocation = network.allocation.clone();

		// ID

		let id = network.clip_count.fetch_add(
			1,
			Ordering::Relaxed
		) + 1;
		println!("aquired clip id {}", id);

		// ALLOCATION

		allocation.clips.write().insert(id, Arc::new(
			RwLock::new(Self::default())
		));

		id
	}
}