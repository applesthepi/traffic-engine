use std::sync::{atomic::Ordering, Arc};

use tokio::sync::RwLock;

use crate::network_allocation;

use super::{Network, network_allocation_mut};

#[derive(Debug, Default, Clone)]
pub struct Fixed {
	pub fw_count: u8,
	pub bw_count: u8,
	pub fw: [u32; 5],
	pub bw: [u32; 5],
}

#[derive(Debug, Default)]
pub struct Clip {
	// Fixed size of how long the clip is. Forward then back.
	pub lanes_fixed: Vec<Fixed>,
	pub fw_bands: Vec<u32>,
}

impl Clip {
	pub async fn new(
		network: &Arc<Network>
	) -> u32 {

		let network_c = network.clone();
		let allocation = network_allocation!(network_c);

		// ID

		let id = allocation.clip_count.fetch_add(
			1,
			Ordering::Relaxed
		) + 1;
		// println!("aquired clip id {}", id);

		// ALLOCATION

		allocation.clips.write().await.insert(id, Arc::new(
			RwLock::new(Self::default())
		));

		id
	}
}