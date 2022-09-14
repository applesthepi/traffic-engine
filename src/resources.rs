use std::{sync::RwLock, collections::HashMap};

use bevy::prelude::*;

pub mod network;
use network::*;

pub struct NetworkResource {
	pub network: RwLock<Network>,
}

impl FromWorld for NetworkResource {
	fn from_world(world: &mut World) -> Self {
		let mut network = NetworkResource {
			network: RwLock::new(Network {
				alloc_clips: HashMap::new(),
				alloc_bands: HashMap::new(),
				alloc_lanes: HashMap::new(),
				start_clips: Vec::new(),

				clip_count: 0,
				band_count: 0,
				lane_count: 0,
				vehicle_count: 0,
			})
		};

		network
	}
}