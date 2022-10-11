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
			network: RwLock::new(Network::default())
		};

		network
	}
}
