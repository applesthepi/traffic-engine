use std::{collections::VecDeque, sync::RwLockReadGuard};

use bevy::prelude::*;
use crate::resources::network::*;
use crate::kernel::*;

#[derive(Component)]
pub struct VehicleComponent {
	pub entity: Entity,
	pub id: u32,
	pub kernel: Kernel,

	pub speed: f32,
	pub acceleration: f32,
	pub distance: f32,

	pub target: VTarget,
	pub stage: VStage,
	pub navigation: Navigation,

	pub active_clip: u32,
	pub active_band: u32,
	pub active_lane: u32,

	pub forward_lanes: VecDeque<(u32, f32)>,
	pub forward_length: f32,
}

impl VehicleComponent {
	pub fn pull_forward_lanes(&mut self, network: &Network) {
		let new_forward = self.navigation.get_forward_lanes(
			network,
			500.0 - self.forward_length,
			match self.forward_lanes.back() {
				Some(x) => x.0,
				None => self.active_lane,
			},
			self.navigation.recent_nav + (self.forward_lanes.len() as u16)
		);
		
		for fl in new_forward.iter() {
			self.forward_length += fl.1;
			self.forward_lanes.push_back(*fl);
		}
	}
}

#[derive(Component)]
pub struct LaneComponent {
	
}

#[derive(Component)]
pub struct CameraComponent {
	
}