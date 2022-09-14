use std::collections::VecDeque;

use bevy::prelude::*;
use crate::resources::network::*;

#[derive(Component)]
pub struct VehicleComponent {
	pub entity: Entity,
	pub id: u32,

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

#[derive(Component)]
pub struct LaneComponent {
	
}

#[derive(Component)]
pub struct CameraComponent {
	
}