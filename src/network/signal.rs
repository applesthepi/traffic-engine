/*
use core::fmt;

use async_trait::async_trait;

use super::{vehicle::{VehicleData, Vehicle, VTarget}, NetworkAllocation};

// Signals are default positioned at the end of the lane. Increasing
// activation_distance will bring the activation point backward into the lane.
#[derive(Debug, Default, Clone, Copy)]
pub struct SignalIdentity {
	pub id: u32,
	pub signal_distance: f32,
	pub active_distance: f32,
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct InstructSlow {
	pub target_speed: f32,
	pub target: VTarget
}

pub enum InstructResult {
	SLOW(InstructSlow),
	KEEP,
	DESTROY
}

#[async_trait]
pub trait Signal: Send + Sync {
	fn identity(&self) -> &SignalIdentity;
	fn identity_mut(&mut self) -> &mut SignalIdentity;
	fn activate(&mut self, allocation: &NetworkAllocation, vehicle: &Vehicle);
	fn instruct(&mut self, allocation: &NetworkAllocation, vehicle: &Vehicle) -> InstructResult;
}

impl fmt::Debug for dyn Signal {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		write!(f, "")
	}
}

#[derive(Debug, Default, Clone, Copy)]
pub struct FullStop {
	pub signal_identity: SignalIdentity,
	pub vehicle_init_speed: f32,
	pub vehicle_init_distance: f32,
}

#[async_trait]
impl Signal for FullStop {
	fn identity(&self) -> &SignalIdentity {
		&self.signal_identity
	}
	
	fn identity_mut(&mut self) -> &mut SignalIdentity {
		&mut self.signal_identity
	}

	fn activate(&mut self,
		allocation: &NetworkAllocation,
		vehicle: &Vehicle
	) {
		let stop_line_distance = vehicle.distance_from_fw(
			allocation,
			self.signal_identity.signal_distance,
			self.signal_identity.lane
		).expect("stop line not in vehicle's fw lanes");
		self.vehicle_init_speed = vehicle.data.speed;
		self.vehicle_init_distance = stop_line_distance;
	}

	fn instruct(&mut self,
		allocation: &NetworkAllocation,
		vehicle: &Vehicle
	) -> InstructResult {
		// TODO: cache propagation
		let stop_line_distance = vehicle.distance_from_fw(
			allocation,
			self.signal_identity.signal_distance,
			self.signal_identity.lane
		).expect("stop line not in vehicle's fw lanes");
		if stop_line_distance < 5.0 && vehicle.data.speed < 10.0 {
			return InstructResult::DESTROY;
		}
		let seconds_to_stationary = vehicle.data.seconds_to_stationary(stop_line_distance);
		// if seconds_to_stationary < 3.0 {
			println!("*****LESS 3 SEC*****");
			let percent = 1.0 - ((self.vehicle_init_distance - stop_line_distance) / self.vehicle_init_distance);
			return InstructResult::SLOW(InstructSlow {
				target_speed: percent * self.vehicle_init_speed,
				target: VTarget::DecTStop
			});
		// }
		// println!("*****MORE THAN 3 SEC*****");
		// InstructResult::KEEP
	}
}
*/