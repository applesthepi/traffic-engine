use std::{collections::VecDeque, sync::atomic::Ordering};

use super::{navigation::{Navigation, ForwardLane}, Network, lane::LaneIdentity, NetworkAllocation};

#[derive(Debug)]
pub struct VehicleIdentity {
	pub sub: u32,
	pub batch: u32,
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

#[derive(Default)]
pub struct VehicleBatch {
	pub vehicles: Vec<Vehicle>,
}

#[derive(Default, Debug)]
pub struct VehicleInt {
	pub id: u32,
	pub distance: f32,
}

pub struct Vehicle {
	pub id: u32,
	pub driver_personality: DriverPersonality,

	pub speed: f32,
	pub acceleration: f32,
	pub distance: f32,

	pub target: VTarget,
	pub stage: VStage,
	pub navigation: Navigation,

	pub active_identity: LaneIdentity,

	pub forward_lanes: VecDeque<ForwardLane>,
	pub forward_length: f32,
}

pub enum VTarget {
	AccFStop,
	DecTStop,
	AvgSpeed,
}

pub enum VStage {
	Wait,
	LiftPush,
	LiftHold,
	LiftPull,
	AccWait,
	AccPush,
	AccHold,
	AccPull,
	AccLift,
	Maintain,
	DecPush,
	DecHold,
	DecPull,
	DecLift,
}

pub struct DriverPersonality {
	pub wait_exp_0: f32,
	pub wait_exp_1: f32,
	pub wait_releases: u8,
}

impl Vehicle {
	pub fn new(
		network: &mut Network,
		src_identity: LaneIdentity,
		dst_identity: LaneIdentity
	) -> u32 {

		let allocation = network.allocation.clone();

		// ID

		let id = network.vehicle_count.fetch_add(
			1,
			Ordering::Relaxed
		) + 1;
		println!("aquired vehicle id {}", id);

		// ALLOCATION

		let mut vehicle = Self {
			id,
			driver_personality: DriverPersonality {
				wait_exp_0: 1.0,
				wait_exp_1: 3.0,
				wait_releases: 0x1 | 0x2 | 0x4 | 0x8 | 0x10 | 0x20,
			},
			speed: 5.0,
			acceleration: 0.0,
			target: VTarget::AccFStop,
			stage: VStage::Wait,
			active_identity: src_identity,
			distance: 0.0,
			navigation: Navigation {
				nav: Vec::new(),
				nav_valid_band_lanes: Vec::new(),
				target_identity: dst_identity,
    			active_nav: 0,
			},
			forward_lanes: VecDeque::new(),//fw_lanes,
			forward_length: 0.0,//lane.length,
		};
		vehicle.navigation.renavigate(&allocation, vehicle.active_identity);

		id
	}

	pub fn tick_temp(
		&mut self,
		allocation: &NetworkAllocation,
		delta_time: f32
	) {
		self.distance += self.speed * delta_time;
		self.speed += self.acceleration * delta_time;

		// TEMP: MOVE TO BEST LANE

		if self.forward_lanes.len() <= 1 {  
			let c_clip = allocation.clip(self.active_identity.clip);
			let ra_clip = c_clip.read();
			let mut best_id: u32 = 0;
			let mut best_diff: i8 = i8::MAX;
			let fixed_current = ra_clip.lanes_fixed.iter().enumerate().find(
				|x|
				x.1.fw == self.active_identity.lane
			).expect("vehicle active lane does not belong to active clip");
			for lane_id in self.navigation.nav_valid_band_lanes[self.navigation.active_nav as usize].iter() {
				if let Some(fixed_valid) = ra_clip.lanes_fixed.iter().enumerate().find(
					|x|
					x.1.fw == *lane_id
				) {
					let diff_abs: i8 = ((fixed_valid.0 as i8) - (fixed_current.0 as i8)).abs();
					if diff_abs < best_diff as i8 {
						best_diff = diff_abs;
						best_id = *lane_id;
					}
				}
			}

			if best_id > 0 && best_id != self.active_identity.lane {
				self.active_identity.lane = best_id;
				println!("moving to best lane");
			}
		}

		let mut lane: u32 = self.active_identity.lane;

		// let mut lane = match network.alloc_lanes.get_mut(&vehicle.active_lane) {
		// 	Some(x) => x,
		// 	None => {
		// 		commands.entity(vehicle.entity).despawn();
		// 		println!("despawning: active lane is invalid");
		// 		continue;
		// 	},
		// };
		loop {
			let c_lane = allocation.lane(lane);
			let mut wa_lane = c_lane.write();

			if let Some(idx) = wa_lane.vehicles.iter().position(
				|x|
				x.id == self.id
			) {
				wa_lane.vehicles.swap_remove(idx);
			} else {
				println!("failed to remove vehicle from lane; id does not exist in specified lane");
			}
			self.distance -= wa_lane.length;
			if self.forward_lanes.is_empty() {
				// let a = &mut *network;
				self.pull_forward_lanes(allocation);
				println!("pulled forward lanes because vehicle needs more {:?}", self.forward_lanes);
			}
			let fw_lane = match self.forward_lanes.pop_front() {
				Some(x) => x,
				None => {
					// commands.entity(self.entity).despawn();
					println!("despawning: no forward lanes ");
					break;
				}
			};
			self.active_identity.lane = fw_lane.id;
			lane = self.active_identity.lane;

			drop(wa_lane);
			drop(c_lane);

			let c_lane = allocation.lane(lane);
			let mut wa_lane = c_lane.write();

			wa_lane.vehicles.push(VehicleInt {
				id: self.id,
				distance: self.distance,
			});
			self.active_identity.band = wa_lane.identity.band;
			self.active_identity.clip = wa_lane.identity.clip;
			self.forward_length -= fw_lane.length;
			self.navigation.active_nav += 1;
			println!("inc active nav to {}", self.navigation.active_nav);

			let v_clip = self.active_identity.clip;
			let v_band = self.active_identity.band;
			let v_lane = self.active_identity.lane;

			// vehicle.pull_forward_lanes(&network);

			if self.navigation.nav[(self.navigation.active_nav) as usize].clip != v_clip ||
				self.navigation.nav[(self.navigation.active_nav) as usize].band != v_band ||
				self.navigation.nav_valid_band_lanes[(self.navigation.active_nav) as usize].iter().find(|&&x| x == v_lane) == None {
				
				// if !vehicle.navigation.renavigate(&network, v_clip, v_band, v_lane) {
				// 	commands.entity(vehicle.entity).despawn();
				// 	println!("despawning: invalid navigation");
				// 	break;
				// }
			}

			// match vehicle.

			if wa_lane.length > self.distance {
				break;
			}
		}
		
		// let v_pos = lane.get_interp_position(self.distance);

		// transform.rotation = Quat::from_rotation_z(0.05);
		// transform.translation = Vec3::new(v_pos.x, v_pos.y, 1.0);
	}

	pub fn pull_forward_lanes(
		&mut self,
		allocation: &NetworkAllocation
	) {
		self.forward_lanes.clear();
		self.forward_length = 0.0;
		let new_forward = self.navigation.get_forward_lanes(
			allocation,
			500.0,
			match self.forward_lanes.back() {
				Some(x) => x.id,
				None => self.active_identity.lane,
			}
		);
		for fl in new_forward.iter() {
			self.forward_length += fl.length;
			self.forward_lanes.push_back(fl.clone());
		}
	}
}