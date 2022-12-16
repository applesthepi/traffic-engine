use std::{collections::VecDeque, sync::{atomic::Ordering, Arc}};

use tokio::sync::RwLock;

use crate::network::network_allocation_mut;

use super::{navigation::{Navigation, ForwardLane}, Network, lane::LaneIdentity, NetworkAllocation, NetworkVertex, BATCH_COUNT};

pub enum TickStatus {
	PERSIST,
	DESTROY
}

#[derive(Debug, Default, Clone, Copy)]
pub struct VehicleIdentity {
	pub sub: u32,
	pub batch: u32,
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

#[derive(Default, Debug)]
pub struct VehicleBatch {
	pub id: u32,
	pub vehicles: Vec<Vehicle>,
}

impl VehicleBatch {
	pub fn new(id: u32) -> Self {
		let vb = VehicleBatch {
			id,
			vehicles: Vec::with_capacity(BATCH_COUNT)
		};
		vb
	}

	pub fn vehicle(&self, id: u32) -> Option<&Vehicle> {
		self.vehicles.iter().find(|x| x.data.identity.sub == id)
	}
}

// #[derive(Default, Debug)]
// pub struct VehicleInt {
// 	pub identity: VehicleIdentity,
// 	pub distance: f32,
// 	pub speed: f32,
// }

#[derive(Debug)]
pub struct Vehicle {
	pub data: VehicleData,
	pub navigation: Navigation,
	pub active_identity: LaneIdentity,
	pub driver_personality: DriverPersonality,

	pub forward_vehicles: Vec<VehicleData>,
	pub forward_lanes: VecDeque<ForwardLane>,
	pub forward_length: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct VehicleData {
	pub identity: VehicleIdentity,
	pub speed: f32,
	pub distance: f32,
	pub pdl_gas: f32,
	pub pdl_break: f32,
	pub target: VTarget,
	pub stage: VStage,
}

#[derive(Default)]
pub struct VehicleBufferInfo {
	pub vertices: Vec<NetworkVertex>,
	pub indices: Vec<u32>
}

#[derive(Debug, Copy, Clone)]
pub enum VTarget {
	Wait,
	AccFStop,
	DecTStop,
	AvgSpeed,
}

#[derive(Debug, Copy, Clone)]
pub enum VStage {
	// stopped
	Wait,
	// releasing break from stop
	LiftPush,
	// hold partial break
	LiftHold,
	// repressing break
	LiftPull,
	AccWait,
	AccPush,
	AccHold,
	AccPull,
	Maintain,
	DecPush,
	DecHold,
	DecPull,
}

#[derive(Debug)]
pub struct DriverPersonality {
	pub willing_max_accel: f32,
	pub willing_max_decel: f32,
}

impl Vehicle {
	pub async fn new(
		network: &Arc<Network>,
		src_identity: LaneIdentity,
		dst_identity: LaneIdentity
	) -> u32 {

		let mut network_c = network.clone();
		let mut allocation = network_allocation_mut!(network_c);

		// ID

		let id = allocation.vehicle_count.fetch_add(
			1,
			Ordering::SeqCst
		) + 1;
		// println!("aquired vehicle id {}", id);

		// ALLOCATION

		let mut vehicle = Self {
			data: VehicleData {
				identity: VehicleIdentity {
					sub: id,
					batch: 0,
					lane: src_identity.lane,
					band: src_identity.band,
					clip: src_identity.clip
				},
				speed: 0.0,
				pdl_gas: 0.0,
				pdl_break: 0.1,
				distance: 0.0,
				target: VTarget::AccFStop,
				stage: VStage::Wait,
			},
			driver_personality: DriverPersonality {
				willing_max_accel: 20.0,
				willing_max_decel: 50.0,
			},
			active_identity: src_identity,
			navigation: Navigation {
				nav: Vec::new(),
				nav_valid_band_lanes: Vec::new(),
				target_identity: dst_identity,
    			active_nav: 0,
			},
			forward_vehicles: Vec::new(),
			forward_lanes: VecDeque::new(),//fw_lanes,
			forward_length: 0.0,//lane.length,
		};
		vehicle.navigation.renavigate(&allocation, vehicle.active_identity).await;
		// println!("{:?}", vehicle.navigation);
		

		// SPAWN VEHICLE

		// VB  -> Vehicle Batch
		// SVB -> Staged Vehicle Batch
		// UVB -> Unused Vehicle Batch
		// println!("{:?}", allocation.vehicle_batches.read().await);
		let ra_svb_con = allocation.staged_vehicle_batch.read().await;
		let mut wa_svb = ra_svb_con.write().await;
		vehicle.data.identity.batch = wa_svb.id;
		let vehicle_data = vehicle.data.clone();
		wa_svb.vehicles.push(vehicle);
		if /*wa_svb.vehicles.len() >= BATCH_COUNT*/ true {
			drop(wa_svb);
			drop(ra_svb_con);
			allocation.cycle_svb().await;


			
		}
		
		let c_lane = allocation.lane(src_identity.lane).await;
		let mut wa_lane = c_lane.write().await;
		wa_lane.vehicles.push(vehicle_data);
		drop(wa_lane);
		drop(c_lane);
		
		// println!("{:?}", allocation.staged_vehicle_batch.read().await);
		id
	}

	pub async fn pull_forward_lanes(
		&mut self,
		allocation: &NetworkAllocation
	) {
		self.forward_lanes.clear();
		self.forward_length = 0.0;
		let new_forward = self.navigation.get_forward_lanes(
			allocation,
			500.0,
			self.active_identity.lane
		).await;
		for fl in new_forward.iter() {
			self.forward_length += fl.length;
			self.forward_lanes.push_back(fl.clone());
		}
	}

	pub async fn pull_forward_vehicles(
		&mut self,
		allocation: &NetworkAllocation
	) {
		self.forward_vehicles.clear();
		let active_lane = allocation.lane(self.active_identity.lane).await;
		let ra_active_lane = active_lane.read().await;
		// let start_dis = ra_active_lane.length - self.distance;
		let mut accumulated_distance: f32 = ra_active_lane.length - self.data.distance;
		for vehicle in ra_active_lane.vehicles.iter() {
			if vehicle.distance < self.data.distance || vehicle.identity.sub == self.data.identity.sub {
				continue;
			}
			let mut vehicle_data = vehicle.clone();
			vehicle_data.distance -= self.data.distance;
			self.forward_vehicles.push(vehicle_data);
		}

		drop(ra_active_lane);
		drop(active_lane);
		for lane in self.forward_lanes.iter() {
			let c_lane = allocation.lane(lane.id).await;
			let ra_lane = c_lane.read().await;
			for vehicle in ra_lane.vehicles.iter() {
				let mut vehicle_data = vehicle.clone();
				vehicle_data.distance += accumulated_distance;
				self.forward_vehicles.push(vehicle_data);
			}
			accumulated_distance += lane.length;
		}
	}

	pub async fn tick_temp(
		&mut self,
		allocation: &NetworkAllocation,
		delta_time: f32
	) -> TickStatus {
		
		// println!("{:?}", self.navigation);
		self.data.distance += self.data.speed * delta_time;
		// self.speed += self.acceleration * delta_time;

		// TEMP: MOVE TO BEST LANE

		self.pull_forward_lanes(allocation).await;

		// if /*self.forward_lanes.len() <= 1*/ false {  
		// 	let c_clip = allocation.clip(self.active_identity.clip).await;
		// 	let ra_clip = c_clip.read().await;
		// 	let mut best_id: u32 = 0;
		// 	let mut best_diff: i8 = i8::MAX;


		// 	let fixed_current = ra_clip.lanes_fixed.iter().enumerate().find(
		// 		|x|
		// 		{
		// 			let f = x.1;
		// 			for i in 0..f.fw_count {
		// 				if f.fw[i as usize] == self.active_identity.lane {
		// 					return true;
		// 				}
		// 			}
		// 			false
		// 		}
		// 	).expect("target lane does not exist");



		// 	// let fixed_current = ra_clip.lanes_fixed.iter().enumerate().find(
		// 	// 	|x|
		// 	// 	x.1.fw == self.active_identity.lane
		// 	// ).expect("vehicle active lane does not belong to active clip");
		// 	for lane_id in self.navigation.nav_valid_band_lanes[self.navigation.active_nav as usize].iter() {
		// 		if let Some(fixed_valid) = ra_clip.lanes_fixed.iter().enumerate().find(
		// 			|x|
		// 			{
		// 				let f = x.1;
		// 				for i in 0..f.fw_count {
		// 					if f.fw[i as usize] == *lane_id {
		// 						return true;
		// 					}
		// 				}
		// 				false
		// 			}
		// 		) {
		// 			let diff_abs: i8 = ((fixed_valid.0 as i8) - (fixed_current.0 as i8)).abs();
		// 			if diff_abs < best_diff as i8 {
		// 				best_diff = diff_abs;
		// 				best_id = *lane_id;
		// 			}
		// 		}
		// 	}

		// 	if best_id > 0 && best_id != self.active_identity.lane {
		// 		self.active_identity.lane = best_id;
		// 		println!("moving to best lane");
		// 	}
		// }

		let mut lane: u32 = self.active_identity.lane;

		// let mut lane = match network.alloc_lanes.get_mut(&vehicle.active_lane) {
		// 	Some(x) => x,
		// 	None => {
		// 		commands.entity(vehicle.entity).despawn();
		// 		println!("despawning: active lane is invalid");
		// 		continue;
		// 	},
		// };

		let c_lane = allocation.lane(lane).await;
		let mut wa_lane = c_lane.write().await;
		let lane_speed: f32 = 100.0;
		let mut vehicle_int = wa_lane.vehicles.iter_mut().find(
			|x|
			x.identity.sub == self.data.identity.sub
		).expect("vehicle does not exist in lane");
		vehicle_int.distance = self.data.distance;
		vehicle_int.speed = self.data.speed;
		if self.data.distance < wa_lane.length {
			drop(wa_lane);
			drop(c_lane);
			return self.tick_st(allocation, delta_time, lane_speed).await;
		}

		if self.forward_lanes.is_empty() {
			if let Some(idx) = wa_lane.vehicles.iter().position(
				|x|
				x.identity.sub == self.data.identity.sub
			) {
				wa_lane.vehicles.swap_remove(idx);
			} else {
				println!("failed to remove vehicle from lane; id does not exist in specified lane");
			}
			return TickStatus::DESTROY;
			// let ra_vb_map = allocation.vehicle_batches.read().await;
			// let vb_c = ra_vb_map.get(&self.bid).expect("invalid b-id").clone();
			// let mut wa_vb = vb_c.write().await;
			// let vehicle_idx = wa_vb.vehicles.iter().enumerate().find(
			// 	|x|
			// 	x.1.id == self.id
			// ).expect("vehicle does not exist in vehicle batch").0;
			// wa_vb.vehicles.remove(vehicle_idx);
			// return;
		}

		drop(wa_lane);
		drop(c_lane);

		loop {
			let c_lane = allocation.lane(lane).await;
			let mut wa_lane = c_lane.write().await;

			if let Some(idx) = wa_lane.vehicles.iter().position(
				|x|
				x.identity.sub == self.data.identity.sub
			) {
				wa_lane.vehicles.swap_remove(idx);
			} else {
				println!("failed to remove vehicle from lane; id does not exist in specified lane");
			}
			self.data.distance -= wa_lane.length;
			// if self.forward_lanes.is_empty() {
			// 	// let a = &mut *network;
			// 	self.pull_forward_lanes(allocation).await;
			// 	println!("pulled forward lanes because vehicle needs more {:?}", self.forward_lanes);
			// }
			let fw_lane = match self.forward_lanes.pop_front() {
				Some(x) => x,
				None => {
					// commands.entity(self.entity).despawn();
					println!("despawning: no forward lanes ");
					break;
				}
			};
			self.active_identity.lane = fw_lane.id;
			self.data.identity.lane = fw_lane.id;
			lane = fw_lane.id;
			drop(wa_lane);
			drop(c_lane);
			let c_lane = allocation.lane(lane).await;
			let mut wa_lane = c_lane.write().await;
			self.active_identity.band = wa_lane.identity.band;
			self.active_identity.clip = wa_lane.identity.clip;
			self.data.identity.band = wa_lane.identity.band;
			self.data.identity.clip = wa_lane.identity.clip;
			self.forward_length -= fw_lane.length;
			wa_lane.vehicles.push(self.data.clone());
			// println!("inc active nav to {}", self.navigation.active_nav);

			// let v_clip = self.active_identity.clip;
			// let v_band = self.active_identity.band;
			// let v_lane = self.active_identity.lane;

			// vehicle.pull_forward_lanes(&network);

			// if self.navigation.nav[(self.navigation.active_nav) as usize].clip != v_clip ||
			// 	self.navigation.nav[(self.navigation.active_nav) as usize].band != v_band ||
			// 	self.navigation.nav_valid_band_lanes[(self.navigation.active_nav) as usize].iter().find(|&&x| x == v_lane) == None {
				
			// 	// if !vehicle.navigation.renavigate(&network, v_clip, v_band, v_lane) {
			// 	// 	commands.entity(vehicle.entity).despawn();
			// 	// 	println!("despawning: invalid navigation");
			// 	// 	break;
			// 	// }
			// }

			// match vehicle.
			self.navigation.active_nav += 1;
			if wa_lane.length > self.data.distance {
				break;
			}
		}

		// TARGET AND STAGE

		self.tick_st(allocation, delta_time, lane_speed).await
		
		// let v_pos = lane.get_interp_position(self.distance);

		// transform.rotation = Quat::from_rotation_z(0.05);
		// transform.translation = Vec3::new(v_pos.x, v_pos.y, 1.0);
	}

	async fn tick_st(
		&mut self,
		allocation: &NetworkAllocation,
		delta_time: f32,
		lane_speed: f32
	) -> TickStatus {
		self.pull_forward_vehicles(allocation).await;
		if self.forward_vehicles.is_empty() {
			println!("fwv dis: NONE");
			self.update_stage(delta_time, lane_speed);
			return TickStatus::PERSIST;
		}
		self.update_target();
		let fw_vehicle = self.forward_vehicles.first().unwrap();
		let vb = allocation.vehicle_batch(fw_vehicle.identity.batch).await;
		let ra_vb = vb.read().await;
		let delta_speed = fw_vehicle.speed - self.data.speed;
		let seconds_to_vehicle = delta_speed / fw_vehicle.distance;
		match self.data.target {
			VTarget::Wait => {},
			VTarget::AccFStop => {
				self.update_stage(delta_time, lane_speed.min(fw_vehicle.speed));
			},
			VTarget::DecTStop => {
				self.update_stage(delta_time, self.data.speed * (fw_vehicle.distance / 10.0));
			},
			VTarget::AvgSpeed => {
				let focus_out = ((seconds_to_vehicle - 2.0) * 0.25).clamp(0.0, 1.0);
				let target_speed = (fw_vehicle.speed * (1.0 - focus_out)) + (lane_speed * focus_out);
				self.update_stage(delta_time, target_speed);
			}
		};

		TickStatus::PERSIST
	}

	pub fn update_target(
		&mut self
	) {
		let fw_vehicle = self.forward_vehicles.first().unwrap();
		match self.data.target {
			VTarget::Wait => {
				if fw_vehicle.distance > 10.0 ||
					(fw_vehicle.speed - self.data.speed) > 5.0 {
					self.data.target = VTarget::AccFStop;
					self.update_target();
					return;
				}
			},
			VTarget::AccFStop => {
				// Accelerating, but fw vehicle is decelerating.
				// CLONE TODO: OUT SOURCE TO FUNCTION
				if fw_vehicle.distance < 100.0 &&
					(fw_vehicle.speed - self.data.speed) < -5.0 {
					match fw_vehicle.target {
						VTarget::DecTStop => {
							self.data.target = VTarget::DecTStop;
							self.update_target();
							return;
						},
						_ => {
							self.data.target = VTarget::AvgSpeed;
							self.update_target();
							return;
						}
					}
				}
				if fw_vehicle.distance < 30.0 &&
					(fw_vehicle.speed - self.data.speed) < 1.0 {
					self.data.target = VTarget::AvgSpeed;
					self.update_target();
					return;
				}
			},
			VTarget::DecTStop => {
				if self.data.speed == 0.0 {
					self.data.target = VTarget::Wait;
				}
			},
			VTarget::AvgSpeed => {
				if (fw_vehicle.speed - self.data.speed) > 10.0 {
					self.data.target = VTarget::AccFStop;
					self.update_target();
					return;
				}
				// CLONE TODO: OUT SOURCE TO FUNCTION
				if fw_vehicle.distance < 100.0 &&
					(fw_vehicle.speed - self.data.speed) < -5.0 {
					match fw_vehicle.target {
						VTarget::DecTStop => {
							self.data.target = VTarget::DecTStop;
							self.update_target();
							return;
						},
						_ => {}
					}
				}
			}
		}
	}

	pub fn update_stage(
		&mut self,
		delta_time: f32,
		target_speed: f32
	) {
		let tolerance: f32 = 5.0;
		let desired_delta = target_speed - self.data.speed;
		println!("stage: {:?}", self.data.stage);
		println!("desired_delta: {}", desired_delta);
		match self.data.stage {
			VStage::Wait => {
				if desired_delta > tolerance {
					self.data.stage = VStage::LiftPush;
					self.update_stage(delta_time, target_speed);
					return;
				}
			},
			VStage::LiftPush => {
				if desired_delta >= 0.0 && desired_delta < tolerance {
					self.data.stage = VStage::LiftHold;
					return;
				}
				if desired_delta < 0.0 {
					self.data.stage = VStage::LiftPull;
					self.update_stage(delta_time, target_speed);
					return;
				}
				self.data.pdl_break -= delta_time * (0.001 * desired_delta).clamp(0.0, 0.1);
				if self.data.pdl_break < 0.0 {
					self.data.pdl_break = 0.0;
					self.data.stage = VStage::AccWait;
				}
			},
			VStage::LiftHold => {
				if desired_delta >= 0.0 && desired_delta < tolerance {
					return;
				}
				if desired_delta < 0.0 {
					self.data.stage = VStage::LiftPull;
					self.update_stage(delta_time, target_speed);
					return;
				}
				// desired_delta > tolerance
				self.data.stage = VStage::LiftPush;
				self.update_stage(delta_time, target_speed);
				return;
			},
			VStage::LiftPull => {
				if desired_delta >= 0.0 {
					self.data.stage = VStage::LiftHold;
					self.update_stage(delta_time, target_speed);
					return;
				}
				self.data.pdl_break -= delta_time * (0.05 * desired_delta).clamp(-2.0, 0.0);
				if self.data.pdl_break > 1.0 {
					self.data.pdl_break = 1.0;
				}
			},
			VStage::AccWait => {

			},
			VStage::AccPush => {

			},
			VStage::AccHold => {

			},
			VStage::AccPull => {

			},
			VStage::Maintain => {

			},
			VStage::DecPush => {

			},
			VStage::DecHold => {

			},
			VStage::DecPull => {

			},
		}
		println!("  gas: {}", self.data.pdl_gas);
		println!("break: {}", self.data.pdl_break);
		self.data.speed += delta_time * self.data.pdl_gas * self.driver_personality.willing_max_accel;
		
		let mut decel_pedal = self.data.pdl_break - 0.1;
		if decel_pedal < 0.0 {
			// idle forward (no break)
			let delta_idle = (150.0 - (self.data.speed * 10.0)).clamp(0.0, 150.0) * 3.0;
			decel_pedal *= delta_idle;
		} else {
			// breaking
			decel_pedal *= self.driver_personality.willing_max_decel;
		}
		let delta_speed = decel_pedal;
		println!("delta_speed: {}", delta_speed);
		self.data.speed -= delta_time * delta_speed;//((self.pdl_break - 0.1) * self.driver_personality.willing_max_decel).clamp((self.speed - 20.0).clamp(-20.0, 0.0), 1.0);
		if self.data.speed <= 0.0 {
			self.data.speed = 0.0;
			self.data.stage = VStage::Wait;
		}
	}

	
}