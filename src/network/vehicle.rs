use std::{collections::VecDeque, sync::{atomic::Ordering, Arc}};



use crate::network::{network_allocation_mut, signal::InstructResult};

use super::{navigation::{Navigation, ForwardLane}, Network, lane::LaneIdentity, NetworkAllocation, NetworkVertex, BATCH_COUNT, signal::{Signal, InstructSlow}};

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

// #[derive(Debug, Default)]
// pub struct GpuVehicle {
// 	// pub 
// }

#[derive(Debug, Default)]
pub struct Vehicle {
	pub data: VehicleData,
	pub navigation: Navigation,
	pub active_identity: LaneIdentity,
	pub driver_personality: DriverPersonality,

	pub active_signals: Vec<Arc<dyn Signal>>,
	pub forward_signals: Vec<Arc<dyn Signal>>,
	pub forward_vehicles: Vec<VehicleData>,
	pub forward_lanes: VecDeque<ForwardLane>,
	pub forward_length: f32,
	
	last_forward_signals: Vec<Arc<dyn Signal>>,
	destroyed_active_signals: Vec<Arc<dyn Signal>>,
	signal_instructs: Vec<InstructSlow>,
	last_desired_delta: f32,
}

#[derive(Debug, Default, Copy, Clone)]
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

#[derive(Debug, Default, Copy, Clone)]
pub enum VTarget {
	#[default]
	Wait,
	AccFStop,
	DecTStop,
	AvgSpeed,
}

#[derive(Debug, Default, Copy, Clone)]
pub enum VStage {
	// stopped
	#[default]
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

#[derive(Debug, Default)]
pub struct DriverPersonality {
	pub willing_max_accel: f32,
	pub willing_max_decel: f32,
}

impl Vehicle {
	pub fn new(
		network: &Arc<Network>,
		src_identity: LaneIdentity,
		dst_identity: LaneIdentity
	) -> VehicleIdentity {

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
				pdl_gas: 0.0,
				pdl_break: 0.1,
				..Default::default()
			},
			driver_personality: DriverPersonality {
				willing_max_accel: 20.0,
				willing_max_decel: 50.0,
			},
			active_identity: src_identity,
			navigation: Navigation {
				target_identity: dst_identity,
				..Default::default()
			},
			..Default::default()
		};
		vehicle.navigation.renavigate(&allocation, vehicle.active_identity);
		// println!("{:?}", vehicle.navigation);
		

		// SPAWN VEHICLE

		// VB  -> Vehicle Batch
		// SVB -> Staged Vehicle Batch
		// UVB -> Unused Vehicle Batch
		// println!("{:?}", allocation.vehicle_batches.read());
		let ra_svb_con = allocation.staged_vehicle_batch.read().unwrap();
		let mut wa_svb = ra_svb_con.write().unwrap();
		vehicle.data.identity.batch = wa_svb.id;
		let vehicle_data = vehicle.data.clone();
		wa_svb.vehicles.push(vehicle);
		if /*wa_svb.vehicles.len() >= BATCH_COUNT*/ true {
			drop(wa_svb);
			drop(ra_svb_con);
			allocation.cycle_svb();


			
		}
		
		let c_lane = allocation.lane(src_identity.lane);
		let mut wa_lane = c_lane.write().unwrap();
		wa_lane.vehicles.push(vehicle_data);
		drop(wa_lane);
		drop(c_lane);
		
		// println!("{:?}", allocation.staged_vehicle_batch.read());
		vehicle_data.identity
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
			self.active_identity.lane
		);
		for fl in new_forward.iter() {
			self.forward_length += fl.length;
			self.forward_lanes.push_back(fl.clone());
		}
	}

	pub fn pull_forward_vehicles(
		&mut self,
		allocation: &NetworkAllocation
	) {
		self.forward_vehicles.clear();
		let active_lane = allocation.lane(self.active_identity.lane);
		let ra_active_lane = active_lane.read().unwrap();
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
			let c_lane = allocation.lane(lane.id);
			let ra_lane = c_lane.read().unwrap();
			for vehicle in ra_lane.vehicles.iter() {
				let mut vehicle_data = vehicle.clone();
				vehicle_data.distance += accumulated_distance;
				self.forward_vehicles.push(vehicle_data);
			}
			accumulated_distance += lane.length;
		}
	}

	pub fn pull_forward_signals(
		&mut self,
		allocation: &NetworkAllocation
	) {
		// PROPAGATE IDENTITY LANE

		self.last_forward_signals.clear();
		for signal in self.forward_signals.iter() {
			self.last_forward_signals.push(signal.clone());
		}
		self.forward_signals.clear();
		let active_lane = allocation.lane(self.active_identity.lane);
		let ra_active_lane = active_lane.read().unwrap();
		let mut accumulated_distance: f32 = ra_active_lane.length - self.data.distance;
		for signal in ra_active_lane.signals.iter() {
			let signal_identity = signal.identity();
			if signal_identity.signal_distance - self.data.distance > signal_identity.active_distance {
				self.forward_signals.push(signal.clone());
			}
		}

		// PROPAGATE FORWARD LANES
		drop(ra_active_lane);
		drop(active_lane);
		for lane in self.forward_lanes.iter() {
			let c_lane = allocation.lane(lane.id);
			let ra_lane = c_lane.read().unwrap();
			for signal in ra_lane.signals.iter() {
				let signal_identity = signal.identity();
				if signal_identity.signal_distance + accumulated_distance > signal_identity.active_distance {
					self.forward_signals.push(signal.clone());
				}
			}
			accumulated_distance += lane.length;
		}

		// ACTIVATE SIGNAL

		// TODO: Bugs possible when comparing last fw lanes to fw lanes. Make more explicit.
		// TODO: Active signal detection can be optimized
		for lfw_signal in self.last_forward_signals.iter() {
			let fw_signal = self.forward_signals.iter().find(
				|x|
				x.identity().id == lfw_signal.identity().id
			);
			if fw_signal.is_none() {
				let mut signal = lfw_signal.clone();
				unsafe {
					let wa_signal = Arc::get_mut_unchecked(&mut signal);
					wa_signal.activate(allocation, &self);
				}
				self.active_signals.push(signal);
			}
		}
	}

	pub fn tick_temp(
		&mut self,
		allocation: &NetworkAllocation,
		delta_time: f32
	) -> TickStatus {
		
		// println!("{:?}", self.navigation);
		self.data.distance += self.data.speed * delta_time;
		// self.speed += self.acceleration * delta_time;

		// TEMP: MOVE TO BEST LANE

		self.pull_forward_lanes(allocation);

		// if /*self.forward_lanes.len() <= 1*/ false {  
		// 	let c_clip = allocation.clip(self.active_identity.clip);
		// 	let ra_clip = c_clip.read();
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

		let c_lane = allocation.lane(lane);
		let mut wa_lane = c_lane.write().unwrap();
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
			return self.tick_st(allocation, delta_time, lane_speed);
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
			// let ra_vb_map = allocation.vehicle_batches.read();
			// let vb_c = ra_vb_map.get(&self.bid).expect("invalid b-id").clone();
			// let mut wa_vb = vb_c.write();
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
			let c_lane = allocation.lane(lane);
			let mut wa_lane = c_lane.write().unwrap();

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
			// 	self.pull_forward_lanes(allocation);
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
			let c_lane = allocation.lane(lane);
			let mut wa_lane = c_lane.write().unwrap();
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

		self.tick_st(allocation, delta_time, lane_speed)
		
		// let v_pos = lane.get_interp_position(self.distance);

		// transform.rotation = Quat::from_rotation_z(0.05);
		// transform.translation = Vec3::new(v_pos.x, v_pos.y, 1.0);
	}

	fn tick_st(
		&mut self,
		allocation: &NetworkAllocation,
		delta_time: f32,
		lane_speed: f32
	) -> TickStatus {
		self.pull_forward_vehicles(allocation);
		self.pull_forward_signals(allocation);
		if self.forward_vehicles.is_empty() {
			println!("fwv dis: NONE");
			self.update_target_solo(lane_speed);
			let signal_instruct = self.calc_signal_target(allocation, lane_speed);
			if signal_instruct.target_speed < lane_speed {
				self.data.target = signal_instruct.target;
				self.update_stage(delta_time, signal_instruct.target_speed);
			} else {
				println!("%%%%%%%%%%%\n%%%%%%%%%%%\n%%%%%%%%%");
				self.update_stage(delta_time, lane_speed);
			}
			return TickStatus::PERSIST;
		}
		self.update_target_fw();
		let signal_instruct = self.calc_signal_target(allocation, lane_speed);
		let fw_vehicle = self.forward_vehicles.first().unwrap();
		let vb = allocation.vehicle_batch(fw_vehicle.identity.batch);
		let ra_vb = vb.read().unwrap();
		let seconds_to_vehicle = self.data.seconds_to_moving(fw_vehicle.distance, fw_vehicle.speed);
		match self.data.target {
			VTarget::Wait => {},
			// VTarget::AccFStop => {
			// 	let focus
			// 	self.update_stage(delta_time, lane_speed.min(fw_vehicle.speed));
			// },
			VTarget::DecTStop => {
				let target_speed = self.data.speed * (fw_vehicle.distance / 10.0);
				let min_speed = signal_instruct.target_speed.min(target_speed);
				if min_speed == signal_instruct.target_speed {
					self.data.target = signal_instruct.target;
				}
				self.update_stage(delta_time, min_speed);
			},
			VTarget::AvgSpeed | VTarget::AccFStop => {
				let focus_out = ((seconds_to_vehicle - 2.0) * 0.25).clamp(0.0, 1.0);
				let target_speed = (fw_vehicle.speed * (1.0 - focus_out)) + (lane_speed * focus_out);
				if signal_instruct.target_speed < target_speed {
					self.data.target = signal_instruct.target;
					self.update_stage(delta_time, signal_instruct.target_speed);
				} else {
					self.update_stage(delta_time, target_speed);
				}
			}
		};

		TickStatus::PERSIST
	}

	#[allow(unreachable_code)]
	pub fn update_target_solo(
		&mut self,
		lane_speed: f32
	) {
		if (lane_speed - self.data.speed).abs() < 1.0 {
			self.data.target = VTarget::AvgSpeed;
		} else {
			self.data.target = VTarget::AccFStop;
		}
		println!("target: {:?}", self.data.target);
		return;
		match self.data.target {
			VTarget::Wait => {},
			VTarget::AccFStop => {},
			VTarget::DecTStop => {},
			VTarget::AvgSpeed => {}
		};
		//println!("target: {:?}", self.data.target);
	}

	pub fn update_target_fw(
		&mut self,
	) {
		let fw_vehicle = self.forward_vehicles.first().unwrap();
		match self.data.target {
			VTarget::Wait => {
				if fw_vehicle.distance > 10.0 ||
					(fw_vehicle.speed - self.data.speed) > 5.0 {
					self.data.target = VTarget::AccFStop;
					self.update_target_fw();
					return;
				}
			},
			VTarget::AccFStop => {
				// Accelerating, but fw vehicle is decelerating.
				// CLONE; TODO: OUT SOURCE TO FUNCTION
				if fw_vehicle.distance < 100.0 &&
					(fw_vehicle.speed - self.data.speed) < -5.0 {
					match fw_vehicle.target {
						VTarget::DecTStop => {
							self.data.target = VTarget::DecTStop;
							self.update_target_fw();
							return;
						},
						_ => {
							self.data.target = VTarget::AvgSpeed;
							self.update_target_fw();
							return;
						}
					}
				}
				// close to fw, but not going anywhere.
				if fw_vehicle.distance < 30.0 &&
					(fw_vehicle.speed - self.data.speed) < 1.0 {
					self.data.target = VTarget::AvgSpeed;
					self.update_target_fw();
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
					self.update_target_fw();
					return;
				}
				// CLONE; TODO: OUT SOURCE TO FUNCTION
				if fw_vehicle.distance < 100.0 &&
					(fw_vehicle.speed - self.data.speed) < -5.0 {
					match fw_vehicle.target {
						VTarget::DecTStop => {
							self.data.target = VTarget::DecTStop;
							self.update_target_fw();
							return;
						},
						_ => {}
					}
				}
			}
		};
		println!("target: {:?}", self.data.target);
	}

	pub fn update_stage(
		&mut self,
		delta_time: f32,
		target_speed: f32
	) {
		let tolerance: f32 = 0.01;
		let desired_delta = target_speed - self.data.speed;
		let delta_delta = desired_delta - self.last_desired_delta;
		self.last_desired_delta = desired_delta;
		println!("stage: {:?}", self.data.stage);
		println!("target_speed: {}", target_speed);
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
				// RELEASING BREAK
				self.data.pdl_break -= delta_time * (0.05 * desired_delta).clamp(0.0, 1.0);
				if self.data.pdl_break < 0.0 {
					self.data.pdl_break = 0.0;
					self.data.stage = VStage::AccWait;
				}
			},
			VStage::LiftHold => {
				if desired_delta >= 0.0 && desired_delta < tolerance {
					return;
				}
				if delta_delta * delta_time > 0.5 && delta_delta * delta_time < 5.0 && desired_delta.abs() < 10.0 {
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
				if desired_delta >= 0.0 || delta_delta * delta_time > 0.5 {
					self.data.stage = VStage::LiftHold;
					self.update_stage(delta_time, target_speed);
					return;
				}
				// PRESSING BREAK
				if delta_delta * delta_time <= 0.0 {
					self.data.pdl_break -= delta_time * (0.05 * desired_delta).clamp(-2.0, 0.0);
					if self.data.pdl_break > 1.0 {
						self.data.pdl_break = 1.0;
					}
				}
			},
			VStage::AccWait => {
				if desired_delta < 0.0 {
					self.data.stage = VStage::LiftPull;
					self.update_stage(delta_time, target_speed);
					return;
				}
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

	fn calc_signal_target(
		&mut self,
		allocation: &NetworkAllocation,
		lane_speed: f32
	) -> InstructSlow {
		
		// SIGNAL INSTRUCT

		self.signal_instructs.clear();
		self.destroyed_active_signals.clear();
		for signal in self.active_signals.iter() {
			println!("INSTRUCTING SIGNAL");
			let mut c_signal = signal.clone();
			unsafe {
				let wa_signal = Arc::get_mut_unchecked(&mut c_signal);
				match wa_signal.instruct(allocation, &self) {
					InstructResult::KEEP => {},
					InstructResult::DESTROY => {
						self.destroyed_active_signals.push(c_signal);
					},
					InstructResult::SLOW(instruct_slow) => {
						self.signal_instructs.push(instruct_slow);
					}
				}
			}
		}

		// DESTROY SIGNALS

		for signal in self.destroyed_active_signals.iter() {
			let pos = self.active_signals.iter().position(
				|x|
				x.identity().id == signal.identity().id
			).expect("can not destroy signal that does not exist");
			self.active_signals.swap_remove(pos);
		}

		// MIN SPEED

		let mut min_signal_instruct: InstructSlow = InstructSlow {
			target_speed: lane_speed,
			target: VTarget::AvgSpeed
		};
		for signal_instruct in self.signal_instructs.iter() {
			if signal_instruct.target_speed < min_signal_instruct.target_speed {
				min_signal_instruct = *signal_instruct;
			}
		}
		min_signal_instruct
	}

	pub fn distance_from_fw(
		&self,
		allocation: &NetworkAllocation,
		target_offset_distance: f32,
		target_lane_id: u32
	) -> Option<f32> {
		let c_lane = allocation.lane(self.active_identity.lane);
		let ra_lane = c_lane.read().unwrap();
		let mut accumulated_distance: f32 = ra_lane.length - self.data.distance;
		drop(ra_lane);
		drop(c_lane);
		for lane in self.forward_lanes.iter() {
			if lane.id == target_lane_id {
				return Some(accumulated_distance + target_offset_distance);
			}
			accumulated_distance += lane.length;
		}
		None
	}
}

impl VehicleData {
	pub fn seconds_to_stationary(&self, distance: f32) -> f32 {
		distance / self.speed
	}

	pub fn seconds_to_moving(&self, distance: f32, speed: f32) -> f32 {
		distance / (speed - self.speed)
	}
}