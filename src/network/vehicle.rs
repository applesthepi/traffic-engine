use std::{collections::VecDeque, sync::{atomic::Ordering, Arc}};

use tokio::sync::RwLock;

use super::{navigation::{Navigation, ForwardLane}, Network, lane::LaneIdentity, NetworkAllocation, NetworkVertex, BATCH_COUNT};

pub enum TickStatus {
	PERSIST,
	DESTROY
}

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
}

#[derive(Default, Debug)]
pub struct VehicleInt {
	pub id: u32,
	pub distance: f32,
}

pub struct Vehicle {
	pub id: u32,
	pub bid: u32,
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

#[derive(Default)]
pub struct VehicleBufferInfo {
	pub vertices: Vec<NetworkVertex>,
	pub indices: Vec<u32>
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
	pub async fn new(
		network: &Arc<Network>,
		src_identity: LaneIdentity,
		dst_identity: LaneIdentity
	) -> u32 {

		let mut allocation = network.allocation.clone();

		// ID

		let id = network.vehicle_count.fetch_add(
			1,
			Ordering::Relaxed
		) + 1;
		// println!("aquired vehicle id {}", id);

		// ALLOCATION

		let mut vehicle = Self {
			id,
			bid: 0,
			driver_personality: DriverPersonality {
				wait_exp_0: 1.0,
				wait_exp_1: 3.0,
				wait_releases: 0x1 | 0x2 | 0x4 | 0x8 | 0x10 | 0x20,
			},
			speed: 100.0,
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
		vehicle.navigation.renavigate(&allocation, vehicle.active_identity).await;
		// println!("{:?}", vehicle.navigation);
		let c_lane = allocation.lane(src_identity.lane).await;
		let mut wa_lane = c_lane.write().await;
		wa_lane.vehicles.push(VehicleInt {
			id,
			distance: 0.0,
		});
		drop(wa_lane);
		drop(c_lane);

		// SPAWN VEHICLE

		// VB  -> Vehicle Batch
		// SVB -> Staged Vehicle Batch
		// UVB -> Unused Vehicle Batch
		let svb_c = allocation.staged_vehicle_batch.clone();
		let mut wa_svb = svb_c.write().await;
		vehicle.bid = wa_svb.id;
		wa_svb.vehicles.push(vehicle);
		if /*wa_svb.vehicles.len() >= BATCH_COUNT*/ true {
			let bid = wa_svb.id;
			drop(wa_svb);
			let mut wa_uvb = allocation.unused_vehicle_batchs.write().await;
			if let Some(vb) = wa_uvb.pop() {
				allocation.staged_vehicle_batch = vb.clone();
			} else {
				let new_id = network.vehicle_batch_counter.fetch_add(1, Ordering::SeqCst) + 1;
				allocation.staged_vehicle_batch = Arc::new(RwLock::new(VehicleBatch::new(new_id)));
			}
			drop(wa_uvb);
			let mut wa_vb = allocation.vehicle_batches.write().await;
			wa_vb.insert(bid, allocation.staged_vehicle_batch.clone());
		}

		id
	}

	pub async fn tick_temp(
		&mut self,
		allocation: &NetworkAllocation,
		delta_time: f32
	) -> TickStatus {
		// println!("{:?}", self.navigation);
		self.distance += self.speed * delta_time;
		self.speed += self.acceleration * delta_time;

		// TEMP: MOVE TO BEST LANE

		self.pull_forward_lanes(allocation).await;

		if /*self.forward_lanes.len() <= 1*/ false {  
			let c_clip = allocation.clip(self.active_identity.clip).await;
			let ra_clip = c_clip.read().await;
			let mut best_id: u32 = 0;
			let mut best_diff: i8 = i8::MAX;


			let fixed_current = ra_clip.lanes_fixed.iter().enumerate().find(
				|x|
				{
					let f = x.1;
					for i in 0..f.fw_count {
						if f.fw[i as usize] == self.active_identity.lane {
							return true;
						}
					}
					false
				}
			).expect("target lane does not exist");



			// let fixed_current = ra_clip.lanes_fixed.iter().enumerate().find(
			// 	|x|
			// 	x.1.fw == self.active_identity.lane
			// ).expect("vehicle active lane does not belong to active clip");
			for lane_id in self.navigation.nav_valid_band_lanes[self.navigation.active_nav as usize].iter() {
				if let Some(fixed_valid) = ra_clip.lanes_fixed.iter().enumerate().find(
					|x|
					{
						let f = x.1;
						for i in 0..f.fw_count {
							if f.fw[i as usize] == *lane_id {
								return true;
							}
						}
						false
					}
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

		let c_lane = allocation.lane(lane).await;
		let ra_lane = c_lane.read().await;
		if self.distance < ra_lane.length {
			return TickStatus::PERSIST;
		}
		drop(ra_lane);
		drop(c_lane);

		if self.forward_lanes.is_empty() {
			let c_lane = allocation.lane(lane).await;
			let mut wa_lane = c_lane.write().await;
			if let Some(idx) = wa_lane.vehicles.iter().position(
				|x|
				x.id == self.id
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

		loop {
			let c_lane = allocation.lane(lane).await;
			let mut wa_lane = c_lane.write().await;

			if let Some(idx) = wa_lane.vehicles.iter().position(
				|x|
				x.id == self.id
			) {
				wa_lane.vehicles.swap_remove(idx);
			} else {
				println!("failed to remove vehicle from lane; id does not exist in specified lane");
			}
			self.distance -= wa_lane.length;
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
			lane = self.active_identity.lane;

			drop(wa_lane);
			drop(c_lane);

			let c_lane = allocation.lane(lane).await;
			let mut wa_lane = c_lane.write().await;

			wa_lane.vehicles.push(VehicleInt {
				id: self.id,
				distance: self.distance,
			});
			self.active_identity.band = wa_lane.identity.band;
			self.active_identity.clip = wa_lane.identity.clip;
			self.forward_length -= fw_lane.length;
			// println!("inc active nav to {}", self.navigation.active_nav);

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
			self.navigation.active_nav += 1;
			if wa_lane.length > self.distance {
				break;
			}
		}

		TickStatus::PERSIST
		
		// let v_pos = lane.get_interp_position(self.distance);

		// transform.rotation = Quat::from_rotation_z(0.05);
		// transform.translation = Vec3::new(v_pos.x, v_pos.y, 1.0);
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
}