use std::{sync::{RwLock, RwLockReadGuard, RwLockWriteGuard, Arc, Mutex}, collections::{HashMap, BTreeMap}, iter::Map, cell::UnsafeCell};

use bevy::prelude::*;

use crate::components::VehicleComponent;
/*
macro_rules! clip {
    ($expression:expr) => {
		$expression.
        UnsafeCell<Clip>
    };
}
*/
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

pub struct Navigation {
	pub recent_nav: u16,
	pub nav: Vec<LaneIdentifier>,
	pub nav_valid_band_lanes: Vec<Vec<u32>>,

	pub target_clip: u32,
	pub target_band: u32,
	pub target_lane: u32,
}

impl Navigation {
	pub fn new(network: &RwLockReadGuard<Network>, active_clip: u32, active_band: u32, active_lane: u32, target_clip: u32, target_band: u32, target_lane: u32) -> Self {
		let mut nav = Navigation {
			recent_nav: 0,
			nav: Vec::new(),
			nav_valid_band_lanes: Vec::new(),
			target_clip,
			target_band,
			target_lane,
		};

		unsafe {
			nav.renavigate(network, active_clip, active_band, active_lane);
		}
		nav
	}

	fn reset_nav(&mut self) {
		self.recent_nav = 0;
		self.nav.clear();
		self.nav_valid_band_lanes.clear();
	}

	unsafe fn update_nav(&mut self, network: &RwLockReadGuard<Network>, band_gf_costs: Vec<(u32, f64, f64)>, band_path_rev: BTreeMap<u32, u32>) {
		
		// reconstruct path
		
		let mut current: u32 = self.target_band;
		loop {
			self.nav.insert(0, LaneIdentifier { band: current, clip: 0 });
			current = match band_path_rev.get(&current) {
				Some(x) => *x,
				None => { break; },
			};
		}

		// valid lanes

		for nav in self.nav.iter_mut() {
			nav.clip = (*network.band(nav.band)).src_clip;
		}

		for (i, nav) in self.nav.iter().enumerate() {
			let band = network.band(nav.band);
			let fw_clip = network.clip((*band).dst_clip);

			if (i + 1) == self.nav.len() {
				self.nav_valid_band_lanes.push(vec![self.target_lane]);
			} else {
				let fw_band = network.band(self.nav[i + 1].band);
				let mut valid_lanes: Vec<u32> = Vec::new();
				println!("mins: {} {} maxes: {} {}", (*fw_band).src_min, (*band).dst_min, (*fw_band).src_max, (*band).dst_max);
				for j in ((*fw_band).src_min..((*fw_band).src_max + 1)).filter(|&x| x >= (*band).dst_min && x <= ((*band).dst_max + 0)) {
					valid_lanes.push((*fw_clip).lanes_fixed[j as usize].0);
				}
				self.nav_valid_band_lanes.push(valid_lanes);
			}
		}
	}

	pub unsafe fn renavigate(&mut self, network: &RwLockReadGuard<Network>, active_clip: u32, active_band: u32, active_lane: u32) -> bool {
		println!("renav: {} {} {}", active_clip, active_band, active_lane);
		self.reset_nav();
		let focus_h: Vec2 = (*network.lane(self.target_lane)).p4;
		let mut current_band = network.band(active_band);
		let mut current_band_gf: u32 = 0;
		let mut band_gf_costs: Vec<(u32, f64, f64)> = Vec::new();
		let mut band_path_rev: BTreeMap<u32, u32> = BTreeMap::new();

		println!("src clip {}", (*current_band).src_clip);

		{
			let bw_clip = network.clip((*current_band).src_clip);
			let band_lane_idx = (*current_band).src_min;
			let band_lane = network.lane((*bw_clip).lanes_fixed[band_lane_idx as usize].1);
			band_gf_costs.push((active_band, 0.0, (*band_lane).p4.distance(focus_h).into()));
		}

		loop {
			// find minimum

			if band_gf_costs.is_empty() {
				return false;
			}

			let min_cost = band_gf_costs.iter().enumerate().min_by(
				|a, b|
				(a.1.2).partial_cmp(&b.1.2).expect("invalid distance")
			).unwrap();

			println!("g: {} f: {}", min_cost.1.1, min_cost.1.2);

			// check ended

			current_band_gf = min_cost.0 as u32;
			current_band = network.band(band_gf_costs[current_band_gf as usize].0);

			// check dead end

			println!("{}", (*current_band).dst_clip);
			let fw_clip =  network.clip((*current_band).dst_clip);
			// if fw_clip.fw_bands.is_empty() {
			// 	band_gf_costs.remove(current_band_gf as usize);
			// 	continue;
			// }

			// branch minimum
			println!("{:?}", *fw_clip);
			for i in (*fw_clip).fw_bands.iter() {
				let mut gf: (u32, f64, f64) = (*i, band_gf_costs[current_band_gf as usize].1, 0.0);
				let band_lane_idx = (*network.band(*i)).src_min;
				let band_lane = network.lane((*fw_clip).lanes_fixed[band_lane_idx as usize].1);
				gf.1 += (*band_lane).length as f64;
				gf.2 = gf.1 + (*band_lane).p4.distance(focus_h) as f64;

				match band_gf_costs.iter().find(
					|x|
					x.0 == gf.0
				) {
					Some(x) => {},
					None => { band_gf_costs.push(gf); },
				};

				// give future ref to current
				
				match band_path_rev.get_mut(&gf.0) {
					Some(x) => { *x = band_gf_costs[current_band_gf as usize].0; },
					None => {
						println!("new: {} old: {}", gf.0, band_gf_costs[current_band_gf as usize].0);
						band_path_rev.insert(gf.0, band_gf_costs[current_band_gf as usize].0);
					},
				};

				if *i == self.target_band {
					self.update_nav(network, band_gf_costs, band_path_rev);
					return true;
				}
			}

			band_gf_costs.remove(current_band_gf as usize);
		}








		// self.nav.push(LaneIdentifier { band: 1, clip: 1 });
		// self.nav.push(LaneIdentifier { band: 2, clip: 2 });
		// self.nav_valid_band_lanes.push(vec![1]);
		// self.nav_valid_band_lanes.push(vec![4]);

		// true

		// println!("renavigating not implemented; despawning vehicle.");
		// false
	}

	pub unsafe fn get_forward_lanes(&self, network: &Network, minimum_length: f32, mut active_lane: u32, mut active_nav: u16) -> Vec<(u32, f32)> {
		let mut result: Vec<(u32, f32)> = Vec::new();
		let mut total_distance: f32 = 0.0;
		let mut lane = network.lane(active_lane);
		loop {
			if (*lane).fw.is_empty() {
				println!("lane.fw.is_empty()");
				break;
			}
			active_nav = active_nav + 1;
			if active_nav >= self.nav.len() as u16 {
				println!("active_nav >= self.nav.len()");
				break;
			}
			let valid_lane = match (*lane).fw.iter().enumerate().find(
				|x| self.nav_valid_band_lanes[active_nav as usize].contains(&x.1.lane)
			) {
				Some(x) => x,
				None => { println!("no lanes fw are valid\n{:?} {:?} {:?}", self.nav, self.nav_valid_band_lanes, (*lane).fw); break; },
			};
			active_lane = valid_lane.1.lane;
			lane = network.lane(active_lane);
			result.push((valid_lane.1.lane, (*lane).length));
			total_distance += (*lane).length;

			if total_distance >= minimum_length {
				println!("total_distance >= minimum_length");
				break;
			}
		}
		result
	}

	/*
	pub fn get_forward_lanes_mut(&self, network: &mut RwLockWriteGuard<Network>, minimum_length: f32, mut active_lane: u32, mut active_nav: u16) -> Vec<(u32, f32)> {
		let mut result: Vec<(u32, f32)> = Vec::new();
		let mut total_distance: f32 = 0.0;
		let mut lane = network.alloc_lanes.get(&active_lane).expect("invalid lane");
		loop {
			if lane.fw.is_empty() {
				break;
			}
			active_nav = active_nav + 1;
			if active_nav >= self.nav.len() as u16 {
				break;
			}
			let valid_lane = match lane.fw.iter().enumerate().find(
				|x| self.nav_valid_band_lanes[active_nav as usize].contains(&x.1.lane)
			) {
				Some(x) => x,
				None => { break; },
			};
			active_lane = valid_lane.1.lane;
			lane = network.alloc_lanes.get(&active_lane).expect("invalid lane");
			result.push((valid_lane.1.lane, lane.length));
			total_distance += lane.length;

			if total_distance >= minimum_length {
				break;
			}
		}
		result
	}
	*/
}

#[derive(Debug)]
pub struct NetworkIdentifier {
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

#[derive(Debug)]
pub struct LaneIdentifier {
	pub band: u32,
	pub clip: u32,
}

pub struct Lane {
	pub identity: LaneIdentifier,
	pub fw: Vec<NetworkIdentifier>,
	pub bw: Vec<NetworkIdentifier>,

	pub p1: Vec2,
	pub p2: Vec2,
	pub p3: Vec2,
	pub p4: Vec2,

	pub points: Vec<(Vec2, f32)>,
	pub length: f32,

	pub vehicles: Vec<(u32, f32)>,
}

impl Lane {
	pub fn get_interp_position(&self, distance: f32) -> Vec2 {
		if distance == 0.0 {
			return self.p1;
		}
		
		let mut m_dis: f32 = 0.0;
		let mut m_iter: usize = 0;
		
		while m_dis < distance {
			m_iter = m_iter + 1;
			m_dis = self.points[m_iter].1;
		}

		let distance_delta = distance - self.points[m_iter - 1].1;
		let point_delta = self.points[m_iter].1 - self.points[m_iter - 1].1;
		let interp_percent = distance_delta / point_delta;

		self.points[m_iter - 1].0.lerp(self.points[m_iter].0, interp_percent)
	}
}

// pub struct BandNode {
// 	// Valid lane idx (0-X) within the previous band to access this future band.
// 	pub min: u8,
// 	pub max: u8,
// }

pub struct Band {
	// pub fw: Vec<(BandNode, Band)>,
	// pub lanes: Vec<u32>,
	pub src_clip: u32,
	pub src_min: u8,
	pub src_max: u8,

	pub dst_clip: u32,
	pub dst_min: u8,
	pub dst_max: u8,

	pub empty: bool,
}

#[derive(Debug)]
pub struct Clip {
	// Fixed size of how long the clip is. Forward then back.
	pub lanes_fixed: Vec<(u32, u32)>,
	pub fw_bands: Vec<u32>,
}

#[derive(Default)]
pub struct Network {
	pub alloc_clips: Arc<Mutex<HashMap<u32, UnsafeCell<Clip>>>>,
	pub alloc_bands: Arc<Mutex<HashMap<u32, UnsafeCell<Band>>>>,
	pub alloc_lanes: Arc<Mutex<HashMap<u32, UnsafeCell<Lane>>>>,
	pub start_clips: Arc<Mutex<Vec<usize>>>,

	pub clip_count: RwLock<u32>,
	pub band_count: RwLock<u32>,
	pub lane_count: RwLock<u32>,
	pub vehicle_count: RwLock<u32>,
}

impl Network {
	pub fn clip(&self, clip_id: u32) -> *mut Clip {
		self.alloc_clips.clone().lock().expect("failed to lock alloc object").get_mut(&clip_id).expect("invalid clip id").get()
	}

	pub fn band(&self, band_id: u32) -> *mut Band {
		self.alloc_bands.clone().lock().expect("failed to lock alloc object").get_mut(&band_id).expect("invalid band id").get()
	}

	pub fn lane(&self, lane_id: u32) -> *mut Lane {
		self.alloc_lanes.clone().lock().expect("failed to lock alloc object").get_mut(&lane_id).expect("invalid lane id").get()
	}
}
