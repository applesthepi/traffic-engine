use std::{sync::{RwLock, RwLockReadGuard, RwLockWriteGuard}, collections::HashMap};

use bevy::prelude::*;

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

		nav.renavigate(network, active_clip, active_band, active_lane);
		nav
	}

	pub fn renavigate(&mut self, network: &RwLockReadGuard<Network>, active_clip: u32, active_band: u32, active_lane: u32) -> bool {
		self.recent_nav = 0;
		self.nav.clear();
		self.nav_valid_band_lanes.clear();

		let active_clip = network.alloc_clips.get(&active_clip).expect("invalid clip");









		self.nav.push(LaneIdentifier { band: 1, clip: 1 });
		self.nav.push(LaneIdentifier { band: 2, clip: 2 });
		self.nav_valid_band_lanes.push(vec![1]);
		self.nav_valid_band_lanes.push(vec![4]);

		true

		// println!("renavigating not implemented; despawning vehicle.");
		// false
	}

	pub fn get_forward_lanes(&self, network: &RwLockReadGuard<Network>, minimum_length: f32, mut active_lane: u32, mut active_nav: u16) -> Vec<(u32, f32)> {
		let mut result: Vec<(u32, f32)> = Vec::new();
		let mut total_distance: f32 = 0.0;
		loop {
			let mut lane = network.alloc_lanes.get(&active_lane).expect("invalid lane");
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

	pub fn get_forward_lanes_mut(&self, network: &mut RwLockWriteGuard<Network>, minimum_length: f32, mut active_lane: u32, mut active_nav: u16) -> Vec<(u32, f32)> {
		let mut result: Vec<(u32, f32)> = Vec::new();
		let mut total_distance: f32 = 0.0;
		loop {
			let mut lane = network.alloc_lanes.get(&active_lane).expect("invalid lane");
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
}

pub struct NetworkIdentifier {
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

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

pub struct BandNode {
	// Valid lane idx (0-X) within the previous band to access this future band.
	pub min: u8,
	pub max: u8,
}

pub struct Band {
	pub fw: Vec<BandNode>,
	pub lanes: Vec<u32>,
}

pub struct Clip {
	// Fixed size of how long the clip is. Forward then back.
	pub lanes_fixed: Vec<(u32, u32)>,
}

pub struct Network {
	pub alloc_clips: HashMap<u32, Clip>,
	pub alloc_bands: HashMap<u32, Band>,
	pub alloc_lanes: HashMap<u32, Lane>,
	pub start_clips: Vec<usize>,

	pub clip_count: u32,
	pub band_count: u32,
	pub lane_count: u32,
	pub vehicle_count: u32,
}