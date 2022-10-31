// Contains all basic network data structures.

use std::sync::atomic::AtomicU32;
use std::sync::Arc;
use std::collections::HashMap;

use parking_lot::RwLock;

#[macro_use]
pub mod clip;
#[macro_use]
pub mod band;
#[macro_use]
pub mod lane;
#[macro_use]
pub mod vehicle;
#[macro_use]
pub mod navigation;

use crate::network::clip::*;
use crate::network::band::*;
use crate::network::lane::*;
use crate::network::vehicle::*;
// use crate::network::navigation::*;

#[derive(Default)]
pub struct Network {
	pub allocation: NetworkAllocation,
	pub clip_count: AtomicU32,
	pub band_count: AtomicU32,
	pub lane_count: AtomicU32,
	pub vehicle_count: AtomicU32,
}

#[derive(Default, Clone)]
pub struct NetworkAllocation {
	pub clips: Arc<RwLock<HashMap<u32, Arc<RwLock<Clip>>>>>,
	pub bands: Arc<RwLock<HashMap<u32, Arc<RwLock<Band>>>>>,
	pub lanes: Arc<RwLock<HashMap<u32, Arc<RwLock<Lane>>>>>,
	pub vehicle_batches: Arc<RwLock<HashMap<u32, Arc<RwLock<VehicleBatch>>>>>,
}

impl NetworkAllocation {
	pub fn clip(&self, clip_id: u32) -> Arc<RwLock<Clip>> {
		let allocation_clips = self.clips.read();
		let clip_c = allocation_clips.get(&clip_id).expect("invalid clip id").clone();
		clip_c
	}

	pub fn band(&self, band_id: u32) -> Arc<RwLock<Band>> {
		let allocation_bands = self.bands.read();
		let band_c = allocation_bands.get(&band_id).expect("invalid band id").clone();
		band_c
	}

	pub fn lane(&self, lane_id: u32) -> Arc<RwLock<Lane>> {
		let allocation_lanes = self.lanes.read();
		let lane_c = allocation_lanes.get(&lane_id).expect("invalid lane id").clone();
		lane_c
	}
}