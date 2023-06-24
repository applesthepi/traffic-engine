use std::sync::{Arc, RwLock, atomic::{AtomicU32, Ordering}};

use self::{lane::Lane, clip::Clip, band::Band};

pub mod clip;
pub mod band;
pub mod lane;
// pub mod vehicle;
// pub mod navigation;
// pub mod signal;

pub const BAND_MAX_CONTROLS: usize = 16;

pub const CLIP_MAX_LENGTH: usize = 64;
pub const CLIP_MAX_CONNECTIONS: usize = 5;

pub const LANE_MAX_CONNECTIONS: usize = 5;
pub const LANE_MAX_POINTS: usize = 10;
pub const LANE_MAX_VEHICLES: usize = 100;

pub const THREADS_PER_BLOCK: usize = 64;
pub const BLOCKS_PER_PAGE: usize = 512;
pub const VEHICLES_PER_PAGE: usize = THREADS_PER_BLOCK * BLOCKS_PER_PAGE;

pub const CP_MIN_DISTANCE: f32 = 0.1;

pub struct UnusedIds {
	pub unused_clips: Vec<u32>,
	pub unused_bands: Vec<u32>,
	pub unused_lanes: Vec<u32>,
	pub unused_vehicles: Vec<u32>,
}

pub struct Network {
	pub clips: Arc<RwLock<Vec<Clip>>>,
	pub bands: Arc<RwLock<Vec<Band>>>,
	pub lanes: Arc<RwLock<Vec<Lane>>>,
	// pub vehicles: Arc<RwLock<Vec<Vehicle>>>,

	pub unused_ids: Arc<RwLock<UnusedIds>>,

	clip_id_counter: AtomicU32,
	band_id_counter: AtomicU32,
	lane_id_counter: AtomicU32,
	vehicle_id_counter: AtomicU32,
}

impl Default for UnusedIds {
	fn default() -> Self {
		Self {
			unused_clips: Vec::with_capacity(1024),
			unused_bands: Vec::with_capacity(1024),
			unused_lanes: Vec::with_capacity(1024),
			unused_vehicles: Vec::with_capacity(1024 * 3)
		}
	}
}

impl Network {
	pub fn new() -> Self {
		let mut clips = Vec::with_capacity(1024);
		clips.resize_with(1024, Clip::default);
		let mut bands = Vec::with_capacity(1024);
		bands.resize_with(1024, Band::default);
		let mut lanes = Vec::with_capacity(1024);
		lanes.resize_with(1024, Lane::default);
		// let mut vehicles = Vec::with_capacity(1024);
		// vehicles.resize_with(1024 * 3, Vehicle::default);
		Network {
			clips: Arc::new(RwLock::new(clips)),
			bands: Arc::new(RwLock::new(bands)),
			lanes: Arc::new(RwLock::new(lanes)),
			// vehicles: Arc::new(RwLock::new(vehicles)),
			unused_ids: Arc::new(RwLock::new(UnusedIds::default())),
			clip_id_counter: AtomicU32::new(1),
			band_id_counter: AtomicU32::new(1),
			lane_id_counter: AtomicU32::new(1),
			vehicle_id_counter: AtomicU32::new(1),
		}
	}

	pub fn fetch_clip_id(&mut self) -> u32 {
		let mut wa_unused_ids = self.unused_ids.write().unwrap();
		let id = match wa_unused_ids.unused_clips.pop() {
			Some(unused_id) => {
				unused_id
			},
			None => {
				self.clip_id_counter.fetch_add(1, Ordering::SeqCst)
			}
		};
		let mut wa_clips = self.clips.write().unwrap();
		let clips_length = wa_clips.len();
		if id >= clips_length as u32 {
			wa_clips.resize_with(clips_length * 2, Clip::default);
		}
		id
	}

	pub fn fetch_band_id(&mut self) -> u32 {
		let mut wa_unused_ids = self.unused_ids.write().unwrap();
		let id = match wa_unused_ids.unused_bands.pop() {
			Some(unused_id) => {
				unused_id
			},
			None => {
				self.band_id_counter.fetch_add(1, Ordering::SeqCst)
			}
		};
		let mut wa_bands = self.bands.write().unwrap();
		let bands_length = wa_bands.len();
		if id >= bands_length as u32 {
			wa_bands.resize_with(bands_length * 2, Band::default);
		}
		id
	}

	pub fn fetch_lane_id(&mut self) -> u32 {
		let mut wa_unused_ids = self.unused_ids.write().unwrap();
		let id = match wa_unused_ids.unused_lanes.pop() {
			Some(unused_id) => {
				unused_id
			},
			None => {
				self.lane_id_counter.fetch_add(1, Ordering::SeqCst)
			}
		};
		let mut wa_lanes = self.lanes.write().unwrap();
		let lanes_length = wa_lanes.len();
		if id >= lanes_length as u32 {
			wa_lanes.resize_with(lanes_length * 2, Lane::default);
		}
		id
	}

	// pub fn fetch_vehicle_id(&mut self) -> u32 {
	// 	let mut wa_unused_ids = self.unused_ids.write().unwrap();
	// 	let id = match wa_unused_ids.unused_vehicles.pop() {
	// 		Some(unused_id) => {
	// 			unused_id
	// 		},
	// 		None => {
	// 			let idx = self.vehicle_id_counter.fetch_add(1, Ordering::SeqCst);
	// 			if idx % (VEHICLES_PER_PAGE as u32) == 0 {
	// 				self.vehicle_id_counter.fetch_add(1, Ordering::SeqCst)
	// 			} else {
	// 				idx
	// 			}
	// 		}
	// 	};
	// 	let mut wa_vehicles = self.vehicles.write().unwrap();
	// 	let vehicles_length = wa_vehicles.len();
	// 	if id >= vehicles_length as u32 {
	// 		wa_vehicles.resize_with(vehicles_length * 2, Vehicle::default);
	// 	}
	// 	id
	// }

	pub fn clips(&self) -> Arc<RwLock<Vec<Clip>>> {
		self.clips.clone()
	}

	pub fn bands(&self) -> Arc<RwLock<Vec<Band>>> {
		self.bands.clone()
	}

	pub fn lanes(&self) -> Arc<RwLock<Vec<Lane>>> {
		self.lanes.clone()
	}

	// pub fn vehicles(&self) -> Arc<RwLock<Vec<Vehicle>>> {
	// 	self.vehicles.clone()
	// }
}