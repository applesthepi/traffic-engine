use std::sync::RwLockWriteGuard;

use nalgebra::Vector2;

use crate::{network::{navigation::Point}};

use super::{Network, LANE_MAX_CONNECTIONS, LANE_MAX_POINTS, LANE_MAX_VEHICLES, clip::Clip};

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct LaneIdentity {
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct Lane {
	pub identity: LaneIdentity,
	pub fw_lanes: [LaneIdentity; LANE_MAX_CONNECTIONS],
	pub bw_lanes: [LaneIdentity; LANE_MAX_CONNECTIONS],

	pub p1: Vector2<f32>,
	pub p2: Vector2<f32>,
	pub p3: Vector2<f32>,
	pub p4: Vector2<f32>,

	pub points: [Point; LANE_MAX_POINTS],
	pub point_distance: [f32; LANE_MAX_POINTS],
	pub length: f32,

	pub vehicles: [u32; LANE_MAX_VEHICLES],
	// pub signals: Vec<Arc<dyn Signal>>
}

impl Default for Lane {
	fn default() -> Self {
		Self {
			p1: Vector2::default(),
			p2: Vector2::default(),
			p3: Vector2::default(),
			p4: Vector2::default(),
			points: [Point::default(); LANE_MAX_POINTS],
			point_distance: Default::default(),
			identity: LaneIdentity::default(),
			fw_lanes: [LaneIdentity::default(); LANE_MAX_CONNECTIONS],
			bw_lanes: [LaneIdentity::default(); LANE_MAX_CONNECTIONS],
			length: 1.0,
			vehicles: [0; LANE_MAX_VEHICLES],
		}
	}
}

impl Lane {
	pub fn from_streight(
		network: &mut Network,
		wa_clips: &mut RwLockWriteGuard<Vec<Clip>>,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		p1: Vector2<f32>, p2: Vector2<f32>,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32
	) -> u32 {
		let control = (p2 - p1) * 0.1;
		Lane::new(
			network,
			wa_clips,
			wa_lanes,
			p1,
			p1 + control,
			p2 - control,
			p2,
			clip_bw,
			clip_fw,
			lnum_bw,
			lnum_fw,
			band
		)
	}

	pub fn new(
		network: &mut Network,
		wa_clips: &mut RwLockWriteGuard<Vec<Clip>>,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		p1: Vector2<f32>, p2: Vector2<f32>, p3: Vector2<f32>, p4: Vector2<f32>,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32
	) -> u32 {
		let id = network.fetch_lane_id();
		
		// GENERATE POSITIONS

		let mut points: [Point; LANE_MAX_POINTS] = Default::default();
		let mut point_distance: [f32; LANE_MAX_POINTS] = Default::default();
		let mut last_point = p1;
		let mut accumulated_distance: f32 = 0.0;
		let count: u8 = 5;
		for i in 0..count {
			let t: f32 = (i as f32) / ((count - 1) as f32);
			let omt: f32 = 1.0 - t;
			let tm1: Vector2<f32> = p1 * omt.powf(3.0);
			let tm2: Vector2<f32> = p2 * omt.powf(2.0) * t * 3.0;
			let tm3: Vector2<f32> = p3 * omt * t.powf(2.0) * 3.0;
			let tm4: Vector2<f32> = p4 * t.powf(3.0);
			let p: Vector2<f32> = tm1 + tm2 + tm3 + tm4;
			let mut dis: f32 = 0.0;
			if i > 0 {
				dis = last_point.metric_distance(&p);
			}
			last_point = p;
			accumulated_distance += dis;
			points[i as usize] = Point{ position: Vector2::new(p.x, p.y), accumulated_distance };
			point_distance[i as usize] = accumulated_distance;
		}

		// ALLOCATE

		let identity = LaneIdentity {
			lane: id,
			band,
			clip: clip_bw
		};
		wa_lanes[id as usize] = Self {
			p1,
			p2,
			p3,
			p4,
			points: points.try_into().unwrap(),
			point_distance,
			identity,
			length: accumulated_distance,
			..Default::default()
			// signals: Vec::new()
		};

		let wa_clip_fw = &mut wa_clips[clip_fw as usize];
		wa_clip_fw.add_bw_lane(wa_lanes, lnum_fw, identity);
		drop(wa_clip_fw);
		let wa_clip_bw = &mut wa_clips[clip_bw as usize];
		wa_clip_bw.add_fw_lane(wa_lanes, lnum_bw, identity);
		

		// RESIZE BAND

		let mut wa_bands = network.bands.write().unwrap();
		let wa_band = &mut wa_bands[band as usize];
		if wa_band.empty {
			wa_band.empty = false;
			wa_band.src_min = lnum_bw;
			wa_band.src_max = lnum_bw;
			wa_band.dst_min = lnum_fw;
			wa_band.dst_max = lnum_fw;
		} else {
			wa_band.src_min = wa_band.src_min.min(lnum_bw);
			wa_band.src_max = wa_band.src_max.max(lnum_bw);
			wa_band.dst_min = wa_band.src_min.min(lnum_fw);
			wa_band.dst_max = wa_band.src_max.max(lnum_fw);
		}

		wa_clip_bw.add_fw_band(band);
		id
	}

	pub fn add_fw_lane(
		&mut self,
		identity: LaneIdentity,
	) {
		if !self.fw_lanes.contains(&identity) {
			let fw_lane = self.fw_lanes.iter_mut().find(
				|x|
				**x == LaneIdentity::default()
			).expect("no empty fw slot for lane");
			*fw_lane = identity;
		}
	}

	pub fn add_bw_lane(
		&mut self,
		identity: LaneIdentity,
	) {
		if !self.bw_lanes.contains(&identity) {
			let bw_lane = self.bw_lanes.iter_mut().find(
				|x|
				**x == LaneIdentity::default()
			).expect("no empty bw slot for lane");
			*bw_lane = identity;
		}
	}

	pub fn remove_fw_lane(
		&mut self,
		identity: &LaneIdentity,
	) {
		let fw_lane = self.fw_lanes.iter_mut().find(
			|x|
			**x == *identity
		).expect("can not remove fw lane; does not exist");
		*fw_lane = LaneIdentity::default();
	}

	pub fn remove_bw_lane(
		&mut self,
		identity: &LaneIdentity,
	) {
		let bw_lane = self.bw_lanes.iter_mut().find(
			|x|
			**x == *identity
		).expect("can not remove bw lane; does not exist");
		*bw_lane = LaneIdentity::default();
	}

	pub fn add_vehicle(
		&mut self,
		vehicle_idx: u32,
	) {
		let vehicle = self.vehicles.iter_mut().find(
			|x|
			**x == 0
		).expect("no empty vehicle slots for lane");
		*vehicle = vehicle_idx;
	}

	pub fn remove_vehicle(
		&mut self,
		vehicle_idx: u32,
	) {
		let vehicle = self.vehicles.iter_mut().find(
			|x|
			**x == vehicle_idx
		).expect("can not remove vehicle; does not exist");
		*vehicle = 0;
	}

	pub fn generate_c_points(
		&self
	) -> [[f32; 2]; LANE_MAX_POINTS] {
		let mut points: [[f32; 2]; LANE_MAX_POINTS] = [[0.0, 0.0]; LANE_MAX_POINTS];
		for (i, point) in self.points.iter().enumerate() {
			points[i] = point.position.as_slice().try_into().unwrap();
		}
		points
	}
}