use std::{sync::RwLockWriteGuard, ops::Mul};

use nalgebra::{Vector2, Vector3, Matrix4, Matrix3, Point2};

use self::point::Point;

use super::{Network, LANE_MAX_CONNECTIONS, LANE_MAX_POINTS, LANE_MAX_VEHICLES, clip::Clip, CLIP_MAX_LENGTH};

pub mod point;

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
	pub band_control_idx: u8,

	pub points: [Point; LANE_MAX_POINTS],
	pub point_distance: [f32; LANE_MAX_POINTS],
	pub length: f32,

	pub vehicles: [u32; LANE_MAX_VEHICLES],
	// pub signals: Vec<Arc<dyn Signal>>
}

impl Default for Lane {
	fn default() -> Self {
		Self {
			points: [Point::default(); LANE_MAX_POINTS],
			point_distance: Default::default(),
			identity: LaneIdentity::default(),
			fw_lanes: [LaneIdentity::default(); LANE_MAX_CONNECTIONS],
			bw_lanes: [LaneIdentity::default(); LANE_MAX_CONNECTIONS],
			band_control_idx: u8::MAX,
			length: 1.0,
			vehicles: [0; LANE_MAX_VEHICLES],
		}
	}
}

impl Lane {
	pub fn new_nop(
		network: &mut Network,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32,
		band_control_idx: u8,
	) -> u32 {
		Lane::new(
			network,
			clip_bw,
			clip_fw,
			lnum_bw,
			lnum_fw,
			band,
			band_control_idx,
			false,
		)
	}

	pub fn new_op(
		network: &mut Network,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32,
		band_control_idx: u8,
	) -> u32 {
		Lane::new(
			network,
			clip_bw,
			clip_fw,
			lnum_bw,
			lnum_fw,
			band,
			band_control_idx,
			true,
		)
	}

	fn new(
		network: &mut Network,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32,
		band_control_idx: u8,
		regenerate_band: bool,
	) -> u32 {
		let id = network.fetch_lane_id();

		let clips = network.clips();
		let mut wa_clips = clips.write().unwrap();
		let lanes = network.lanes();
		let mut wa_lanes = lanes.write().unwrap();

		// ALLOCATE

		let identity = LaneIdentity {
			lane: id,
			band,
			clip: clip_bw
		};
		wa_lanes[id as usize] = Self {
			points: [Point::default(); LANE_MAX_POINTS],
			point_distance: [0.0; LANE_MAX_POINTS],
			band_control_idx,
			identity,
			length: 0.0,
			..Default::default()
		};

		let wa_clip_fw = &mut wa_clips[clip_fw as usize];
		let dst_clip_fixed_idx = wa_clip_fw.add_bw_lane(&mut wa_lanes, lnum_fw, identity);
		drop(wa_clip_fw);
		let wa_clip_bw = &mut wa_clips[clip_bw as usize];
		let src_clip_fixed_idx = wa_clip_bw.add_fw_lane(&mut wa_lanes, lnum_bw, identity);
		
		// RESIZE BAND

		let mut wa_bands = network.bands.write().unwrap();
		let wa_band = &mut wa_bands[band as usize];
		if wa_band.empty {
			wa_band.empty = false;
			// SRC
			wa_band.src_min = lnum_bw;
			wa_band.src_max = lnum_bw;
			wa_band.src_fixed_idx[lnum_bw as usize] = src_clip_fixed_idx;
			// DST
			wa_band.dst_min = lnum_fw;
			wa_band.dst_max = lnum_fw;
			wa_band.dst_fixed_idx[lnum_fw as usize] = dst_clip_fixed_idx;
		} else {
			// SRC
			wa_band.src_min = wa_band.src_min.min(lnum_bw);
			wa_band.src_max = wa_band.src_max.max(lnum_bw);
			wa_band.src_fixed_idx[lnum_bw as usize] = src_clip_fixed_idx;
			// DST
			wa_band.dst_min = wa_band.src_min.min(lnum_fw);
			wa_band.dst_max = wa_band.src_max.max(lnum_fw);
			wa_band.dst_fixed_idx[lnum_fw as usize] = dst_clip_fixed_idx;
		}
		let control = &mut wa_band.controls[band_control_idx as usize];
		if control.empty {
			control.empty = false;
			// CON
			control.lane_src_lnum[0] = lnum_bw;
			control.lane_src_fixed_idx[0] = src_clip_fixed_idx;
			// CON SRC
			control.src_min = lnum_bw;
			control.src_max = lnum_bw;
			// CON DST
			control.dst_min = lnum_fw;
			control.dst_max = lnum_fw;
		} else {
			// CON
			for i in 0..(CLIP_MAX_LENGTH as u8) {
				if control.lane_src_lnum[i as usize] == u8::MAX {
					control.lane_src_lnum[i as usize] = lnum_bw;
					control.lane_src_fixed_idx[i as usize] = src_clip_fixed_idx;
					break;
				}
			}
			// CON SRC
			control.src_min = control.src_min.min(lnum_bw);
			control.src_max = control.src_max.max(lnum_bw);
			// CON DST
			control.dst_min = control.dst_min.min(lnum_fw);
			control.dst_max = control.dst_max.max(lnum_fw);
		}
		drop(control);

		wa_clip_bw.add_fw_band(band);
		drop(wa_clips);
		// drop(wa_bands);
		drop(wa_lanes);
		if regenerate_band {
			wa_band.regenerate_points(
				&network,
			);
		}
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