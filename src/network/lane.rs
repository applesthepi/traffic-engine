use std::{sync::RwLockWriteGuard, ops::Mul};

use nalgebra::Vector2;
use nalgebra_glm::{identity, translate2d, vec2, translate, rotate, rotate2d, vec3, vec4, Vec3, Mat4, Vec2};

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

	pub p1: Vec2,
	pub p2: Vec2,
	pub p3: Vec2,
	pub p4: Vec2,

	pub points: [Point; LANE_MAX_POINTS],
	pub point_distance: [f32; LANE_MAX_POINTS],
	pub length: f32,

	pub vehicles: [u32; LANE_MAX_VEHICLES],
	// pub signals: Vec<Arc<dyn Signal>>
}

impl Default for Lane {
	fn default() -> Self {
		Self {
			p1: Vec2::default(),
			p2: Vec2::default(),
			p3: Vec2::default(),
			p4: Vec2::default(),
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
	// pub fn from_streight(
	// 	network: &mut Network,
	// 	wa_clips: &mut RwLockWriteGuard<Vec<Clip>>,
	// 	wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
	// 	p1: Vector2<f32>, p2: Vector2<f32>,
	// 	clip_bw: u32, clip_fw: u32,
	// 	lnum_bw: u8, lnum_fw: u8,
	// 	band: u32
	// ) -> u32 {
	// 	let control = (p2 - p1) * 0.1;
	// 	Lane::new(
	// 		network,
	// 		wa_clips,
	// 		wa_lanes,
	// 		p1,
	// 		p1 + control,
	// 		p2 - control,
	// 		p2,
	// 		clip_bw,
	// 		clip_fw,
	// 		lnum_bw,
	// 		lnum_fw,
	// 		band
	// 	)
	// }

	/// c1 & c2: Relative control points to the src & dst lane
	/// points. `(FRAC_PI_4, 5.0)` will be 5.0 meters
	/// FORWARD or BACKWARD from the src lane points reglardless
	/// of clip direction, and 45 degress clockwise.
	pub fn new(
		network: &mut Network,
		c1: (f32, f32), c2: (f32, f32),
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32
	) -> u32 {
		let id = network.fetch_lane_id();

		let clips = network.clips().clone();
		let mut wa_clips = clips.write().unwrap();
		let lanes = network.lanes().clone();
		let mut wa_lanes = lanes.write().unwrap();
		
		// GENERATE CONTROL POINTS

		let mut p1: Vec3 = vec3(0.0, 0.0, 0.0);
		let mut p2: Vec3 = vec3(0.0, 0.0, 0.0);
		let mut p3: Vec3 = vec3(0.0, 0.0, 0.0);
		let mut p4: Vec3 = vec3(0.0, 0.0, 0.0);

		{
			let wa_clip_bw = &mut wa_clips[clip_bw as usize];
			let mut clip_bw_offset: f32 = 0.0;
			for lane_fixed_idx in 0..lnum_bw {
				let lane_fixed = &wa_clip_bw.lanes_fixed[lane_fixed_idx as usize];
				clip_bw_offset += lane_fixed.width;
			}
			let matrix_bwc: Mat4 = rotate(
				&identity(),
				0.0,
				&vec3(0.0, 1.0, 0.0),
			);
			let matrix_bwc: Mat4 = translate(
				&matrix_bwc,
				&wa_clip_bw.position,
			);
			p1 = matrix_bwc.transform_vector(
				&vec3(clip_bw_offset, 0.0, 0.0),
			);
			let matrix_c1: Mat4 = rotate(
				&identity(),
				c1.0,
				&vec3(0.0, 1.0, 0.0),
			);
			let matrix_c1: Mat4 = translate(
				&matrix_c1,
				&p1,
			);
			p2 = matrix_c1.transform_vector(
				&vec3(0.0, 0.0, c1.1),
			);
		} {
			let wa_clip_fw = &mut wa_clips[clip_fw as usize];
			let mut clip_fw_offset: f32 = 0.0;
			for lane_fixed_idx in 0..lnum_fw {
				let lane_fixed = &wa_clip_fw.lanes_fixed[lane_fixed_idx as usize];
				clip_fw_offset += lane_fixed.width;
			}
			let matrix_fwc: Mat4 = rotate(
				&identity(),
				0.0,
				&vec3(0.0, 1.0, 0.0),
			);
			let matrix_fwc: Mat4 = translate(
				&matrix_fwc,
				&wa_clip_fw.position,
			);
			p4 = matrix_fwc.transform_vector(
				&vec3(clip_fw_offset, 0.0, 0.0),
			);
			let matrix_c2: Mat4 = rotate(
				&identity(),
				c2.0,
				&vec3(0.0, 1.0, 0.0),
			);
			let matrix_c2: Mat4 = translate(
				&matrix_c2,
				&p4,
			);
			p3 = matrix_c2.transform_vector(
				&vec3(0.0, 0.0, c2.1),
			);
		}

		let p1_2d: Vec2 = p1.xz();
		let p2_2d: Vec2 = p2.xz();
		let p3_2d: Vec2 = p3.xz();
		let p4_2d: Vec2 = p4.xz();

		// GENERATE POSITIONS

		let mut points: [Point; LANE_MAX_POINTS] = Default::default();
		let mut point_distance: [f32; LANE_MAX_POINTS] = Default::default();
		let mut last_point: Vec2 = p1_2d;
		let mut accumulated_distance: f32 = 0.0;
		let count: u8 = 5;
		for i in 0..count {
			let t: f32 = (i as f32) / ((count - 1) as f32);
			let omt: f32 = 1.0 - t;
			let tm1: Vec2 = p1_2d * omt.powf(3.0);
			let tm2: Vec2 = p2_2d * omt.powf(2.0) * t * 3.0;
			let tm3: Vec2 = p3_2d * omt * t.powf(2.0) * 3.0;
			let tm4: Vec2 = p4_2d * t.powf(3.0);
			let p: Vec2 = tm1 + tm2 + tm3 + tm4;
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
			p1: p1_2d,
			p2: p2_2d,
			p3: p3_2d,
			p4: p4_2d,
			points: points.try_into().unwrap(),
			point_distance,
			identity,
			length: accumulated_distance,
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
			wa_band.src_min = lnum_bw;
			wa_band.src_max = lnum_bw;
			wa_band.src_fixed_idx[lnum_bw as usize] = src_clip_fixed_idx;
			wa_band.dst_min = lnum_fw;
			wa_band.dst_max = lnum_fw;
			wa_band.dst_fixed_idx[lnum_fw as usize] = dst_clip_fixed_idx;
		} else {
			wa_band.src_min = wa_band.src_min.min(lnum_bw);
			wa_band.src_max = wa_band.src_max.max(lnum_bw);
			wa_band.src_fixed_idx[lnum_bw as usize] = src_clip_fixed_idx;
			wa_band.dst_min = wa_band.src_min.min(lnum_fw);
			wa_band.dst_max = wa_band.src_max.max(lnum_fw);
			wa_band.dst_fixed_idx[lnum_fw as usize] = dst_clip_fixed_idx;
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