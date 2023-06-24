use std::sync::RwLockWriteGuard;

use nalgebra::{Vector2, Vector3, vector, Point3, Matrix3, Point2, point, Matrix4};

use crate::network::{LANE_MAX_POINTS, lane::point::Point};

use self::{control::Control, control_point::BandControlPoint};

use super::{Network, CLIP_MAX_LENGTH, CP_MIN_DISTANCE, lane::Lane, clip::Clip, BAND_MAX_CONTROLS};

pub mod control_point;
pub mod control;

#[derive(Default, Debug, Clone, Copy)]
pub struct BandIdentity {
	pub band: u32,
	pub clip: u32,
}

/// Slice of lanes between 2 clips that are
/// mathmaticaly connected.
#[derive(Debug, Clone, Copy)]
pub struct Band {
	pub id: u32,
	pub src_clip: u32,
	pub src_min: u8,
	pub src_max: u8,

	/// Each fixed slot in a clip can have multiple
	/// lanes splitting; This selects
	/// which one of those is part of this band.
	pub src_fixed_idx: [u8; CLIP_MAX_LENGTH],
	pub dst_clip: u32,
	pub dst_min: u8,
	pub dst_max: u8,

	/// Each fixed slot in a clip can have multiple
	/// lanes merging; This selects
	/// which one of those is part of this band.
	pub dst_fixed_idx: [u8; CLIP_MAX_LENGTH],
	
	/// See `te::network::band::control::Control` for more information.
	pub controls: [Control; BAND_MAX_CONTROLS],

	/// Whether or not this band contains dummy initialized
	/// data or not.
	pub empty: bool,
}

impl Band {
	pub fn new(
		network: &mut Network,
		src_clip: u32, dst_clip: u32,
	) -> u32 {
		let id = network.fetch_band_id();
		let mut wa_bands = network.bands.write().unwrap();
		wa_bands[id as usize] = Self {
			id,
			src_clip,
			src_min: u8::MAX,
			src_max: u8::MAX,
			src_fixed_idx: [0; CLIP_MAX_LENGTH],
			dst_clip,
			dst_min: u8::MAX,
			dst_max: u8::MAX,
			dst_fixed_idx: [0; CLIP_MAX_LENGTH],
			controls: [Control::default(); BAND_MAX_CONTROLS],
			empty: true
		};
		drop(wa_bands);
		network.clips.write().unwrap()[src_clip as usize].add_fw_band(id);
		id
	}

	pub fn default(
	) -> Self {
		Self {
			id: 0,
			src_clip: 0,
			src_min: u8::MAX,
			src_max: u8::MAX,
			src_fixed_idx: [0; CLIP_MAX_LENGTH],
			dst_clip: 0,
			dst_min: u8::MAX,
			dst_max: u8::MAX,
			dst_fixed_idx: [0; CLIP_MAX_LENGTH],
			controls: [Control::default(); BAND_MAX_CONTROLS],
			empty: true
		}
	}

	pub fn set_control(
		&mut self,
		control_idx: u8,
		c1: BandControlPoint,
		c2: BandControlPoint,
	) {
		let control = &mut self.controls[control_idx as usize];
		control.c1 = c1;
		control.c2 = c2;
	}

	/// Regenerates the band control points and all lane points.
	pub fn regenerate_points(
		&mut self,
		network: &&mut Network,
	) {
		debug_assert!(self.src_min < u8::MAX);
		debug_assert!(self.src_max < u8::MAX);
		debug_assert!(self.dst_min < u8::MAX);
		debug_assert!(self.dst_max < u8::MAX);

		let mut wa_clips = network.clips.write().unwrap();
		let mut wa_lanes = network.lanes.write().unwrap();
		let left_lane_fixed = &wa_clips[self.src_clip as usize].lanes_fixed[self.src_min as usize];
		let left_lane_id = left_lane_fixed.fw[*self.src_fixed_idx.first().unwrap() as usize];
		let left_lane = &wa_lanes[left_lane_id as usize];

		// GENERATE CONTROL POINTS

		let mut control_count: u8 = 0;
		for (i, control) in self.controls.iter().enumerate() {
			if control.src_min == u8::MAX {
				control_count = i as u8;
				break;
			}
		}

		for i in 0..control_count {
			let control = &self.controls[i as usize];
			
			let p1: Point3<f32>;
			let p2: Point3<f32>;
			let p3: Point3<f32>;
			let p4: Point3<f32>;

			// ABSOLUTE CONTROLS

			{
				let wa_src_clip = &mut wa_clips[self.src_clip as usize];
				let mut src_clip_offset: f32 = 0.0;
				for lane_fixed_idx in 0..control.src_min {
					let lane_fixed = &wa_src_clip.lanes_fixed[lane_fixed_idx as usize];
					src_clip_offset += lane_fixed.width;
				}
				// P1 of the band is along the clip. RX is the width offset from the start
				// of the clip to the first lane.
				p1 = wa_src_clip.get_position_bank_rx(
					src_clip_offset,
				).into();
				let m: Matrix4<f32> = Matrix4::new_rotation(
					vector![control.c1.vangle, wa_src_clip.angle, 0.0],
				);
				let m: Matrix4<f32> = m.append_translation(
					&p1.coords,
				);
				// P2 of the band is relative to P1. Transformed using the control
				// point forward and angle (angle of P1 is stored in C1).
				p2 = m.transform_point(
					&point![0.0, 0.0, control.c1.forward],
				);
			} {
				let wa_dst_clip = &mut wa_clips[self.dst_clip as usize];
				let mut dst_clip_offset: f32 = 0.0;
				for lane_fixed_idx in 0..control.dst_min {
					let lane_fixed = &wa_dst_clip.lanes_fixed[lane_fixed_idx as usize];
					dst_clip_offset += lane_fixed.width;
				}
				// P4 of the band is along the clip. RX is the width offset from the start
				// of the clip to the first lane.
				p4 = wa_dst_clip.get_position_bank_rx(
					dst_clip_offset,
				).into();
				let m: Matrix4<f32> = Matrix4::new_rotation(
					vector![control.c2.vangle, wa_dst_clip.angle, 0.0],
				);
				let m: Matrix4<f32> = m.append_translation(
					&p4.coords,
				);
				// P3 of the band is relative to P4. Transformed using the control
				// point forward and angle (angle of P4 is stored in C2).
				p3 = m.transform_point(
					&point![0.0, 0.0, control.c2.forward],
				);
			}
			
			// FAR LEFT POSITIONS

			let mut points: [Point; LANE_MAX_POINTS] = Default::default();
			let mut point_distance: [f32; LANE_MAX_POINTS] = Default::default();
			let mut last_point: Vector3<f32> = Vector3::new(p1.x, p1.y, p1.z);
			let mut accumulated_distance: f32 = 0.0;

			let count: u16 = LANE_MAX_POINTS as u16;
			for i in 0..count {
				let t: f32 = (i as f32) / ((count - 1) as f32);
				let omt: f32 = 1.0 - t;
				let tm1: Point3<f32> = p1 * omt.powf(3.0);
				let tm2: Point3<f32> = p2 * omt.powf(2.0) * t * 3.0;
				let tm3: Point3<f32> = p3 * omt * t.powf(2.0) * 3.0;
				let tm4: Point3<f32> = p4 * t.powf(3.0);
				let p: Vector3<f32> = tm1.coords + tm2.coords + tm3.coords + tm4.coords;
				let mut dis: f32 = 0.0;
				if i > 0 {
					dis = last_point.metric_distance(&p);
				}
				last_point = p;
				accumulated_distance += dis;
				points[i as usize] = Point{ position: Vector3::new(p.x, p.y, p.z), accumulated_distance };
				point_distance[i as usize] = accumulated_distance;
			}

			// PATHING MATRIX
			// For each computed left lane point, set all
			// the lanes' points using matrix.

			let wa_src_clip = &wa_clips[self.src_clip as usize];
			let wa_dst_clip = &wa_clips[self.dst_clip as usize];

			for i in 0..(LANE_MAX_POINTS as u8) {
				let t: f32 = (i as f32) / (LANE_MAX_POINTS as f32);
				let angle = (wa_src_clip.angle * (1.0 - t)) + (wa_dst_clip.angle * t);
				let position: Vector3<f32> = points[i as usize].position;
				let mut width_offset: f32 = 0.0;
				for j in 0..(CLIP_MAX_LENGTH as u8) {
					let lane_src_lnum = control.lane_src_lnum[j as usize];
					if lane_src_lnum == u8::MAX {
						break;
					}
					let lane_src_fixed_idx = control.lane_src_fixed_idx[j as usize];
					let wa_clip_fixed = &wa_src_clip.lanes_fixed[lane_src_lnum as usize];
					let lane_id = wa_clip_fixed.fw[lane_src_fixed_idx as usize];
					let wa_lane = &mut wa_lanes[lane_id as usize];
					let hwidth = wa_clip_fixed.width * 0.5;
					
					let m: Matrix4<f32> = Matrix4::new_rotation(
						vector![0.0, angle, wa_src_clip.bank], // TODO: !!!! 2D bezier for bank !!!!!
					);
					let m: Matrix4<f32> = m.append_translation(
						&position,
					);
					width_offset += hwidth;
					let p = m.transform_point(
						&point![width_offset, 0.0, 0.0],
					);
					width_offset += hwidth;
					wa_lane.points[i as usize].position = p.coords;
					println!("{} - {:?}", lane_id, p.coords);
				}
			}

			// LANE DISTANCES

			for j in 0..(CLIP_MAX_LENGTH as u8) {
				let lane_src_lnum = control.lane_src_lnum[j as usize];
				if lane_src_lnum == u8::MAX {
					break;
				}
				let lane_src_fixed_idx = control.lane_src_fixed_idx[j as usize];
				let wa_clip_fixed = &wa_src_clip.lanes_fixed[lane_src_lnum as usize];
				let lane_id = wa_clip_fixed.fw[lane_src_fixed_idx as usize];
				let wa_lane = &mut wa_lanes[lane_id as usize];
				wa_lane.length = wa_lane.points.last().unwrap().accumulated_distance;
			}
		} // iter controls
		println!("==========================");
	}
}