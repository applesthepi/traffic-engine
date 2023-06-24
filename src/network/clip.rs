use std::sync::RwLockWriteGuard;

use nalgebra::{Vector2, Vector3, vector, Point3, Matrix4, point, Matrix3};

use super::{Network, CLIP_MAX_CONNECTIONS, CLIP_MAX_LENGTH, lane::{Lane, LaneIdentity}};

#[derive(Debug, Clone, Copy)]
/// Each clip slot can have multiple lanes merging or diverging at this point.
pub struct Fixed {
	pub empty: bool,
	/// Width of this slot in the clip; Slots can vary in size.
	pub width: f32,
	/// Forward branching lanes from this slot in the clip.
	pub fw: [u32; CLIP_MAX_CONNECTIONS],
	/// Backward merging lanes from this slot in the clip.
	pub bw: [u32; CLIP_MAX_CONNECTIONS],
}

impl Default for Fixed {
	fn default() -> Self {
		Self {
			empty: true,
			width: 3.8,
			fw: [0; CLIP_MAX_CONNECTIONS],
			bw: [0; CLIP_MAX_CONNECTIONS],
		}
	}
}

#[derive(Debug, Clone, Copy)]
pub struct Clip {
	/// Fixed size of how long the clip is. Forward then back.
	pub lanes_fixed: [Fixed; CLIP_MAX_LENGTH],
	pub fw_bands: [u32; CLIP_MAX_LENGTH],
	/// 3D position of left most point of clip. The
	/// direction is pointing away from this point.
	pub position: Vector3<f32>,
	pub angle: f32,
	/// Bank of clip will be used inside all
	/// lane's beziar curves to compute vertices.
	pub bank: f32,
}

impl Default for Clip {
	fn default() -> Self {
		Self {
			lanes_fixed: [Fixed::default(); CLIP_MAX_LENGTH],
			fw_bands: [0; CLIP_MAX_LENGTH],
			position: vector![0.0, 0.0, 0.0],
			angle: 0.0,
			bank: 0.0,
		}
	}
}

impl Clip {
	pub fn new(
		network: &mut Network,
		position: [f32; 3],
		angle: f32,
		bank: f32,
	) -> u32 {
		let id = network.fetch_clip_id();
		let clip = Self {
			position: Vector3::from_row_slice(&position),
			angle,
			bank,
			..Default::default()
		};
		let mut wa_clips = network.clips.write().unwrap();
		wa_clips[id as usize] = clip;
		id
	}

	/// Get the 3D position along the clip using
	/// it's bank and how far along the point is
	/// relative x.
	pub fn get_position_bank_rx(
		&self,
		rx: f32,
	) -> Vector3<f32> {
		debug_assert!(rx >= 0.0);
		let mut clip_width: f32 = 0.0;
		for lane_fixed in self.lanes_fixed.iter() {
			clip_width += lane_fixed.width;
		}
		debug_assert!(rx <= clip_width);
		
		// RHS
		
		let m: Matrix4<f32> = Matrix4::new_rotation(
			vector![0.0, self.angle, self.bank],
		);
		let m: Matrix4<f32> = m.append_translation(
			&self.position,
		);
		let rhs: Vector3<f32> = m.transform_point(
			&point![clip_width, 0.0, 0.0],
		).coords;

		// LERP

		let t = rx / clip_width;
		self.position.lerp(
			&rhs,
			t,
		)
	}

	pub fn add_fw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: LaneIdentity,
	) -> u8 {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
		lane_fixed.empty = false;
		let (
			lane_fixed_fw_idx,
			lane_fixed_fw,
		) = lane_fixed.fw.iter_mut().enumerate().find(
			|(_, x)|
			**x == 0
		).expect(format!("no empty fw slot for lane_fixed index {}", lane_fixed_idx).as_str());
		*lane_fixed_fw = identity.lane;

		for lane_fixed_bw in lane_fixed.bw.iter().filter(
			|x|
			**x > 0
		) {
			let bw_lane = &mut wa_lanes[*lane_fixed_bw as usize];
			bw_lane.add_fw_lane(identity);
			let bw_lane_identity = bw_lane.identity;
			drop(bw_lane);
			// TODO: reverse in removal functions
			wa_lanes[identity.lane as usize].add_bw_lane(bw_lane_identity);
		}
		lane_fixed_fw_idx as u8
	}

	pub fn add_bw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: LaneIdentity,
	) -> u8 {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
		lane_fixed.empty = false;
		let (
			lane_fixed_bw_idx,
			lane_fixed_bw,
		) = lane_fixed.bw.iter_mut().enumerate().find(
			|(_, x)|
			**x == 0
		).expect(format!("no empty bw slot for lane_fixed index {}", lane_fixed_idx).as_str());
		*lane_fixed_bw = identity.lane;

		for lane_fixed_fw in lane_fixed.fw.iter().filter(
			|x|
			**x > 0
		) {
			let fw_lane = &mut wa_lanes[*lane_fixed_fw as usize];
			fw_lane.add_bw_lane(identity);
			let fw_lane_identity = fw_lane.identity;
			drop(fw_lane);
			// TODO: reverse in removal functions
			wa_lanes[identity.lane as usize].add_fw_lane(fw_lane_identity);
		}
		lane_fixed_bw_idx as u8
	}

	pub fn remove_fw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: &LaneIdentity,
	) {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
		lane_fixed.empty = true;
		let lane_fixed_fw = lane_fixed.fw.iter_mut().find(
			|x|
			**x == identity.lane
		).expect("can not remove fw lane; does not exist");
		*lane_fixed_fw = 0;

		for lane_fixed_bw in lane_fixed.bw.iter().filter(
			|x|
			**x > 0
		) {
			wa_lanes[*lane_fixed_bw as usize].remove_fw_lane(&identity);
		}
	}

	pub fn remove_bw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: &LaneIdentity,
	) {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
		lane_fixed.empty = true;
		let lane_fixed_bw = lane_fixed.bw.iter_mut().find(
			|x|
			**x == identity.lane
		).expect("can not remove bw lane; does not exist");
		*lane_fixed_bw = 0;

		for lane_fixed_fw in lane_fixed.fw.iter().filter(
			|x|
			**x > 0
		) {
			wa_lanes[*lane_fixed_fw as usize].remove_bw_lane(&identity);
		}
	}

	pub fn add_fw_band(
		&mut self,
		band: u32,
	) {
		let fw_band = self.fw_bands.iter_mut().find(
			|x|
			**x == 0
		).expect("no empty fw band slot");
		*fw_band = band;
	}

	pub fn remove_fw_band(
		&mut self,
		band: u32,
	) {
		let fw_band = self.fw_bands.iter_mut().find(
			|x|
			**x == band
		).expect("can not remove fw band; does not exist");
		*fw_band = 0;
	}
}