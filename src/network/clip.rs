use std::sync::RwLockWriteGuard;

use super::{Network, CLIP_MAX_CONNECTIONS, CLIP_MAX_LENGTH, lane::{Lane, LaneIdentity}};

#[derive(Debug, Default, Clone, Copy)]
pub struct Fixed {
	// pub fw_count: u8,
	// pub bw_count: u8,
	pub fw: [u32; CLIP_MAX_CONNECTIONS],
	pub bw: [u32; CLIP_MAX_CONNECTIONS],
}

#[derive(Debug, Clone, Copy)]
pub struct Clip {
	// Fixed size of how long the clip is. Forward then back.
	pub lanes_fixed: [Fixed; CLIP_MAX_LENGTH],
	pub fw_bands: [u32; CLIP_MAX_LENGTH],
}

impl Default for Clip {
	fn default() -> Self {
		Self {
			lanes_fixed: [Fixed::default(); CLIP_MAX_LENGTH],
			fw_bands: [0; CLIP_MAX_LENGTH],
		}
	}
}

impl Clip {
	pub fn new(
		network: &mut Network
	) -> u32 {
		let id = network.fetch_clip_id();
		let mut wa_clips = network.clips.write().unwrap();
		wa_clips[id as usize] = Self::default();
		id
	}

	pub fn add_fw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: LaneIdentity,
	) {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
		let lane_fixed_fw = lane_fixed.fw.iter_mut().find(
			|x|
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
	}

	pub fn add_bw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: LaneIdentity,
	) {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
		let lane_fixed_bw = lane_fixed.bw.iter_mut().find(
			|x|
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
	}

	pub fn remove_fw_lane(
		&mut self,
		wa_lanes: &mut RwLockWriteGuard<Vec<Lane>>,
		lane_fixed_idx: u8,
		identity: &LaneIdentity,
	) {
		let lane_fixed = &mut self.lanes_fixed[lane_fixed_idx as usize];
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