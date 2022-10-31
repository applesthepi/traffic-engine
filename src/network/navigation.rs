use std::collections::BTreeMap;

use nalgebra_glm::Vec2;

use crate::network::band::Band;

use super::{lane::LaneIdentity, NetworkAllocation, band::BandIdentity};

#[derive(Debug, Clone)]
pub struct ForwardLane {
	pub id: u32,
	pub length: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct GFCost {
	pub band_id: u32,
	pub g_cost: f64,
	pub f_cost: f64,
}

#[derive(Debug)]
pub struct Point {
	pub position: Vec2,
	pub accumulated_distance: f32,
}

pub struct Navigation {
	pub active_nav: u16,
	pub nav: Vec<BandIdentity>,
	pub nav_valid_band_lanes: Vec<Vec<u32>>,
	pub target_identity: LaneIdentity,
}

impl Navigation {
	fn reset_nav(&mut self) {
		self.active_nav = 0;
		self.nav.clear();
		self.nav_valid_band_lanes.clear();
	}

	pub fn renavigate(
		&mut self,
		allocation: &NetworkAllocation,
		active_identity: LaneIdentity
	) -> bool {
		
		println!("renav from:\n{:?}\nto:\n{:?}", active_identity, self.target_identity);
		self.reset_nav();
		let focus_h: Vec2 = allocation.lane(self.target_identity.lane).read().p4;
		let mut current_band_gf: u32 = 0;
		let mut band_gf_costs: Vec<GFCost> = Vec::new();
		let mut band_path_rev: BTreeMap<u32, u32> = BTreeMap::new();

		let mut band_current: u32 = 0;

		// INITIAL

		{
			let c_band_current = allocation.band(band_current);
			let ra_band_current = c_band_current.read();
			
			let c_clip_bw = allocation.clip(ra_band_current.src_clip);
			let ra_clip_bw = c_clip_bw.read();
			let lane_idx = ra_band_current.src_min;
			let c_lane = allocation.lane(ra_clip_bw.lanes_fixed[lane_idx as usize].fw);
			let ra_lane = c_lane.read();
			band_gf_costs.push(GFCost {
				band_id: active_identity.band,
				g_cost: 0.0,
				f_cost: ra_lane.p4.metric_distance(&focus_h).into()
			});
		}

		loop {
			let c_band_current = allocation.band(band_current);
			let ra_band_current = c_band_current.read();
			
			// MINIMUM

			if band_gf_costs.is_empty() {
				return false;
			}
			let min_cost = band_gf_costs.iter().enumerate().min_by(
				|a, b|
				(a.1.f_cost).partial_cmp(&b.1.f_cost).expect("invalid distance")
			).unwrap();
			println!("g: {} f: {}", min_cost.1.g_cost, min_cost.1.f_cost);
			current_band_gf = min_cost.0 as u32;
			let gf = band_gf_costs[current_band_gf as usize];

			
			band_current = gf.band_id;

			// DEAD END

			let c_clip_fw = allocation.clip(ra_band_current.dst_clip);
			let ra_clip_fw = c_clip_fw.read();
			// if fw_clip.fw_bands.is_empty() {
			// 	band_gf_costs.remove(current_band_gf as usize);
			// 	continue;
			// }

			// BRANCH MINIMUM

			for i in ra_clip_fw.fw_bands.iter() {
				let mut gf = GFCost {
					band_id: *i,
					g_cost: band_gf_costs[current_band_gf as usize].g_cost,
					f_cost: 0.0,
				};
				let band_lane_idx = allocation.band(*i).read().src_min;
				let c_lane_band = allocation.lane(ra_clip_fw.lanes_fixed[band_lane_idx as usize].fw);
				let ra_lane_band = c_lane_band.read();
				gf.g_cost += ra_lane_band.length as f64;
				gf.f_cost = gf.g_cost + ra_lane_band.p4.metric_distance(&focus_h) as f64;
				match band_gf_costs.iter().find(
					|x|
					x.band_id == gf.band_id
				) {
					Some(x) => {},
					None => { band_gf_costs.push(gf); },
				};

				// give future ref to current
				
				match band_path_rev.get_mut(&gf.band_id) {
					Some(x) => { *x = band_gf_costs[current_band_gf as usize].band_id; },
					None => {
						println!("new: {} old: {}", gf.band_id, band_gf_costs[current_band_gf as usize].band_id);
						band_path_rev.insert(gf.band_id, band_gf_costs[current_band_gf as usize].band_id);
					},
				};

				if *i == self.target_identity.band {
					self.update_nav(&allocation, &band_gf_costs, &band_path_rev);
					return true;
				}
			}

			band_gf_costs.remove(current_band_gf as usize);
		}
	}

	fn update_nav(
		&mut self,
		allocation: &NetworkAllocation,
		band_gf_costs: &Vec<GFCost>,
		band_path_rev: &BTreeMap<u32, u32>
	) {
		
		// RECONSTRUCT
		
		let mut current: u32 = self.target_identity.band;
		loop {
			self.nav.insert(0, BandIdentity {
				band: current,
				clip: 0,
			});
			current = match band_path_rev.get(&current) {
				Some(x) => *x,
				None => { break; },
			};
		}

		// VALIDATE LANES

		for nav in self.nav.iter_mut() {
			nav.clip = allocation.band(nav.band).read().src_clip;
		}

		for (i, nav) in self.nav.iter().enumerate() {
			let c_band = allocation.band(nav.band);
			let ra_band = c_band.read();
			let c_clip_fw = allocation.clip(ra_band.dst_clip);
			let ra_clip_fw = c_clip_fw.read();
			if (i + 1) == self.nav.len() {
				self.nav_valid_band_lanes.push(vec![self.target_identity.lane]);
			} else {
				let c_band_fw = allocation.band(self.nav[i + 1].band);
				let ra_band_fw = c_band_fw.read();
				let mut valid_lanes: Vec<u32> = Vec::new();
				// println!("mins: {} {} maxes: {} {}", (*fw_band).src_min, (*band).dst_min, (*fw_band).src_max, (*band).dst_max);
				for j in (ra_band_fw.src_min..(ra_band_fw.src_max + 1)).filter(
					|&x|
					x >= ra_band.dst_min && x <= (ra_band.dst_max + 0)
				) {
					valid_lanes.push(ra_clip_fw.lanes_fixed[j as usize].bw);
				}
				self.nav_valid_band_lanes.push(valid_lanes);
			}
		}
	}

	pub fn get_forward_lanes(
		&self,
		allocation: &NetworkAllocation,
		minimum_length: f32,
		mut active_lane: u32
	) -> Vec<ForwardLane> {
		let mut result: Vec<ForwardLane> = Vec::new();
		let mut total_distance: f32 = 0.0;
		let mut lane: u32 = active_lane;
		let mut nav_idx: u32 = 0;
		loop {
			let c_lane = allocation.lane(lane);
			let ra_lane = c_lane.read();
			{
				if ra_lane.fw_lanes.is_empty() {
					println!("lane.fw_lanes.is_empty()");
					break;
				}
				nav_idx += 1;
				let valid_lane = match ra_lane.fw_lanes.iter().enumerate().find(
					|x|
					self.nav_valid_band_lanes[nav_idx as usize].contains(&x.1.lane)
				) {
					Some(x) => x,
					None => {
						println!("no lanes fw are valid\n{:?} {:?} {:?}", self.nav, self.nav_valid_band_lanes, ra_lane.fw_lanes);
						break;
					},
				};
				active_lane = valid_lane.1.lane;
				lane = active_lane;
			} {
				result.push(ForwardLane {
					id: ra_lane.identity.lane,
					length: ra_lane.length,
				});
				total_distance += ra_lane.length;
			}
			if total_distance >= minimum_length {
				println!("total_distance >= minimum_length");
				break;
			}
		}
		result
	}
}