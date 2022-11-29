use std::collections::BTreeMap;

use glam::Vec2;

use super::{lane::LaneIdentity, NetworkAllocation, band::BandIdentity};

#[derive(Debug, Clone)]
pub struct ForwardLane {
	pub id: u32,
	pub length: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct GFCost {
	// pub band_id: u32,
	pub g_cost: f64,
	pub f_cost: f64,
}

#[derive(Debug, Default, Clone)]
pub struct Point {
	pub position: Vec2,
	pub accumulated_distance: f32,
}

#[derive(Debug)]
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

	pub async fn renavigate(
		&mut self,
		allocation: &NetworkAllocation,
		active_identity: LaneIdentity
	) -> bool {
		// G -> cost to start node
		// H -> estemated cost of start to end
		if active_identity.band == self.target_identity.band {
			println!("can not navigate to the same band");
			return false;
		}
		// println!("renav from:\n{:?}\nto:\n{:?}", active_identity, self.target_identity);
		self.reset_nav();
		let focus_h: Vec2 = allocation.lane(self.target_identity.lane).await.read().await.p4;
		let mut open_gf: Vec<u32> = Vec::new();
		let mut band_gf: BTreeMap<u32, GFCost> = BTreeMap::new();
		let mut preceding_gf: BTreeMap<u32, u32> = BTreeMap::new();
		// let mut band_path_rev: BTreeMap<u32, u32> = BTreeMap::new();

		let mut band_current: u32 = active_identity.band;

		// INITIAL
 
		let c_lane_band = allocation.lane(active_identity.lane).await;
		let ra_lane_band = c_lane_band.read().await;
		open_gf.push(active_identity.band);
		band_gf.insert(
			active_identity.band,
			GFCost {
				g_cost: 0.0,
				f_cost: ra_lane_band.p4.distance(focus_h) as f64
			}
		);
		drop(ra_lane_band);
		drop(c_lane_band);

		while !open_gf.is_empty() {

			// MIN

			let min_cost = open_gf.iter().enumerate().min_by(
				|a, b|
				(band_gf[a.1].f_cost).partial_cmp(&band_gf[b.1].f_cost).expect("invalid distance")
			).unwrap();
			let band_min = *min_cost.1;
			if band_min == self.target_identity.band {
				let preceding_band_id = preceding_gf[&band_min];
				let c_band_prev = allocation.band(preceding_band_id).await;
				let ra_band_prev = c_band_prev.read().await;
				let c_clip_prev = allocation.clip(ra_band_prev.dst_clip).await;
				let ra_clip_prev = c_clip_prev.read().await;
				let lf_target_lane = ra_clip_prev.lanes_fixed.iter().enumerate().find(
					|x|
					{
						let f = x.1;
						for i in 0..f.fw_count {
							if f.fw[i as usize] == self.target_identity.lane {
								return true;
							}
						}
						false
					}
				).expect("target lane does not exist");
				if (lf_target_lane.0 as u8) >= ra_band_prev.dst_min &&
					(lf_target_lane.0 as u8) <= ra_band_prev.dst_max {
					self.update_nav(&allocation, &preceding_gf, &active_identity).await;
					return true;
				}
				open_gf.remove(min_cost.0);
				println!("reached invalid finish (impossible lane) from: {} to: {}", preceding_band_id, band_min);
				continue;
			}
			// println!("current: id: {} costs: {:?}", band_min, band_gf[&band_min]);
			open_gf.remove(min_cost.0);
			
			// BRANCH FROM CURRENT

			let gf = band_gf[&band_min];
			let c_band_current = allocation.band(band_min).await;
			let ra_band_current = c_band_current.read().await;
			let c_clip_fw = allocation.clip(ra_band_current.dst_clip).await;
			let ra_clip_fw = c_clip_fw.read().await;
			// let current_gf_cost = band_gf_costs[current_band_gf as usize];
			for i in ra_clip_fw.fw_bands.iter() {
				let band_lane_idx = allocation.band(*i).await.read().await.src_min;
				let c_lane_band = allocation.lane(ra_clip_fw.lanes_fixed[band_lane_idx as usize].fw[0]).await;
				let ra_lane_band = c_lane_band.read().await;
				let pos_g_cost: f64 = gf.g_cost + ra_lane_band.length as f64;
				let fw_band_g_cost: f64 = match band_gf.get(i) {
					Some(x) => x.g_cost,
					None => f64::INFINITY
				};
				if pos_g_cost < fw_band_g_cost {
					if let Some(x) = preceding_gf.get_mut(i) {
						*x = band_min;
					} else {
						preceding_gf.insert(*i, band_min);
					}
					// println!("from: {} to: {}", band_min, *i);
					let new_gf = GFCost {
						g_cost: pos_g_cost,
						f_cost: pos_g_cost + ra_lane_band.p4.distance(focus_h) as f64
					};
					if let Some(x) = band_gf.get_mut(i) {
						*x = new_gf;
					} else {
						band_gf.insert(*i, new_gf);
					}
					if !open_gf.contains(i) {
						open_gf.push(*i);
					}
				}
			}
		}

		return false;
	}

	async fn update_nav(
		&mut self,
		allocation: &NetworkAllocation,
		preceding_gf: &BTreeMap<u32, u32>,
		active_identity: &LaneIdentity
	) {
		
		// RECONSTRUCT

		// println!("{:?}", preceding_gf);
		
		let mut current: u32 = self.target_identity.band;
		loop {
			self.nav.insert(0, BandIdentity {
				band: current,
				clip: 0,
			});
			current = match preceding_gf.get(&current) {
				Some(x) => *x,
				None => { break; },
			};
			if current == active_identity.lane {
				break;
			}
		}

		// VALIDATE LANES

		for nav in self.nav.iter_mut() {
			nav.clip = allocation.band(nav.band).await.read().await.src_clip;
		}

		for (i, nav) in self.nav.iter().enumerate() {
			let c_band = allocation.band(nav.band).await;
			let ra_band = c_band.read().await;
			let c_clip_fw = allocation.clip(ra_band.dst_clip).await;
			let ra_clip_fw = c_clip_fw.read().await;
			if (i + 1) == self.nav.len() {
				self.nav_valid_band_lanes.push(vec![self.target_identity.lane]);
			} else {
				let c_band_fw = allocation.band(self.nav[i + 1].band).await;
				let ra_band_fw = c_band_fw.read().await;
				let mut valid_lanes: Vec<u32> = Vec::new();
				// println!("mins: {} {} maxes: {} {}", (*fw_band).src_min, (*band).dst_min, (*fw_band).src_max, (*band).dst_max);
				for j in (ra_band_fw.src_min..(ra_band_fw.src_max + 1)).filter(
					|&x|
					x >= ra_band.dst_min && x <= (ra_band.dst_max + 0)
				) {
					let lane_fixed = &ra_clip_fw.lanes_fixed[j as usize];
					for l in 0..lane_fixed.bw_count {
						valid_lanes.push(lane_fixed.bw[l as usize]);
					}
				}
				self.nav_valid_band_lanes.push(valid_lanes);
			}
		}
	}

	pub async fn get_forward_lanes(
		&self,
		allocation: &NetworkAllocation,
		minimum_length: f32,
		active_lane: u32
	) -> Vec<ForwardLane> {
		let mut result: Vec<ForwardLane> = Vec::new();
		let mut total_distance: f32 = 0.0;
		let mut lane: u32 = active_lane;
		let mut nav_idx: u32 = self.active_nav as u32;
		loop {
			if nav_idx >= self.nav_valid_band_lanes.len() as u32 {
				break;
			}
			let c_lane = allocation.lane(lane).await;
			let ra_lane = c_lane.read().await;
			{
				if ra_lane.fw_lanes.is_empty() {
					println!("lane.fw_lanes.is_empty()");
					break;
				}
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
				lane = valid_lane.1.lane;
				nav_idx += 1;
			} {
				drop(ra_lane);
				drop(c_lane);
				let c_lane = allocation.lane(lane).await;
				let ra_lane = c_lane.read().await;
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